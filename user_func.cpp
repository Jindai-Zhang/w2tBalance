#include "user_func.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// 本文件分为三部分：控制辅助函数、配置/按键回调、每周期平衡计算。
// 保持模型构型不变时，主要调参入口为 user_func.h 的 BalanceParameters，
// 以及下面的腿关节 PD 数组；不要只改增益而忽略单位、轴向和传动比。
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0; // 1° 对应的弧度值：角度乘它、弧度除它
// 名称必须同时与 Bitbot XML 的 device 和模型 XML 的 joint/actuator 对应。
// 顺序对应设备 id=1~10：右侧五个关节在前、左侧五个关节在后。
constexpr std::array<const char*, 10> kNames = {
    "rhip_roll", "rfem_pitch", "rtib_pitch", "rATS_pitch", "rNUT_slide",
    "lhip_roll", "lfem_pitch", "ltib_pitch", "lATS_pitch", "lNUT_slide"};
// 单侧参考构型：髋 0°、股骨 30°、胫骨 -60°、ATS 初始 30°、滑块 0 m。
// 下标 3 的 ATS 不参与腿部位置保持，平衡时必须能连续转动；其 30° 仅为构型记录。
constexpr std::array<double, 5> kPose = {0.0, 30.0*kDeg, -60.0*kDeg, 30.0*kDeg, 0.0};
// 在力矩模式下显式计算关节 PD，替代示例 XML 中统一的 28000 位置增益。
// 转动关节 Kp/Kd 单位为 Nm/rad、Nm/(rad/s)；滑块为 N/m、N/(m/s)。
// 增大 Kp 可减小负载下的位置偏差，但也可能加剧接触振荡，需配合 Kd 验证。
constexpr std::array<double, 5> kJointKp = {800.0, 1200.0, 1600.0, 0.0, 20000.0};
constexpr std::array<double, 5> kJointKd = {30.0, 40.0, 50.0, 0.0, 100.0};
// 腿关节的物理输出限幅：转动关节 Nm、滑块 N；轮端另用 params 中的限制。
constexpr std::array<double, 5> kJointLimit = {100.0, 100.0, 200.0, 40.0, 100.0};

// 前端按下/松开分别发送 Down=1、Up=2；仅响应松开，避免长按反复初始化控制器。
bool KeyReleased(bitbot::EventValue value)
{
  return value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up);
}

// 对称限幅辅助函数，所有调用处的 limit 均应为非负幅值。
double Clamp(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

// 清零设备目标和记录的命令；物理 ctrl 在框架下一次 WriteBus 时更新。
// 故障清零不经过正常控制的力矩斜率限制，避免故障后仍缓慢释放输出。
void ZeroTorques(UserData& u)
{
  u.command.fill(0.0);
  for (auto* joint : u.joints)
    if (joint) joint->SetTargetTorque(0.0);
}

// 输入 torque 是关节侧的 Nm（滑块为 N），而 Bitbot SetTargetTorque 直接写 ctrl。
// 当前 motor 的关节输出 = gear * ctrl，因此 ATS 的轮端力矩必须先除以 6。
// 统一在此进行传动换算和模型 ctrlrange 限幅，避免各控制环重复或漏做换算。
void SetJointTorque(UserData& u, std::size_t index, double torque)
{
  const int actuator = u.actuator_ids[index];
  double ctrl = torque / u.gears[index];
  if (u.model->actuator_ctrllimited[actuator])
    ctrl = std::clamp(ctrl, u.model->actuator_ctrlrange[2*actuator],
                           u.model->actuator_ctrlrange[2*actuator+1]);
  // 记录实际下发命令对应的关节侧输出，供监测、饱和判断和下一周期限斜率使用。
  u.command[index] = ctrl * u.gears[index];
  u.joints[index]->SetTargetTorque(ctrl);
}

// 发布到前端及 CSV；字段名须与 user_func.h 中 Kernel 的模板字符串一致。
// 这里的轮端力矩是控制命令，不是传感器测得的实际力矩。
void Publish(Kernel::ExtraData& out, const UserData& u, bool active)
{
  out.Set<"time_s">(u.data->time);
  out.Set<"pitch_deg">(u.imu->GetPitch() / kDeg);
  out.Set<"roll_deg">(u.imu->GetRoll() / kDeg);
  out.Set<"pitch_target_deg">(u.pitch_target / kDeg);
  out.Set<"yaw_deg">(u.imu->GetYaw() / kDeg);
  out.Set<"yaw_target_deg">(u.yaw_target / kDeg);
  out.Set<"wheel_position_m">(u.position);
  out.Set<"wheel_velocity_mps">(u.velocity);
  out.Set<"right_wheel_Nm">(u.command[3]);
  out.Set<"left_wheel_Nm">(u.command[8]);
  out.Set<"balance_active">(static_cast<std::uint32_t>(active));
  out.Set<"fault_code">(static_cast<std::uint32_t>(u.fault_code));
}

// 故障处理：锁存故障 -> 清积分和全部输出 -> 向内核发送一次 STOP。
// STOP 通过队列处理；本周期仍将零命令写回总线，不依赖停止事件立即生效。
void Trip(const bitbot::KernelInterface& kernel, UserData& u, int code)
{
  if (!u.fault)
    std::cerr << "Balance stopped, fault " << code
              << ". Restart the backend to restore the initial pose.\n";
  u.fault = true;
  u.fault_code = code;
  u.initialized = false;
  u.velocity_integral = 0.0;
  ZeroTorques(u);
  if (!u.stop_requested)
  {
    kernel.EmitEvent(static_cast<bitbot::EventId>(bitbot::KernelEvent::STOP), 0);
    u.stop_requested = true;
  }
}

// 在任何控制计算前拦截 NaN/Inf，防止非有限值经 PID、限幅传播到执行器。
bool SensorsFinite(const UserData& u)
{
  if (!std::isfinite(u.imu->GetPitch()) || !std::isfinite(u.imu->GetRoll()) ||
      !std::isfinite(u.imu->GetYaw()) || !std::isfinite(u.imu->GetGyroX()) ||
      !std::isfinite(u.imu->GetGyroY()) || !std::isfinite(u.imu->GetGyroZ()))
    return false;
  for (auto* joint : u.joints)
    if (!std::isfinite(joint->GetActualPosition()) || !std::isfinite(joint->GetActualVelocity()))
      return false;
  return true;
}

double WheelCoordinate(const UserData& u)
{
  // 双侧股骨、胫骨、ATS 关节正轴均为 -Y，IMU 正俯仰为绕 +Y 转动。
  // 等效轮转角 = 机身俯仰 - 左右平均的(股骨角 + 胫骨角 + ATS角)。
  // 乘等效半径得到前向里程；补偿机身/腿部转角，避免将姿态摆动误认为行进。
  // 这是固定构型、小姿态、近似无滑动条件下的估计，分段轮面或打滑会引入误差。
  double angle = u.imu->GetPitch();
  for (std::size_t j : {1u, 2u, 3u, 6u, 7u, 8u})
    angle -= 0.5 * u.joints[j]->GetActualPosition();
  return u.params.wheel_radius * angle;
}

// 对上述轮里程关系求时间导数，直接使用角速度反馈，避免位置数值差分放大噪声。
double WheelVelocity(const UserData& u, double pitch_rate)
{
  double rate = pitch_rate;
  for (std::size_t j : {1u, 2u, 3u, 6u, 7u, 8u})
    rate -= 0.5 * u.joints[j]->GetActualVelocity();
  return u.params.wheel_radius * rate;
}
} // 匿名命名空间：辅助函数和常量只在本文件内使用

void ConfigFunc(const bitbot::MujocoBus& bus, UserData& u)
{
  // 框架会在 UpdateDevices 之前调用这里，因此只绑定指针/模式，不采集站立零点。
  // 重置全部用户状态；实际位移原点和航向参考留到首次 StateBalance 时捕获。
  u = UserData{};
  u.model = bus.GetMujocoModel();
  u.data = bus.GetMujocoData();
  if (!u.model || !u.data)
    throw std::runtime_error("w2tROBOT MuJoCo model was not loaded");
  if (u.model->opt.timestep <= 0.0 || u.model->opt.timestep > 0.005)
    throw std::runtime_error("Balance requires a simulation timestep in (0, 0.005] s");
  // 同时核对设备 id、名称及 actuator -> joint 关联，避免把力矩施加到错误关节。
  for (std::size_t j = 0; j < u.joints.size(); ++j)
  {
    const auto device = bus.GetDevice<bitbot::MujocoJoint>(static_cast<unsigned int>(j+1));
    if (!device || (*device)->Name() != kNames[j])
      throw std::runtime_error(std::string("Missing/mismatched joint: ") + kNames[j]);
    u.joints[j] = *device;
    const int actuator = mj_name2id(u.model, mjOBJ_ACTUATOR, kNames[j]);
    const int joint = mj_name2id(u.model, mjOBJ_JOINT, kNames[j]);
    if (actuator < 0 || joint < 0 || u.model->actuator_trnid[2*actuator] != joint)
      throw std::runtime_error(std::string("Invalid actuator mapping: ") + kNames[j]);
    u.actuator_ids[j] = actuator;
    // MuJoCo 为每个执行器保存 6 个传动参数；当前单自由度 motor 使用第一个。
    u.gears[j] = u.model->actuator_gear[6*actuator];
    if (!std::isfinite(u.gears[j]) || std::abs(u.gears[j]) < 1e-9)
      throw std::runtime_error("Invalid actuator gear");
    // 在运行时统一改为力矩模式，XML 仍负责设备映射和初始姿态。
    // 后续腿部 PD 与轮端平衡力矩都由本文件生成，不再叠加示例位置伺服。
    u.joints[j]->SetMode(bitbot::MujocoJointMode::TORQUE);
  }
  // IMU 姿态来自 site_imu，角速度来自 gyro_imu；接口输出 rad 和 rad/s。
  const auto imu = bus.GetDevice<bitbot::MujocoImu>(11);
  if (!imu || (*imu)->Name() != "imu" ||
      mj_name2id(u.model, mjOBJ_SITE, "site_imu") < 0 ||
      mj_name2id(u.model, mjOBJ_SENSOR, "gyro_imu") < 0)
    throw std::runtime_error("Missing IMU/gyro_imu/site_imu");
  u.imu = *imu;
  ZeroTorques(u);
  std::cout << "Fixed-leg cascaded PID ready, timestep=" << u.model->opt.timestep
            << " s; ATS gears=" << u.gears[3] << ", " << u.gears[8] << ".\n";
}

// 5：解除控制而非“暂停并保持站立”；机器人没有主动支撑，可能随即倾倒。
std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData& u)
{
  if (!KeyReleased(value)) return std::nullopt;
  ZeroTorques(u);
  u.initialized = false;
  u.velocity_integral = 0.0;
  std::cout << "Disarmed: all joint torques are zero; robot is not supported.\n";
  return static_cast<bitbot::StateId>(States::Waiting);
}

// b：只请求切换状态；当前姿态是否允许启用，由下一周期的新传感器读数判断。
// 故障锁存后禁止重新启用，避免绕过停机保护。
std::optional<bitbot::StateId> EventBalance(bitbot::EventValue value, UserData& u)
{
  if (!KeyReleased(value) || u.fault) return std::nullopt;
  u.initialized = false;
  return static_cast<bitbot::StateId>(States::Balance);
}

void StateWaiting(const bitbot::KernelInterface&, Kernel::ExtraData& out, UserData& u)
{
  // 每周期维持零输出并清历史；重新进入 Balance 时重新捕获参考值。
  ZeroTorques(u);
  u.initialized = false;
  u.velocity_integral = 0.0;
  u.last_time = -1.0;
  Publish(out, u, false);
}

void StateBalance(const bitbot::KernelInterface& kernel, Kernel::ExtraData& out, UserData& u)
{
  // 1. 先处理锁存故障和无效输入，再允许任何控制计算或输出。
  if (u.fault) { ZeroTorques(u); Publish(out, u, false); return; }
  if (!SensorsFinite(u))
  {
    Trip(kernel, u, 1); Publish(out, u, false); return;
  }
  // 2. 用实际仿真时间差计算 dt，不能用墙钟时间或示例中固定的 0.004 s。
  // 首次采样使用模型 timestep；后续也能正确处理框架偶尔跳过某个时间步的情况。
  const double now = u.data->time;
  double dt = u.model->opt.timestep;
  if (!std::isfinite(now) || (u.last_time >= 0.0 && now < u.last_time))
  {
    Trip(kernel, u, 4); Publish(out, u, false); return;
  }
  // 暂停时的 mj_forward 或同一时刻的重复回调不能重复积分。
  if (u.initialized && now == u.last_time) return;
  if (u.last_time >= 0.0)
  {
    dt = now - u.last_time;
    // 过大间隔会使积分和斜率限制失去原有含义，按时钟异常停机。
    if (dt <= 0.0 || dt > 0.02)
    {
      Trip(kernel, u, 4); Publish(out, u, false); return;
    }
  }
  u.last_time = now;
  const auto& p = u.params;
  const double pitch = u.imu->GetPitch();
  const double roll = u.imu->GetRoll();
  // 3. 将机体系陀螺仪角速度转换为欧拉俯仰角速度；直立附近约等于 gyro Y。
  const double pitch_rate = std::cos(roll)*u.imu->GetGyroY() - std::sin(roll)*u.imu->GetGyroZ();
  const double raw_velocity = WheelVelocity(u, pitch_rate);
  // 启用许可范围比运行中停机阈值更严格，避免在明显倾斜时突然接入控制。
  if (std::abs(pitch) > p.fall_pitch || std::abs(roll) > p.fall_roll ||
      (!u.initialized && (std::abs(pitch) > p.arming_pitch || std::abs(roll) > p.arming_roll)))
  {
    Trip(kernel, u, 2); Publish(out, u, false); return;
  }
  // 超速检测使用未滤波估计，避免低通延迟掩盖异常。
  if (std::abs(raw_velocity) > p.speed_limit)
  {
    Trip(kernel, u, 3); Publish(out, u, false); return;
  }
  // 4. 每次启用时捕获“当前位置/当前航向”，目标是围绕该参考位置保持平衡。
  if (!u.initialized)
  {
    u.position_origin = WheelCoordinate(u);
    u.position = 0.0;
    u.velocity = raw_velocity;
    u.velocity_integral = 0.0;
    u.yaw_target = u.imu->GetYaw();
    u.initialized = true;
  }
  u.position = WheelCoordinate(u) - u.position_origin;
  // 一阶低通：alpha = dt/(时间常数+dt)，降低接触切换对速度外环的扰动。
  u.velocity += dt/(p.velocity_filter_time + dt) * (raw_velocity - u.velocity);

  // 5. 位置 P 环：前向偏移为正时，给出负目标速度，使机器人缓慢回到启用位置附近。
  // 默认允许 +/-5 mm 编码器位置偏差。静止时若仍追赶毫米级误差，速度 PI 会不断积累，
  // 数十秒后可能触发一次慢速移动；连续死区在边界处连续接入，不突然跳变目标速度。
  // 容差内目标速度为零，仍保留速度反馈及既有积分补偿，并不关闭平衡或强制清积分。
  const double position_error = u.position - Clamp(u.position, p.position_deadband);
  const double velocity_target = Clamp(-p.position_kp*position_error, p.velocity_target_limit);
  const double velocity_error = velocity_target - u.velocity;
  // 速度 PI 环：目标比实际更快时给出正俯仰目标，让机身向 +X 前倾。
  // 速度误差积分单位为 m，先限制在 +/-1.5 m，再判断是否接受本周期积分。
  const double integral_candidate = Clamp(u.velocity_integral + velocity_error*dt, 1.5);
  // 初始机身竖直不等于重心已对齐轮轴。提供已标定的平衡俯仰前馈，
  // 避免靠速度积分经历一次明显后退后，才逐步建立所需的站立俯仰角。
  const double desired_pitch = p.pitch_equilibrium + p.velocity_kp*velocity_error + p.velocity_ki*integral_candidate;
  // 抗积分饱和：上次轮端命令达到控制器限幅时冻结积分；未饱和时，只有目标俯仰
  // 在范围内，或误差正将它拉回范围内，才接受候选积分。此处不包含打滑检测。
  const bool wheel_saturated = std::abs(u.command[3]) >= p.wheel_torque_limit - 1e-6 ||
                               std::abs(u.command[8]) >= p.wheel_torque_limit - 1e-6;
  if (!wheel_saturated && (std::abs(desired_pitch) < p.pitch_target_limit ||
                           desired_pitch*velocity_error < 0.0))
    u.velocity_integral = integral_candidate;
  u.pitch_target = Clamp(p.pitch_equilibrium + p.velocity_kp*velocity_error +
                         p.velocity_ki*u.velocity_integral, p.pitch_target_limit);

  // 6. 俯仰 PD 内环：机身绕 +Y 正转表示向 +X 前倾，但双轮关节正轴是 -Y，
  // 因此前倾误差为正时，需要负轮端力矩让轮子向前追上重心；不能随意翻转符号。
  // common 为每个轮子的公共力矩分量；平衡前馈处理主要偏置，速度 PI 修正剩余误差。
  const double common = -p.pitch_kp*(pitch-u.pitch_target) - p.pitch_kd*pitch_rate;
  // 航向误差取最短角差，避免跨越 +/-pi 时突变；差动力矩单侧限幅为 5 Nm。
  // 后面按“右轮加、左轮减”分配，结合 -Y 轮轴方向抵消正偏航误差。
  const double yaw_error = std::remainder(u.imu->GetYaw()-u.yaw_target, 2.0*kPi);
  // 机体系 gyro Z 不是一般姿态下的欧拉偏航角速度；使用横滚/俯仰完成转换。
  // 前面的倾角检查保证 cos(pitch) 远离零。
  const double yaw_rate = (std::sin(roll)*u.imu->GetGyroY() +
                           std::cos(roll)*u.imu->GetGyroZ()) / std::cos(pitch);
  const double differential = Clamp(p.yaw_kp*yaw_error + p.yaw_kd*yaw_rate, 5.0);

  // 7. 腿部 PD 保持固定构型。j%5 使左右腿复用同一组参数，ATS 轮关节单独处理。
  for (std::size_t j = 0; j < u.joints.size(); ++j)
  {
    const std::size_t axis = j % 5;
    if (axis == 3) continue;
    double target = kPose[axis];
    // 两侧髋轴均为 -X，给相同符号的小幅横滚补偿；目标修正最多 +/-0.10 rad。
    // -0.5 为姿态比例补偿，-0.03 s 为角速度补偿；这里只修正髋目标，不改变腿长。
    if (axis == 0) target = Clamp(-0.5*roll - 0.03*u.imu->GetGyroX(), 0.10);
    const double torque = kJointKp[axis]*(target-u.joints[j]->GetActualPosition()) -
                          kJointKd[axis]*u.joints[j]->GetActualVelocity();
    SetJointTorque(u, j, Clamp(torque, kJointLimit[axis]));
  }
  // 8. 合成公共/差动力矩，先限幅，再按 Nm/s 限制每周期变化，最后换算为 ctrl。
  // 下标 3、8 分别为右/左 ATS；command 保存上次输出，是斜率限制的参考值。
  for (std::size_t j : {3u, 8u})
  {
    const double desired = Clamp(common + (j == 3 ? differential : -differential), p.wheel_torque_limit);
    const double torque = u.command[j] + Clamp(desired-u.command[j], p.wheel_torque_slew*dt);
    SetJointTorque(u, j, torque);
  }
  // 状态回调返回后，框架执行 WriteBus，把本轮所有目标统一送入 MuJoCo。
  Publish(out, u, true);
}

void FinishFunc(UserData& u)
{
  // 框架此时已经释放 mjModel/mjData，不能再访问 u.model/u.data 或调用 Publish。
  // 仅清理仍有效的设备目标；仿真已结束，这不是一次新的物理输出。
  ZeroTorques(u);
  std::cout << "Balance backend finished.\n";
}
