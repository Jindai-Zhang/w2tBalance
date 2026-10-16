#pragma once

#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#include <array>
#include <optional>

// 自定义事件使用 1000 以上的编号，避开 Bitbot 内置事件。
// 编号供后端状态机使用；前端通过 main.cpp 注册的字符串名称发送事件。
enum class Events : bitbot::EventId
{
  Wait = 1001,   // 5：解除控制，所有执行器目标力矩/推力清零
  Balance        // b：从 Waiting 请求进入平衡，姿态检查由 StateBalance 完成
};

// 用户状态与内核的 idle/stopped 状态分开；Waiting 不是站立保持状态。
enum class States : bitbot::StateId
{
  Waiting = 1001, // 零力矩解除控制，机器人会失去主动支撑
  Balance        // 固定腿构型下的原地平衡
};

// 控制器参数入口：位置 P -> 速度 PI -> 俯仰 PD，另加偏航 PD 抑制转向漂移。
// 内部统一采用 SI 单位：rad、rad/s、m、s、关节侧 Nm；滑块使用 N。
// 当前参数对应股骨 30°、胫骨 -60°、滑块 0 m 的固定腿轮式构型。
// 调参建议：先确认反馈方向及俯仰内环，再调整速度环，最后调整位置回归速度。
// 修改构型、质量、轮半径或采样周期后，需要重新验证，不能直接沿用原测试结论。
struct BalanceParameters
{
  double wheel_radius = 0.138;  // 等效滚动半径，m；用于编码器位移/速度估计
  double pitch_kp = 160.0;      // 每侧轮端姿态刚度，Nm/rad；过大会加剧接触振荡
  double pitch_kd = 25.0;       // 每侧轮端俯仰阻尼，Nm/(rad/s)
  double pitch_equilibrium = 0.026; // 固定构型的平衡俯仰前馈，rad（约 1.49°）；减少启动后移
  double velocity_kp = 0.12;    // 速度误差到目标俯仰角的比例增益，rad/(m/s)
  double velocity_ki = 0.04;    // 速度误差积分增益，rad/m；补偿静态偏置
  double position_kp = 0.25;    // 位移误差到目标速度的增益，1/s；控制返回起点的快慢
  double position_deadband = 0.005; // 编码器位置允许偏差，m；避免毫米级残差使速度积分长期累积
  double yaw_kp = 20.0;         // 航向误差到单侧差动力矩的增益，Nm/rad
  double yaw_kd = 5.0;          // 偏航角速度阻尼，Nm/(rad/s)
  double wheel_torque_limit = 40.0; // 单侧轮端力矩上限，Nm；尚未除以 gear
  double wheel_torque_slew = 500.0; // 单侧轮端力矩变化率上限，Nm/s
  double pitch_target_limit = 0.12; // 总俯仰目标限幅，rad，约 6.88°
  double velocity_target_limit = 0.20; // 位置环输出的目标速度上限，m/s
  double fall_pitch = 0.5235987755982988; // 运行中俯仰停机阈值，rad，30°
  double fall_roll = 0.3490658503988659;  // 运行中横滚停机阈值，rad，20°
  double arming_pitch = 0.1745329251994329; // 启用时俯仰许可范围，rad，10°
  double arming_roll = 0.1396263401595464;  // 启用时横滚许可范围，rad，8°
  double speed_limit = 2.5;     // 未滤波轮速估计的停机阈值，m/s
  double velocity_filter_time = 0.03; // 速度低通时间常数，s；越大越平滑但延迟越大
};

// 用户控制状态由 Bitbot 保存，并传给配置、事件及周期回调，避免散落的全局变量。
struct UserData
{
  // 数组下标 0~4 为右侧、5~9 为左侧；各侧依次为髋/股骨/胫骨/ATS轮/滑块。
  // 下标从 0 开始，而 XML 中的设备 id 从 1 开始，获取设备时必须加 1。
  std::array<bitbot::MujocoJoint*, 10> joints{};
  bitbot::MujocoImu* imu = nullptr; // XML 中 id=11，姿态接口单位为 rad
  const mjModel* model = nullptr; // 模型参数：时间步长、传动比、控制量限制等
  mjData* data = nullptr;         // 仿真运行数据，用于读取实际仿真时间
  std::array<int, 10> actuator_ids{}; // 设备顺序到 MuJoCo actuator 索引的映射
  std::array<double, 10> gears{};     // 执行器到关节的传动系数；当前 ATS 为 6
  std::array<double, 10> command{};   // 上次下发并限幅后的关节侧命令，Nm 或 N；不是测量值
  BalanceParameters params{};       // 控制参数副本，便于无界面测试覆盖参数

  bool initialized = false;     // 是否已捕获本次平衡的位移原点和参考航向
  bool fault = false;           // 故障锁存；普通按键不清除，需重启后端
  bool stop_requested = false;  // 防止重复向内核队列发送 STOP
  int fault_code = 0;           // 0正常；1传感器非有限值；2姿态；3超速；4时钟异常
  double last_time = -1.0;      // 上次有效控制时间，s；-1 表示尚无历史采样
  double position_origin = 0.0; // 启用时的编码器位移估计，m
  double position = 0.0;        // 相对启用位置的位移，m；不是外部定位坐标
  double velocity = 0.0;        // 低通滤波后的前向速度估计，m/s
  double velocity_integral = 0.0; // 速度误差的时间积分，m
  double pitch_target = 0.0;    // 平衡俯仰前馈与速度 PI 合成后的限幅目标，rad
  double yaw_target = 0.0;      // 启用瞬间捕获的航向，rad，用于抑制偏航漂移
};

// 这些字符串同时定义前端 extra 列及 CSV 表头；新增监测量时同步修改 Publish。
// 控制计算使用弧度，仅发布 *_deg 时换算为度；左右轮 *_Nm 是轮端力矩命令。
using Kernel = bitbot::MujocoKernel<UserData,
    "time_s", "pitch_deg", "roll_deg", "pitch_target_deg", "yaw_deg", "yaw_target_deg",
    "wheel_position_m", "wheel_velocity_mps", "right_wheel_Nm", "left_wheel_Nm",
    "balance_active", "fault_code">;

// 配置回调绑定设备并设置运行模式；此时设备的实际位置可能尚未完成首次更新。
void ConfigFunc(const bitbot::MujocoBus& bus, UserData& user_data);
// 事件回调返回目标状态编号表示切换；返回 nullopt 表示只修改数据或忽略该事件。
std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventBalance(bitbot::EventValue value, UserData& user_data);
// 周期回调由框架在 ReadBus 之后、WriteBus 之前执行，不要在其中重复推进仿真。
void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);
void StateBalance(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);
// 结束回调执行时 MuJoCo 数据已释放，只能清理仍有效的用户/设备目标状态。
void FinishFunc(UserData& user_data);
