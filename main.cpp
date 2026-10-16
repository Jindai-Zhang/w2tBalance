#ifdef _WIN32
// 禁用 Windows 的 min/max 宏，避免与标准库的同名函数冲突。
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "user_func.h"

#ifdef _WIN32
// Windows 头文件必须放在 Bitbot 头文件之后，否则 SetPort 宏会把
// Backend::SetPort 改写为 SetPortA，造成链接失败；随后取消该宏定义。
#include <windows.h>
#ifdef SetPort
#undef SetPort
#endif
#endif

#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;
constexpr const char* kDefaultConfig = "bitbot_w2tROBOT.xml";

// 从指定目录逐级向父目录寻找默认配置；到达盘符根目录后结束搜索。
std::optional<fs::path> FindConfigAbove(fs::path directory)
{
  // 因此既可从项目根目录启动，也可从 build/bin/Debug 或 Release 启动。
  while (!directory.empty())
  {
    const auto candidate = directory / kDefaultConfig;
    if (fs::is_regular_file(candidate))
      return fs::canonical(candidate);
    const auto parent = directory.parent_path();
    if (parent == directory)
      break;
    directory = parent;
  }
  return std::nullopt;
}

fs::path ExecutableDirectory(const char* argv0)
{
#ifdef _WIN32
  // 通过 PATH 启动时 argv[0] 可能只有文件名，Windows API 可取得实际程序路径。
  std::vector<wchar_t> buffer(32768);
  const DWORD size = GetModuleFileNameW(nullptr, buffer.data(),
                                      static_cast<DWORD>(buffer.size()));
  if (size > 0 && size < buffer.size())
    return fs::path(std::wstring(buffer.data(), size)).parent_path();
#endif
  return fs::absolute(argv0).parent_path();
}

fs::path ResolveConfig(int argc, const char* const argv[])
{
  // 配置查找优先级：命令行显式路径 > 当前工作目录及其父目录 > 程序目录及其父目录。
  // 显式路径不存在时直接报错，避免意外加载另一份同名机器人配置。
  if (argc > 2)
    throw std::runtime_error("Usage: main_app [path/to/bitbot_w2tROBOT.xml]");
  if (argc == 2)
  {
    const fs::path requested(argv[1]);
    if (!fs::is_regular_file(requested))
      throw std::runtime_error("Configuration file does not exist: " + requested.string());
    return fs::canonical(requested);
  }
  if (auto found = FindConfigAbove(fs::current_path()))
    return *found;
  if (auto found = FindConfigAbove(ExecutableDirectory(argv[0])))
    return *found;
  throw std::runtime_error(
    "Cannot find bitbot_w2tROBOT.xml. Pass its path as the first argument.");
}

void PrintControls()
{
  // 控制台帮助与 backend.json 的键位保持一致；真正的按键由配套前端发送。
  std::cout
    << "\nW2T standing balance / Bitbot frontend (port from XML; default 12888)\n"
    << "  8      Start from kernel_idle; balance begins on key release.\n"
    << "  5      Disarm to waiting (zero torque; robot will not remain standing).\n"
    << "  b      Enable balance from waiting, if the posture is safe.\n"
    << "  m      Enable CSV recording in ./log/data (not a toggle).\n"
    << "  Space  Kernel stop; restart this process to run again.\n"
    << "Use the frontend control page for these keys, not the MuJoCo window.\n"
    << "To reset or reload the model, restart this backend process.\n\n";
}
}  // 匿名命名空间：这些启动辅助函数仅供本文件使用。

int main(int argc, char const* argv[])
{
  try
  {
    // 帮助查询不创建内核，也不启动仿真窗口或后端服务。
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"))
    {
      std::cout << "Usage: main_app [path/to/bitbot_w2tROBOT.xml]\n";
      PrintControls();
      return 0;
    }

    const auto config = ResolveConfig(argc, argv);
    // Bitbot 的模型和 backend.json 路径相对于 XML 解析，而 ./log 相对于工作目录。
    // 统一到配置所在目录，确保从不同位置启动时日志仍写入同一项目目录。
    fs::current_path(config.parent_path());
    std::cout << "Configuration: " << config.string() << '\n';
    PrintControls();

    Kernel kernel(config.string());
    // ConfigFunc 绑定模型设备并初始化控制参数；FinishFunc 在运行结束时收尾。
    kernel.RegisterConfigFunc(&ConfigFunc);
    kernel.RegisterFinishFunc(&FinishFunc);

    // 事件名必须与 backend.json 的 event 字段一致，回调负责修改目标或请求状态切换。
    // 不启用全局用户事件，防止从 kernel_idle 或 kernel_stopped 绕过启动/停止流程。
    // start、stop、enable_record 已由 Bitbot 内核注册，这里不重复注册。
    kernel.RegisterEvent("waiting", static_cast<bitbot::EventId>(Events::Wait), &EventWait);
    kernel.RegisterEvent("balance", static_cast<bitbot::EventId>(Events::Balance), &EventBalance);

    // 每个状态后面的事件列表是该状态允许响应的用户事件白名单。
    // waiting 为零力矩解除控制状态，只有 balance 事件可重新请求平衡。
    kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting,
      {static_cast<bitbot::EventId>(Events::Balance)});
    // balance 每个控制周期执行 StateBalance，并允许通过 waiting 事件解除控制。
    kernel.RegisterState("balance", static_cast<bitbot::StateId>(States::Balance), &StateBalance,
      {static_cast<bitbot::EventId>(Events::Wait)});

    // 前端按下并松开 8 后，内核同时启动物理仿真并进入这里指定的首个用户状态。
    // 首状态设为平衡，避免先进入零力矩 waiting 而让机器人失去支撑。
    kernel.SetFirstState(static_cast<bitbot::StateId>(States::Balance));
    kernel.Run();
    return 0;
  }
  catch (const std::exception& error)
  {
    // 配置路径或初始化检查失败时给出原因，并以非零退出码结束程序。
    std::cerr << "W2T startup failed: " << error.what() << '\n';
    return 1;
  }
}
