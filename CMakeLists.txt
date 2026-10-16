cmake_minimum_required(VERSION 3.14)

project(bitbot)

# 使用 dependencies 目录内的依赖压缩包，保持与当前工程附带的 Bitbot 版本一致。
set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

include(bitbot.cmake)

BitbotAddDependency(
  NAME           bitbot_mujoco
  FILE_NAME      "bitbot_mujoco-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# 主程序由启动入口和控制回调组成，通过 BitbotMujoco 接入仿真、设备总线及前端后端。
add_executable(main_app main.cpp user_func.cpp)
target_link_libraries(main_app
  BitbotMujoco
)
target_compile_options(main_app PUBLIC)
# Bitbot 的编译期监视字段等接口使用 C++20 特性。
target_compile_features(main_app PUBLIC cxx_std_20)

# 可选的无界面集成检查：复用正式模型、设备总线及 user_func.cpp 控制代码。
# 默认关闭；需要验证调参效果时，在 CMake 配置阶段加入 -DBITBOT_BUILD_BALANCE_TESTS=ON。
option(BITBOT_BUILD_BALANCE_TESTS "Build headless W2T balance integration checks" OFF)
if(BITBOT_BUILD_BALANCE_TESTS)
  enable_testing()
  add_executable(balance_smoke tests/balance_smoke.cpp user_func.cpp)
  target_include_directories(balance_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(balance_smoke PRIVATE BitbotMujoco)
  target_compile_features(balance_smoke PRIVATE cxx_std_20)
  # 注册到 CTest；测试工作目录固定为项目根目录，确保能找到模型及相关配置文件。
  add_test(NAME balance_smoke COMMAND balance_smoke)
  set_tests_properties(balance_smoke PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 180)
endif()
