#include <iostream>
#include <string>

#include "robot_api.h"

#include <chrono>
#include <thread>
#include <iomanip>

int main() {
  // 这里只做最小可运行示例：验证能成功链接到 robot_api.lib 并调用导出函数
  // 实际使用时请替换为真实的 jsonConfigStr（例如读取 retarget.json）。
  const std::string jsonConfigStr = "{}";

  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::cout << "[demo2] start: " << std::put_time(std::localtime(&now_c), "%F %T")
            << " thread=" << std::this_thread::get_id() << "\n";

  std::cout << "[demo2] jsonConfig: '" << jsonConfigStr << "'\n";

  MocapApi::MCPRobotHandle_t robot = 0;
  std::cout << "[demo2] calling RA_CreateRobot...\n";
  auto err = RA_CreateRobot(jsonConfigStr.c_str(), &robot);
  std::cout << "[demo2] RA_CreateRobot -> code=" << static_cast<int>(err) << ", handle=" << robot << "\n";

  auto errorToString = [](MocapApi::EMCPError e) -> std::string {
    if (e == MocapApi::Error_None) return "Error_None";
    return std::string("Error_") + std::to_string(static_cast<int>(e));
  };

  if (err != MocapApi::Error_None) {
    std::cout << "[demo2] failed to create robot: " << errorToString(err) << "\n";
    return static_cast<int>(err);
  }

  if (robot == 0) {
    std::cout << "[demo2] warning: RA_CreateRobot returned handle 0\n";
  }

  std::cout << "[demo2] calling RA_SetRobotFPS(60)...\n";
  err = RA_SetRobotFPS(60, robot);
  std::cout << "[demo2] RA_SetRobotFPS -> code=" << static_cast<int>(err)
            << " (" << errorToString(err) << ")\n";

  std::cout << "[demo2] sleeping 200ms to simulate work...\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "[demo2] calling RA_DestroyRobot...\n";
  err = RA_DestroyRobot(robot);
  std::cout << "[demo2] RA_DestroyRobot -> code=" << static_cast<int>(err)
            << " (" << errorToString(err) << ")\n";

  std::cout << "[demo2] exiting with code " << static_cast<int>(err) << "\n";
  return static_cast<int>(err);
}
