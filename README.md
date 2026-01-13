# RobotAPI

面向动捕数据的机器人接口封装，提供静态库、DLL 以及演示程序。

## 目录结构（关键）
- `librobot/`：机器人核心算法与实现，生成 `librobot.lib`
- `mcp_robot.cpp`/`mcp_robot.h`：封装 MocapApi 到内部 `eba::Robot`
- `robot_api_exports.*`：对外 C 接口（`RA_*` 函数），生成 DLL
- `robot_api_demo.cpp`：示例程序，演示如何消费动捕事件并驱动机器人
- `third_party/MocapApi/`：外部动捕 SDK（头文件和运行时 DLL）
- `build.bat`：一键配置并构建（VS2022 x64，Release）

## 构建
要求：CMake、VS2022（MSVC），Windows x64。

```bat
:: 在仓库根目录执行
build.bat
```
生成产物（默认 `Release`）：
- `build/librobot/Release/librobot.lib`
- `build/Release/robot_api.lib`（静态库）
- `build/Release/robot_api.dll` + 导入库（共享库，导出 RA_* 接口）
- `build/Release/robot_api_demo.exe`
- `MocapApi.dll` 自动拷贝至输出目录

## 运行 demo
1. 确保输出目录下有 `robot_api.dll` 与 `MocapApi.dll`。
2. 将 `retarget.json`（机器人映射配置）放在 `robot_api_demo.exe` 可访问路径，默认 demo 使用 `../retarget.json`。
3. 启动服务或设备数据源后运行 `robot_api_demo.exe`，可选参数：
   ```
   robot_api_demo.exe [server_ip] [server_port] [log_buffer_kb]
   ```
4. 终端输出会打印 Avatar 更新、机器人根节点姿态和指定关节角度等信息。

## DLL 对外接口（`robot_api_exports.h`）
核心 C 接口（使用 `__declspec(dllexport)` 导出）：
- 创建/销毁：`RA_CreateRobot` / `RA_DestroyRobot`
- 配置：`RA_SetRobotFPS`
- 更新：`RA_UpdateRobot`
- 查询：`RA_GetRobotRootPosition`，`RA_GetRobotRootRotation`，`RA_GetRobotRawJointAngle`，`RA_GetRobotRetargetJointAngle`
- 序列化：`RA_GetRobotRosFrameJson`
- 推进：`RA_RunRobotStep`，`RA_RunRobotStep1`
- 其他：`RA_GetRobotSlideSpeed`，`RA_GetRobotSlideHeight`

示例（伪代码）：
```cpp
#include "robot_api_exports.h"
using namespace MocapApi;

MCPRobotHandle_t h = 0;
RA_CreateRobot(retargetJson, &h);
RA_SetRobotFPS(100, h);
RA_UpdateRobot(avatarHandle, h);
float pos[3], rot[4];
RA_GetRobotRootPosition(pos, pos+1, pos+2, h);
RA_GetRobotRootRotation(rot, rot+1, rot+2, rot+3, h);
RA_DestroyRobot(h);
```
Demo3 运行
.\Release\robot_api_demo3.exe D:\Git\RobotAPI\walk_1_001_C_y_lanqiu_04_chr01.bvh D:\Git\RobotAPI\retarget.json 60
## 注意事项
- 运行时仍依赖 MocapApi 提供的 avatar/joint 数据接口；请保持 SDK DLL 在可搜索路径。
- 源码包含 UTF-8 内容，若出现代码页警告，可将相关文件保存为 UTF-8。
- 如需自定义生成类型（Debug/其他架构），可修改 `build.bat` 中的 `CONFIG`/`ARCH`。
