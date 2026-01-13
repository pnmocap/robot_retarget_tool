#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <MocapApi.h>
#include <thread>
#include <windows.h>
#include "robot_api.h"


using namespace MocapApi;
// 服务IP地址
std::string strServerIP = "127.0.0.1";
// 服务端口号
uint32_t strServerPort = 7012;
/// 输出到文件的buffer
bool bPrint2Buffer = false;
// 输出缓冲区长度
size_t nBufferLen = 1024 * 200 * 1024;
// 输出缓冲区首地址
char* logBuffer = nullptr;
// 输出缓冲区首偏移
int logOffset = 0;

// 全局错误码
EMCPError error = EMCPError::Error_None;

#define PrintInfo(format, ...)                                           \
  if (bPrint2Buffer) {                                                   \
    logQuit();                                                           \
    logOffset += snprintf(logBuffer + logOffset, nBufferLen - logOffset, \
format, ##__VA_ARGS__);                        \
  } else                                                                 \
    printf(format, ##__VA_ARGS__);

void logQuit() {
  if (logOffset >= nBufferLen - 200) {
    std::ofstream outfile;
    outfile.open("./record.log");
    outfile << logBuffer;
    outfile.close();
    exit(1);
  }
}

static const char* getErrorMsg(EMCPError error)
{
    switch (error) 
    {
    case Error_None:
        return "中文";
    case Error_MoreEvent:
        return "存在更多事件";
    case Error_InsufficientBuffer:
        return "缓冲区不足";
    case Error_InvalidObject:
        return "无效对象";
    case Error_InvalidHandle:
        return "无效句柄";
    case Error_InvalidParameter:
        return "无效参数";
    case Error_NotSupported:
        return "不支持的操作";
    case Error_IgnoreUDPSetting:
        return "忽略UDP设置";
    case Error_IgnoreTCPSetting:
        return "忽略TCP设置";
    case Error_IgnoreBvhSetting:
        return "忽略BVH设置";
    case Error_JointNotFound:
        return "关节未找到";
    case Error_WithoutTransformation:
        return "无变换数据";
    case Error_NoneMessage:
        return "无消息";
    case Error_NoneParent:
        return "无父节点";
    case Error_NoneChild:
        return "无子节点";
    case Error_AddressInUse:
        return "地址已被占用";
    case Error_ServerNotReady:
        return "服务器未就绪";
    case Error_ClientNotReady:
        return "客户端未就绪";
    case Error_IncompleteCommand:
        return "命令不完整";
    case Error_UDP:
        return "UDP错误";
    case Error_TCP:
        return "TCP错误";
    case Error_QueuedCommandFaild:
        return "排队命令失败";  // 注意：Faild 建议修正为 Failed（保持与枚举一致）
    case Error_InterfaceIncompatible:
        return "接口不兼容";
    default:
        return "未知错误";  // 修复：return 和字符串在同一行
    }  
}

//#include <fstream>
//#include <sstream>
//#include <string>
//#include <stdexcept>


void printTimestampEx(uint64_t timestamp, const char* type, int32_t count) {
    // 1. 避免重复构造时间点（原版本重复创建tp和tt）
    const auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>(
        std::chrono::microseconds(timestamp)
    );
    const auto tt = std::chrono::system_clock::to_time_t(tp);

    // 2. 局部时间转换使用线程安全的localtime_r（替代非线程安全的localtime）
    std::tm datetime{};  // 初始化避免栈内存垃圾值
#ifdef _WIN32
    // Windows平台使用localtime_s（C11标准，线程安全）
    localtime_s(&datetime, &tt);
#else
    // Linux/macOS平台使用localtime_r（POSIX标准，线程安全）
    localtime_r(&tt, &datetime);
#endif

    // 3. 提取微秒部分（避免重复计算timestamp % 1e6）
    const int microseconds = static_cast<int>(timestamp % 1000000);

    // 4. 合并打印语句减少IO操作
    PrintInfo(
        "%04d-%02d-%02d %02d:%02d:%02d.%06d\ttype: %s\tcount: %d\n",
        datetime.tm_year + 1900,
        datetime.tm_mon + 1,
        datetime.tm_mday,
        datetime.tm_hour,
        datetime.tm_min,
        datetime.tm_sec,
        microseconds,
        type ? type : "unknown",  
        count
    );
}

// 从外部文件读取retarget配置
std::string loadRetargetFromFile(const std::string& filePath) {
    // 检查文件路径是否为空
    if (filePath.empty()) {
        PrintInfo("retarget配置文件路径为空\n");
        return "";
    }

    // 使用二进制模式打开，避免Windows下自动转换换行符导致的问题
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        PrintInfo("无法打开retarget配置文件: %s（可能路径错误或无权限）\n", filePath.c_str());
        return "";
    }
    try {
        // 获取文件大小
        file.seekg(0, std::ios::end);
        std::streamsize fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        // 检查文件是否为空
        if (fileSize <= 0) {
            PrintInfo("retarget配置文件为空: %s\n", filePath.c_str());
            return "";
        }
        // 预分配缓冲区，避免多次扩容
        std::string buffer;
        buffer.reserve(static_cast<size_t>(fileSize) + 1); // +1 预留null终止符位置
        // 读取文件内容
        buffer.assign(std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>());

        // 确保字符串以null结尾（兼容C风格接口）
        buffer.push_back('\0');
        buffer.pop_back(); // 移除多余的null字符（避免长度错误）
        return buffer;
    }
    catch (const std::exception& e) {
        PrintInfo("读取retarget配置文件失败: %s（错误：%s）\n", filePath.c_str(), e.what());
        return "";
    }
}

// 骨骼节点定义
const char* tagnames[] = {"JointTag_Hips",
                          "JointTag_RightUpLeg",
                          "JointTag_RightLeg",
                          "JointTag_RightFoot",
                          "JointTag_LeftUpLeg",
                          "JointTag_LeftLeg",
                          "JointTag_LeftFoot",
                          "JointTag_Spine",
                          "JointTag_Spine1",
                          "JointTag_Spine2",
                          "JointTag_Neck",
                          "JointTag_Neck1",
                          "JointTag_Head",
                          "JointTag_RightShoulder",
                          "JointTag_RightArm",
                          "JointTag_RightForeArm",
                          "JointTag_RightHand",
                          "JointTag_RightHandThumb1",
                          "JointTag_RightHandThumb2",
                          "JointTag_RightHandThumb3",
                          "JointTag_RightInHandIndex",
                          "JointTag_RightHandIndex1",
                          "JointTag_RightHandIndex2",
                          "JointTag_RightHandIndex3",
                          "JointTag_RightInHandMiddle",
                          "JointTag_RightHandMiddle1",
                          "JointTag_RightHandMiddle2",
                          "JointTag_RightHandMiddle3",
                          "JointTag_RightInHandRing",
                          "JointTag_RightHandRing1",
                          "JointTag_RightHandRing2",
                          "JointTag_RightHandRing3",
                          "JointTag_RightInHandPinky",
                          "JointTag_RightHandPinky1",
                          "JointTag_RightHandPinky2",
                          "JointTag_RightHandPinky3",
                          "JointTag_LeftShoulder",
                          "JointTag_LeftArm",
                          "JointTag_LeftForeArm",
                          "JointTag_LeftHand",
                          "JointTag_LeftHandThumb1",
                          "JointTag_LeftHandThumb2",
                          "JointTag_LeftHandThumb3",
                          "JointTag_LeftInHandIndex",
                          "JointTag_LeftHandIndex1",
                          "JointTag_LeftHandIndex2",
                          "JointTag_LeftHandIndex3",
                          "JointTag_LeftInHandMiddle",
                          "JointTag_LeftHandMiddle1",
                          "JointTag_LeftHandMiddle2",
                          "JointTag_LeftHandMiddle3",
                          "JointTag_LeftInHandRing",
                          "JointTag_LeftHandRing1",
                          "JointTag_LeftHandRing2",
                          "JointTag_LeftHandRing3",
                          "JointTag_LeftInHandPinky",
                          "JointTag_LeftHandPinky1",
                          "JointTag_LeftHandPinky2",
                          "JointTag_LeftHandPinky3",
                          "JointTag_JointsCount"};

// 全局定义App接口及句柄
IMCPApplication* pGlobalApp = nullptr;
MCPApplicationHandle_t globalAppHandle = 0;
// 全局定义设置接口及句柄
IMCPSettings* pGlobalSetting = nullptr;
MCPSettingsHandle_t globalSettingHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Alice 数据分发
IMCPAliceHub* pAliceDataHub = nullptr;
MCPAliceBusHandle_t aliceBusHandle = 0;
// #endif
// 人物接收接口及句柄
IMCPAvatar* pAvatarInterface = nullptr;
MCPAvatarHandle_t avatarHandle = 0;
// Device接收接口及句柄
IMCPTracker* pDeviceInterface = nullptr;
MCPTrackerHandle_t deviceHandle = 0;
// IMU接收接口及句柄
IMCPSensorModule* pImuInterface = nullptr;
MCPSensorModuleHandle_t imuHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Marker接收接口及句柄
IMCPMarker* pMarkerInterface = nullptr;
MCPMarkerHandle_t markerHandle = 0;
// #endif
// 刚体接收接口及句柄
IMCPRigidBody* pRigidbodyInterface = nullptr;
MCPRigidBodyHandle_t rigidbodyHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Tracker/PWR接收接口及句柄
IMCPPWR* pPWRInterface = nullptr;
MCPPWRHandle_t pwrHandle = 0;
// #endif
// 渲染相关的接口及句柄
IMCPRenderSettings* pRenderSetting = nullptr;
MCPRenderSettingsHandle_t renderSettingHandle = 0;

// 最多事件数量
static constexpr uint32_t nMaxEventCount = 16;
// 接收事件
MCPEvent_t pEvents[nMaxEventCount] = {};
// 当前批次事件数量
uint32_t nEventCount = 0;
// 人物关节句柄
MCPJointHandle_t joint = 0;
// 全身关节句柄
MCPJointHandle_t alljoint[59] = {};

// 机器人接口
// IMCPRobot* robotInterface = nullptr;

int setMCPSettings(IMCPSettings* setting, MCPSettingsHandle_t handle) {


  if (setting == nullptr || handle == 0) {
      PrintInfo("无效的设置接口或句柄\n");
      return 1;
  }

  error = setting->SetSettingsBvhData(BvhDataType_Binary, handle);
  if (error != Error_None) {
      PrintInfo("设置BVH数据类型失败: %s\n", getErrorMsg(error));
  }

  error = setting->SetSettingsBvhRotation(BvhRotation_XYZ, handle);
  if (error != Error_None) {
      PrintInfo("设置BVH旋转类型失败: %s\n", getErrorMsg(error));
  }

  error = setting->SetSettingsBvhTransformation(BvhTransformation_Disable, handle);
  if (error != Error_None) {
      PrintInfo("设置BVH变换类型失败: %s\n", getErrorMsg(error));
  }

  error = setting->SetSettingsUDP(7012, handle);
  if (error != Error_None) {
      PrintInfo("设置UDP端口失败: %s\n", getErrorMsg(error));
  }

  // error = setting->SetSettingsTCP(strServerIP.c_str(), strServerPort, handle);
  // error = setting->SetSettingsUDPServer(strServerIP.c_str(), strServerPort, handle);

  return 0;
}

void setup() {
    // 创建并初始化全局App接口
    error = MCPGetGenericInterface(IMCPApplication_Version, reinterpret_cast<void**>(&pGlobalApp));
    if (error != Error_None) {
        PrintInfo("获取IMCPApplication接口失败: %s\n", getErrorMsg(error));
    }

    error = pGlobalApp->CreateApplication(&globalAppHandle);
    if (error != Error_None) {
        PrintInfo("创建应用失败: %s\n", getErrorMsg(error));
    }

    // 创建并初始化全局设置接口
    error = MocapApi::MCPGetGenericInterface(IMCPSettings_Version, reinterpret_cast<void**>(&pGlobalSetting));
    if (error != Error_None) {
        PrintInfo("获取IMCPSettings接口失败: %s\n", getErrorMsg(error));
    }

    error = pGlobalSetting->CreateSettings(&globalSettingHandle);
    if (error != Error_None) {
        PrintInfo("创建设置失败: %s\n", getErrorMsg(error));
    }

    int result = setMCPSettings(pGlobalSetting, globalSettingHandle);
    if (result != 0) {
        PrintInfo("设置MCP参数失败\n");
    }

    // 激活设置并销毁不再需要的设置
    error = pGlobalApp->SetApplicationSettings(globalSettingHandle, globalAppHandle);
    if (error != Error_None) {
        PrintInfo("应用设置失败: %s\n", getErrorMsg(error));
    }

    error = pGlobalSetting->DestroySettings(globalSettingHandle);
    if (error != Error_None) {
        PrintInfo("销毁设置失败: %s\n", getErrorMsg(error));
    }

    // 添加各种数据接收接口的关联
    error = MCPGetGenericInterface(IMCPAvatar_Version, reinterpret_cast<void**>(&pAvatarInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPAvatar接口失败: %s\n", getErrorMsg(error));
    }

    error = MCPGetGenericInterface(IMCPSensorModule_Version, reinterpret_cast<void**>(&pImuInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPSensorModule接口失败: %s\n", getErrorMsg(error));
    }

    error = MCPGetGenericInterface(IMCPMarker_Version, reinterpret_cast<void**>(&pMarkerInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPMarker接口失败: %s\n", getErrorMsg(error));
    }

    error = MCPGetGenericInterface(IMCPRigidBody_Version, reinterpret_cast<void**>(&pRigidbodyInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPRigidBody接口失败: %s\n", getErrorMsg(error));
    }

    // #ifdef _SUPORT_ALICE_NEW_
    error = MCPGetGenericInterface(IMCPPWR_Version, reinterpret_cast<void**>(&pPWRInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPPWR接口失败: %s\n", getErrorMsg(error));
    }

    error = MCPGetGenericInterface(IMCPAliceHub_Version, reinterpret_cast<void**>(&pAliceDataHub));
    if (error != Error_None) {
        PrintInfo("获取IMCPAliceHub接口失败: %s\n", getErrorMsg(error));
    }
    // #endif
    error = MCPGetGenericInterface(IMCPTracker_Version, reinterpret_cast<void**>(&pDeviceInterface));
    if (error != Error_None) {
        PrintInfo("获取IMCPTracker接口失败: %s\n", getErrorMsg(error));
    }

    for (uint32_t i = 0; i < nMaxEventCount; i++) {
        pEvents[i].size = sizeof(MCPEvent_t);
    }
    // 创建并初始化渲染相关的接口
    error = MCPGetGenericInterface(IMCPRenderSettings_Version, reinterpret_cast<void**>(&pRenderSetting));
    if (error != Error_None) {
        PrintInfo("获取IMCPRenderSettings接口失败: %s\n", getErrorMsg(error));
    }

    error = pRenderSetting->GetPreDefRenderSettings(PreDefinedRenderSettings_Default, &renderSettingHandle);
    if (error != Error_None) {
        PrintInfo("获取默认渲染设置失败: %s\n", getErrorMsg(error));
    }

    error = pGlobalApp->SetApplicationRenderSettings(renderSettingHandle, globalAppHandle);
    if (error != Error_None) {
        PrintInfo("应用渲染设置失败: %s\n", getErrorMsg(error));
    }

    // 启动应用
    error = pGlobalApp->OpenApplication(globalAppHandle);
    if (error != Error_None) {
        PrintInfo("启动应用失败: %s\n", getErrorMsg(error));
    }
}

void destroy() {
  error = pGlobalApp->CloseApplication(globalAppHandle);
  error = pGlobalApp->DestroyApplication(globalAppHandle);

 
}

//生成一个函数，计算两个数的最大公约数



void updateJoints(MCPJointHandle_t joint, MCPAvatarHandle_t avatar) {
  MocapApi::IMCPJoint* jointMgr = nullptr;
  MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version,
                                   reinterpret_cast<void**>(&jointMgr));

  const char* name = nullptr;
  error = jointMgr->GetJointName(&name, joint);
  MCPJointHandle_t tmpjoint = 0;
  pAvatarInterface->GetAvatarJointByName(name, &tmpjoint, avatar);
  float p[3];
  float r[4];
  MCPJointHandle_t childjoints[5];
  EMCPJointTag jointtags[5];
  uint32_t numberOfChildren = 0;
  EMCPJointTag jointT;
  if (joint) {
    error = jointMgr->GetJointLocalRotation(&r[0], &r[1], &r[2], &r[3], joint);
    error = jointMgr->GetJointLocalPosition(&p[0], &p[1], &p[2], joint);
    if (error != Error_None)  // 这里用来规避勾选“不带位移”后仍调用该接口所产生的野值
    {
      p[0] = p[1] = p[2] = 0;
    }
    error = jointMgr->GetJointTag(&jointT, joint);

   /* PrintInfo(
        "JointName:%s  JointTag<%d>:%s \n    pos:(%f, %f, %f)   quat(%f, %f, "
        "%f, %f)\n",
        name, jointT, tagnames[jointT], p[0], p[1], p[2], r[3], r[0], r[1],
        r[2]);*/

    error = jointMgr->GetJointLocalRotationByEuler(&p[0], &p[1], &p[2], joint);

   // PrintInfo("Euler Angles: (%.3f, %.3f, %.3f)\n\n", p.x, p.y, p.z);

    //子关节
    error = jointMgr->GetJointChild(nullptr, &numberOfChildren, joint);


    if (numberOfChildren > 0) {
        error = jointMgr->GetJointChild(childjoints, &numberOfChildren, joint);
        for (uint32_t j = 0; j < numberOfChildren; j++) {
            updateJoints(childjoints[j], avatar);
        }
    }
  }
  return;
}

int main(int argc, char** argv) {
  if (argc > 1) strServerIP = argv[1];
  if (argc > 2) strServerPort = atoi(argv[2]);
  if (argc > 3) {
    bPrint2Buffer = true;
    nBufferLen = (std::max)(1, atoi(argv[3])) * 1024;
    logBuffer = new char[nBufferLen];
  }

  printf("***MocapAPI***\n");
  printf("Server ip: %s    Server port: %d\n", strServerIP.c_str(),strServerPort);
  setup();

  // 免得每次都要构造和析构，提前定义一点变量
  uint32_t size;
  int32_t indexes[1024];
  float pos[3], acce[3], gyro[3], quat[4], axisAngle[4];
  int32_t itemsCount;
  uint64_t timestamp;
  uint32_t count;
  uint32_t id;
  int id2, status;

  uint32_t pwrId;

  time_t now = time(0);      // 当前系统时间精确到秒
  tm* gm = localtime(&now);  // 转为gmt时间
  time_t utc = mktime(gm);   // gmt对应的时间戳

  int64_t time = utc - now;  // 秒级时区转化
  uint64_t baseTime = time * 1e3;

// 根据版本获取机器人接口
  // error = MCPGetGenericInterface(IMCPRobot_Version, (void**)&robotInterface);
  // if (error != Error_None) {
  //     PrintInfo("获取IMCPRobot接口失败: %s\n", getErrorMsg(error));
  // }

  //通过外部文件加载retarget；
  std::string retargetContent = loadRetargetFromFile("../retarget.json"); 
  if (retargetContent.empty()) {
      PrintInfo("retarget配置文件读取失败，无法创建机器人接口\n");
  }

  MCPRobotHandle_t pHandle = 0;
  // 创建一个机器人
  error = RA_CreateRobot(retargetContent.c_str(), &pHandle);
  if (error != Error_None) {
      PrintInfo("创建机器人失败: %s\n", getErrorMsg(error));  
  }

  int fps = 100;
  // 输入数据帧数
  error = RA_SetRobotFPS(fps, pHandle);
  if (error != Error_None) {
      PrintInfo("设置机器人FPS失败: %s\n", getErrorMsg(error));
  }

  bool isListening = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (isListening) 
  {

    nEventCount = 0;
    error = pGlobalApp->PollApplicationNextEvent(nullptr, &nEventCount, globalAppHandle);

    if (error != Error_None && error != Error_NoneMessage) {
        PrintInfo("轮询事件失败: %s\n", getErrorMsg(error));
    }
    if (nEventCount) 
    {
      error = pGlobalApp->PollApplicationNextEvent(pEvents, &nEventCount,globalAppHandle);
      if (error != Error_None) {
          PrintInfo("获取事件数据失败: %s\n", getErrorMsg(error));
      }
      for (uint32_t i = 0; i < nEventCount; i++) {
        if (pEvents[i].eventType == MCPEvent_AvatarUpdated) {

          avatarHandle = pEvents[i].eventData.motionData.avatarHandle;
          const char* tmpName = nullptr;
          error = pAvatarInterface->GetAvatarRootJoint(&joint, avatarHandle);
          error = pAvatarInterface->GetAvatarName(&tmpName, avatarHandle);

          PrintInfo("AvatarName: %s\n", tmpName);
          uint32_t jointSize = 0;
          error = pAvatarInterface->GetAvatarJoints(nullptr, &jointSize,
                                                    avatarHandle);
          error = pAvatarInterface->GetAvatarJoints(alljoint, &jointSize,
                                                    avatarHandle);
          MCPJointHandle_t jointHandle = 0;
          error = pAvatarInterface->GetAvatarJointByName("Hips", &jointHandle,
                                                         avatarHandle);
          if (error == Error_None && jointSize > 0) {
            updateJoints(jointHandle, avatarHandle);
          }

          // 每帧获取机器人的关节数据
          error = RA_UpdateRobot(avatarHandle, pHandle);
          if (error != Error_None) {
              PrintInfo("更新机器人数据失败: %s\n", getErrorMsg(error));
          }
          float p[3] = {0};
          float r[4] = {0};
          // root节点数据
          error = RA_GetRobotRootPosition(&p[0], &p[1], &p[2], pHandle);
          if (error != Error_None) {
              PrintInfo("获取机器人根节点位置失败: %s\n", getErrorMsg(error));
          }

          error = RA_GetRobotRootRotation(&r[0], &r[1], &r[2], &r[3], pHandle);
          if (error != Error_None) {
              PrintInfo("获取机器人根节点旋转失败: %s\n", getErrorMsg(error));
          }
          else
          {
              PrintInfo("RobotRoot: pos:(%f, %f, %f)   quat(%f, %f, %f, %f)\n",
                  p[0], p[1], p[2], r[3], r[0], r[1], r[2]);
          }

          //jointName为此retarget的具体关节节点，可根据需求输入，调用后返回一个角度。
          char* jointName1 = "R_thumb_MCP_joint1";
          float value1 = 0;
          // 获取机器人目标关节角度并检查错误
          EMCPError angleError = RA_GetRobotRetargetJointAngle(jointName1, &value1, pHandle);
          if (angleError != Error_None) {
              PrintInfo("获取关节[%s]角度失败: %s\n", jointName1, getErrorMsg(angleError));
          }
          else {
              PrintInfo("RobotjointName:%s :%f  \n", jointName1, value1 * 57.3f);
          }

          // 获取jsonStr
          bool compress = true;
          const char* result;
          EMCPError robotStatus = RA_GetRobotRosFrameJson(&result, compress, pHandle);
          if (robotStatus == Error_None) {
             // PrintInfo("jsonStr: %s  \n", result);
          }
          else {
              PrintInfo("获取机器人ROS帧JSON失败: %s\n", getErrorMsg(robotStatus));
          }

          error = RA_RunRobotStep(pHandle);
          if (error != Error_None) {
              PrintInfo("运行机器人步骤失败: %s\n", getErrorMsg(error));
          }

        }
        else if (pEvents[i].eventType == MCPEvent_AliceMarkerUpdated) 
        {
          pAliceDataHub->GetMarkerList(
              nullptr, &count);  // 第一次传入参1为nullptr，意为获取对象的数量
          if (count > 0) {
            MCPMarkerHandle_t* recv =
                new MCPMarkerHandle_t[count];  // 动态开辟接收内存
            pAliceDataHub->GetMarkerList(
                recv,
                &count);  // 第二次传入参1不为nullptr，将接收实际的handle列表
            pAliceDataHub->GetMarkerTimestamp(
                &timestamp);  // 以包为单位的时间戳

            printTimestampEx(timestamp, "Marker", count);
            for (size_t i = 0; i < count; ++i) {
              pMarkerInterface->GetMarkerPosition(&pos[0], &pos[1], &pos[2],
                                                  recv[i]);
              PrintInfo("marker:(%f, %f, %f)\n", pos[0], pos[1], pos[2]);
            }

            delete[] recv;
            recv = nullptr;
          }
        } 
        else if (pEvents[i].eventType == MCPEvent_AliceIMUUpdated) 
        {
          //  FrameTime();

          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetSensorModuleList(nullptr, &count);
          if (count > 0) {
            MCPSensorModuleHandle_t* recv = new MCPSensorModuleHandle_t[count];
            pAliceDataHub->GetSensorModuleList(recv, &count);
            pAliceDataHub->GetSensorModuleTimestamp(&timestamp);

            printTimestampEx(timestamp, "IMU", count);

            for (size_t i = 0; i < count; ++i) {
              pImuInterface->GetSensorModuleId(&id, recv[i]);
              pImuInterface->GetSensorModuleAcceleratedVelocity(
                  &acce[0], &acce[1], &acce[2], recv[i]);
              pImuInterface->GetSensorModuleAngularVelocity(&gyro[0], &gyro[1],
                                                            &gyro[2], recv[i]);
              pImuInterface->GetSensorModulePosture(
                  &quat[0], &quat[1], &quat[2], &quat[3], recv[i]);

              PrintInfo(
                  "id:(%d), accelerometer:(%f, %f, %f), gyroscope:(%f, %f, "
                  "%f), quat:(%f, %f, %f, %f)\n",
                  id, acce[0], acce[1], acce[2], gyro[0], gyro[1], gyro[2],
                  quat[0], quat[1], quat[2], quat[3]);
            }
            delete[] recv;
            recv = nullptr;
          }
        } 
        else if (pEvents[i].eventType == MCPEvent_AliceRigidbodyUpdated) 
        {
           // FrameTime();
          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetRigidBodyList(nullptr, &count);
          if (count > 0) {
            MCPRigidBodyHandle_t* recv = new MCPRigidBodyHandle_t[count];
            pAliceDataHub->GetRigidBodyList(recv, &count);
            pAliceDataHub->GetRigidBodyTimestamp(&timestamp);

            printTimestampEx(timestamp, "Rigid body", count);
            for (size_t i = 0; i < count; ++i) {
              pRigidbodyInterface->GetRigidBodyId(&id2, recv[i]);
              pRigidbodyInterface->GetRigidBodyPosition(&pos[0], &pos[1],
                                                        &pos[2], recv[i]);
              pRigidbodyInterface->GetRigidBodyRotation(
                  &quat[0], &quat[1], &quat[2], &quat[3], recv[i]);
              pRigidbodyInterface->GetRigidBodyAxisAngle(
                  &axisAngle[0], &axisAngle[1], &axisAngle[2], &axisAngle[3],
                  recv[i]);
              PrintInfo(
                  "id:(%d), pos:(%f, %f, %f), quat:(%f, %f, %f, "
                  "%f(w))\naxisAngle:((%f, %f, %f), %f)\n",
                  id2, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
                  quat[3], axisAngle[0], axisAngle[1], axisAngle[2],
                  axisAngle[3]);
            }
            delete[] recv;
            recv = nullptr;
          }
        } 
        else if (pEvents[i].eventType == MCPEvent_AliceTrackerUpdated) 
        {
          //  FrameTime();
          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetPWRList(nullptr, &count);
          if (count > 0) {
            MCPPWRHandle_t* recv = new MCPPWRHandle_t[count];
            pAliceDataHub->GetPWRList(recv, &count);
            pAliceDataHub->GetPWRTimestamp(&timestamp);

            
            PrintInfo("TrackerData\n");
            for (size_t i = 0; i < count; ++i) {

              pPWRInterface->GetPWRId(&pwrId,recv[i]);
              pPWRInterface->GetPWRStatus(&status, recv[i]);
              pPWRInterface->GetPWRPosition(&pos[0], &pos[1], &pos[2], recv[i]);
              pPWRInterface->GetPWRQuaternion(&quat[0], &quat[1], &quat[2],
                                              &quat[3], recv[i]);
              PrintInfo(
                  "id:(%d), status(%d), pos:(%f, %f, %f), quat:(%f, %f, %f, "
                  "%f)\n",
                  pwrId, status, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
                  quat[3]);
            }
            delete[] recv;
            recv = nullptr;
          }
        }
        else if (pEvents[i].eventType == MCPEvent_TrackerUpdated) 
        {
          //  FrameTime();

          deviceHandle = pEvents[i].eventData.trackerData._trackerHandle;
          int count = 0;
          pDeviceInterface->GetDeviceCount(&count, deviceHandle);

          auto tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now());
          PrintInfo("[Device Timestamp]%lld\n",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp.time_since_epoch())
                            .count() +
                        baseTime);
          for (int i = 0; i < count; ++i) {
            const char* name;  // 这里无需开辟内存
            pDeviceInterface->GetDeviceName(i, &name, deviceHandle);
            pDeviceInterface->GetTrackerPosition(&pos[0], &pos[1], &pos[2],
                                                 name, deviceHandle);
            pDeviceInterface->GetTrackerRotation(&quat[0], &quat[1], &quat[2],
                                                 &quat[3], name, deviceHandle);
            PrintInfo("DeviceData");
            PrintInfo(
                "[Device]name:(%s), pos:(%f, %f, %f), quat:(%f, %f, %f, %f)\n",
                name, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
                quat[3]);
          }
        }
      }

    }
  }

  RA_DestroyRobot(pHandle);
  destroy();
  return 0;
}
