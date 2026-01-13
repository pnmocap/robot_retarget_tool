#pragma once

#include "MocapApi.h"

#if defined(_WIN32)
  // On Windows, only the DLL itself should export symbols.
  // Consumers should not see ROBOT_API_SHARED_BUILD; they link against the
  // import library produced by the DLL target.
  #if defined(ROBOT_API_SHARED_BUILD)
    #define ROBOT_API_EXPORT __declspec(dllexport)
  #else
    #define ROBOT_API_EXPORT
  #endif
#else
  #if defined(ROBOT_API_SHARED)
    #define ROBOT_API_EXPORT __attribute__((visibility("default")))
  #else
    #define ROBOT_API_EXPORT
  #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

ROBOT_API_EXPORT MocapApi::EMCPError RA_CreateRobot(const char* jsonConfigStr, MocapApi::MCPRobotHandle_t* handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_DestroyRobot(MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_SetRobotFPS(int fps, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_UpdateRobot(MocapApi::MCPAvatarHandle_t avatarHandle, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotRawJointAngle(MocapApi::EMCPRobotJointType jointType, float* value, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotRetargetJointAngle(const char* jointName, float* value, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotRosFrameJson(const char** jsonStr, bool compress, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotRootRotation(float* x, float* y, float* z, float* w, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotRootPosition(float* x, float* y, float* z, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_RunRobotStep(MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_RunRobotStep1(float fixedDelta, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotSlideSpeed(float* value, MocapApi::MCPRobotHandle_t handle);
ROBOT_API_EXPORT MocapApi::EMCPError RA_GetRobotSlideHeight(float* value, MocapApi::MCPRobotHandle_t handle);

#ifdef __cplusplus
}
#endif
