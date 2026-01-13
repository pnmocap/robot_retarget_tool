#include "robot_api.h"
#include "mcp_robot.h"

using namespace MocapApi;

extern "C" {

ROBOT_API_EXPORT EMCPError RA_CreateRobot(const char* jsonConfigStr, MCPRobotHandle_t* handle) {
  return CMCPRobot::Manager::get()->CreateRobot(jsonConfigStr, handle);
}

ROBOT_API_EXPORT EMCPError RA_DestroyRobot(MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->DestroyRobot(handle);
}

ROBOT_API_EXPORT EMCPError RA_SetRobotFPS(int fps, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->SetRobotFPS(fps, handle);
}

ROBOT_API_EXPORT EMCPError RA_UpdateRobot(MCPAvatarHandle_t avatarHandle, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->UpdateRobot(avatarHandle, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotRawJointAngle(EMCPRobotJointType jointType, float* value, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotRawJointAngle(jointType, value, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotRetargetJointAngle(const char* jointName, float* value, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotRetargetJointAngle(jointName, value, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotRosFrameJson(const char** jsonStr, bool compress, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotRosFrameJson(jsonStr, compress, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotRootRotation(float* x, float* y, float* z, float* w, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotRootRotation(x, y, z, w, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotRootPosition(float* x, float* y, float* z, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotRootPosition(x, y, z, handle);
}

ROBOT_API_EXPORT EMCPError RA_RunRobotStep(MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->RunRobotStep(handle);
}

ROBOT_API_EXPORT EMCPError RA_RunRobotStep1(float fixedDelta, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->RunRobotStep1(fixedDelta, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotSlideSpeed(float* value, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotSlideSpeed(value, handle);
}

ROBOT_API_EXPORT EMCPError RA_GetRobotSlideHeight(float* value, MCPRobotHandle_t handle) {
  return CMCPRobot::Manager::get()->GetRobotSlideHeight(value, handle);
}

} // extern "C"
