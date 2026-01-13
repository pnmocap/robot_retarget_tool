#pragma once
#include <list>
#include <memory>
#include <string>

#include "third_party/MocapApi/include/MocapApi.h"
#include "handle_manager.hpp"

namespace eba {
class Robot;
}
namespace MocapApi {
class CMCPRobot {
 public:
  class Manager : public IMCPRobot,
                  public base::handle_manager<CMCPRobot, MCPRobotHandle_t> {
   public:
    static Manager* get();

    EMCPError CreateRobot(const char* jsonConfigStr,
                          MCPRobotHandle_t* pHandle) override final;
    EMCPError SetRobotFPS(int fps, MCPRobotHandle_t handle);
    EMCPError UpdateRobot(MocapApi::MCPAvatarHandle_t avatarHandle,
                          MCPRobotHandle_t handle);
    EMCPError GetRobotRawJointAngle(EMCPRobotJointType jointType, float* value,
                                    MCPRobotHandle_t handle);
    EMCPError GetRobotRetargetJointAngle(const char* jointName, float* value,
                                         MCPRobotHandle_t handle);
    EMCPError GetRobotRosFrameJson(const char** jsonStr, bool compress,
                                   MCPRobotHandle_t handle);
    EMCPError GetRobotRootRotation(float* x, float* y, float* z, float* w,
                                   MCPRobotHandle_t handle);
    EMCPError GetRobotRootPosition(float* x, float* y, float* z,
                                   MCPRobotHandle_t handle);
    EMCPError DestroyRobot(MCPRobotHandle_t handle) override final;
    EMCPError RunRobotStep(MCPRobotHandle_t handle) override final;
    EMCPError RunRobotStep1(float fixedDelta,
                            MCPRobotHandle_t handle) override final;
    EMCPError GetRobotSlideSpeed(float* value,
                                 MCPRobotHandle_t handle) override final;
    EMCPError GetRobotSlideHeight(float* value,
                                  MCPRobotHandle_t handle) override final;

   private:
    std::list<std::shared_ptr<CMCPRobot>> _listRobot;

    Manager();

    ~Manager();

    Manager(const Manager& other) = delete;

    Manager& operator=(const Manager& other) = delete;

    Manager(Manager&& other) = delete;

    Manager& operator=(Manager&& other) = delete;
  };

  CMCPRobot();
  ~CMCPRobot();
  bool load(const char* jsonConfigStr);
  void unload();
  void setFPS(int fps);
  EMCPError update(MCPAvatarHandle_t avatarHandle);
  EMCPError getRawJointAngle(EMCPRobotJointType jointType, float* value);
  EMCPError getRetargetJointAngle(const char* jointName, float* value);
  void getRosFrameJson(const char**, bool compress);
  void getRootRotation(float* x, float* y, float* z, float* w);
  void getRootPosition(float* x, float* y, float* z);
  void runStep();
  void runStep(float fixedDelta);
  EMCPError GetSlideSpeed(float* value);
  EMCPError GetSlideHeight(float* value);

  CMCPRobot(const CMCPRobot& other) = delete;
  CMCPRobot& operator=(const CMCPRobot& other) = delete;
  CMCPRobot(CMCPRobot&& other) = delete;
  CMCPRobot& operator=(CMCPRobot&& other) = delete;

 private:
  std::unique_ptr<eba::Robot> _robot;
  std::string _rosFrameJson;
};
}  // namespace MocapApi