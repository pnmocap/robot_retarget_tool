#include "mcp_robot.h"
#include "librobot/include/robot.h"
#include <algorithm>
#include <cassert>

namespace MocapApi {
CMCPRobot::Manager::Manager()
    : IMCPRobot(),
      base::handle_manager<CMCPRobot, MCPRobotHandle_t>(),
      _listRobot() {}
CMCPRobot::Manager::~Manager() {}

CMCPRobot::Manager* CMCPRobot::Manager::get() {
  static Manager instance;
  return &instance;
}

EMCPError CMCPRobot::Manager::CreateRobot(const char* jsonConfigStr,
                                          MCPRobotHandle_t* pHandle) {
  auto robot = std::make_shared<CMCPRobot>();
  if (robot->load(jsonConfigStr)) {
    _listRobot.push_back(robot);
    *pHandle = insert(robot);
    return Error_None;
  } else {
    return Error_InvalidParameter;
  }
}
EMCPError CMCPRobot::Manager::SetRobotFPS(int fps, MCPRobotHandle_t handle) {
  return run(handle, [fps](auto obj) {
    obj->setFPS(fps);
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::UpdateRobot(MCPAvatarHandle_t avatar,
                                          MCPRobotHandle_t handle) {
  return run(handle, [avatar](auto obj) { return obj->update(avatar); });
}
EMCPError CMCPRobot::Manager::GetRobotRawJointAngle(
    EMCPRobotJointType jointType, float* value, MCPRobotHandle_t handle) {
  return run(handle, [jointType, value](auto obj) {
    return obj->getRawJointAngle(jointType, value);
  });
}
EMCPError CMCPRobot::Manager::GetRobotRetargetJointAngle(const char* jointName,
                                                         float* value,
                                                         MCPRobotHandle_t handle) {
  return run(handle, [jointName, value](auto obj) {
    return obj->getRetargetJointAngle(jointName, value);
  });
}
EMCPError CMCPRobot::Manager::GetRobotRosFrameJson(const char** jsonStr,
                                                   bool compress,
                                                   MCPRobotHandle_t handle) {
  return run(handle, [jsonStr, compress](auto obj) {
    obj->getRosFrameJson(jsonStr, compress);
    return Error_None;
  });
}

EMCPError CMCPRobot::Manager::GetRobotRootRotation(float* x, float* y,
                                                   float* z, float* w,
                                                   MCPRobotHandle_t handle) {
  return run(handle, [&, x, y, z, w](auto obj) {
    obj->getRootRotation(x, y, z, w);
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::GetRobotRootPosition(float* x, float* y,
                                                   float* z,
                                                   MCPRobotHandle_t handle) {
  return run(handle, [&, x, y, z](auto obj) {
    obj->getRootPosition(x, y, z);
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::DestroyRobot(MCPRobotHandle_t handle) {
  auto result = run(handle, [this](auto obj) {
    auto iter = std::find(_listRobot.begin(), _listRobot.end(), obj);
    assert(iter != _listRobot.end());
    _listRobot.erase(iter);
    return Error_None;
  });
  if (result == Error_None) {
    remove(handle);
  }
  return result;
}

EMCPError CMCPRobot::Manager::RunRobotStep(MCPRobotHandle_t handle) {
  return run(handle, [&](auto obj) {
    obj->runStep();
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::RunRobotStep1(float fixedDelta,
                                            MCPRobotHandle_t handle) {
  return run(handle, [&](auto obj) {
    obj->runStep(fixedDelta);
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::GetRobotSlideSpeed(float* value,
                                                 MCPRobotHandle_t handle) {
  return run(handle, [&](auto obj) {
    obj->GetSlideSpeed(value);
    return Error_None;
  });
}
EMCPError CMCPRobot::Manager::GetRobotSlideHeight(float* value,
                                                  MCPRobotHandle_t handle) {
  return run(handle, [&](auto obj) {
    obj->GetSlideHeight(value);
    return Error_None;
  });
}

CMCPRobot::CMCPRobot() : _robot(std::make_unique<eba::Robot>()) {}
CMCPRobot::~CMCPRobot() {}

bool CMCPRobot::load(const char* jsonConfigStr) {
  assert(nullptr != _robot);
  return _robot->Load(jsonConfigStr);
}
void CMCPRobot::unload() {
  assert(nullptr != _robot);
  _robot->Unload();
}
void CMCPRobot::setFPS(int fps) {
  assert(nullptr != _robot);
  _robot->SetFPS(fps);
}
EMCPError CMCPRobot::update(MCPAvatarHandle_t avatar) {
  assert(nullptr != _robot);
  MocapApi::IMCPAvatar* avatarInterface = nullptr;
  MocapApi::IMCPJoint* jointInterface = nullptr;

  auto err = MocapApi::MCPGetGenericInterface(
      MocapApi::IMCPAvatar_Version,
      reinterpret_cast<void**>(&avatarInterface));
  if (err != MocapApi::Error_None) {
    return err;
  }

  err = MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version,
                                         reinterpret_cast<void**>(&jointInterface));
  if (err != MocapApi::Error_None) {
    return err;
  }

  return _robot->Update(avatarInterface, avatar, jointInterface);
}
EMCPError CMCPRobot::getRawJointAngle(EMCPRobotJointType jointType,
                                      float* value) {
  assert(nullptr != _robot);
  return _robot->GetRawJointAngle(static_cast<eba::RobotJointType>(jointType),
                                  value);
}
EMCPError CMCPRobot::getRetargetJointAngle(const char* jointName,
                                           float* value) {
  assert(nullptr != _robot);
  return _robot->GetRetargetJointAngle(jointName, value);
}
void CMCPRobot::getRosFrameJson(const char** str_, bool compress) {
  assert(nullptr != _robot);
  _robot->GetRosFrameJson(_rosFrameJson, compress);
  *str_ = _rosFrameJson.data();
}
void CMCPRobot::getRootRotation(float* x, float* y, float* z, float* w) {
  assert(nullptr != _robot);
  _robot->GetRootRotation(x, y, z, w);
}
void CMCPRobot::getRootPosition(float* x, float* y, float* z) {
  assert(nullptr != _robot);
  _robot->GetRootPosition(x, y, z);
}
void CMCPRobot::runStep() {
  assert(nullptr != _robot);
  _robot->Step();
}
void CMCPRobot::runStep(float fixedDelta) {
  assert(nullptr != _robot);
  _robot->Step(fixedDelta);
}
EMCPError CMCPRobot::GetSlideSpeed(float* value) {
  assert(nullptr != _robot);
  return _robot->GetSlideSpeed(value);
}
EMCPError CMCPRobot::GetSlideHeight(float* value) {
  assert(nullptr != _robot);
  return _robot->GetSlideHeight(value);
}

}  // namespace MocapApi