#ifndef ROBOT_H
#define ROBOT_H

#include "mathutils.h"
#include "jointnode.h"
#include"FscAlgorithmImpl.h"
#include"MotionAdaptorLib.h"
#include <chrono>
#include <list>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <queue>

#include "../../third_party/MocapApi/include/MocapApi.h"
#include <deque>
#include <mutex>

#define JOINT_COUNT 59

namespace eba 
{

    enum RobotJointType
    {
        JointLeftShoulder_Pitch,
        JointLeftShoulder_Roll,
        JointLeftShoulder_Yaw,
        JointLeftElbow,
        JointLeftWrist_Yaw,
        JointLeftWrist_Roll,
        JointLeftWrist_Pitch,

        JointRightShoulder_Pitch,
        JointRightShoulder_Roll,
        JointRightShoulder_Yaw,
        JointRightElbow,
        JointRightWrist_Yaw,
        JointRightWrist_Roll,
        JointRightWrist_Pitch,

        JointLeftHip_Yaw,
        JointLeftHip_Pitch,
        JointLeftHip_Roll,
        JointLeftKnee,
        JointLeftAnkle_Pitch,
        JointLeftAnkle_Roll,

        JointRightHip_Yaw,
        JointRightHip_Pitch,
        JointRightHip_Roll,
        JointRightKnee,
        JointRightAnkle_Pitch,
        JointRightAnkle_Roll,

        JointSpine_Yaw,
        JointSpine_Roll,
        JointSpine_Pitch,

        JointHead_Yaw,
        JointHead_Roll,
        JointHead_Pitch,

        JointLeftThumb1_1,
        JointLeftThumb1_2,
        JointLeftThumb2,
        JointLeftThumb3,
        JointLeftIndex1,
        JointLeftIndex2,
        JointLeftIndex3,
        JointLeftMiddle1,
        JointLeftMiddle2,
        JointLeftMiddle3,
        JointLeftRing1,
        JointLeftRing2,
        JointLeftRing3,
        JointLeftPinky1,
        JointLeftPinky2,
        JointLeftPinky3,

        JointRightThumb1_1,
        JointRightThumb1_2,
        JointRightThumb2,
        JointRightThumb3,
        JointRightIndex1,
        JointRightIndex2,
        JointRightIndex3,
        JointRightMiddle1,
        JointRightMiddle2,
        JointRightMiddle3,
        JointRightRing1,
        JointRightRing2,
        JointRightRing3,
        JointRightPinky1,
        JointRightPinky2,
        JointRightPinky3,

        JointCount,
    };

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(EulerOrder, axis1, axis2, axis3)

    struct RetargetJoint
    {
        int rawJoint;
        std::string retargetJoint;
        float offset = 0;
        int sign = 1;
        float min = -glm::pi<float>();
        float max = glm::pi<float>();
        float maxSpeed = glm::pi<float>() * 10; // 最大速度
        bool useSpeedLimit = true;
        bool useAngleLimit = true;
    };

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RetargetJoint, 
        rawJoint, 
        retargetJoint, 
        offset, 
        sign,
        min, 
        max,
        maxSpeed, 
        useSpeedLimit,
        useAngleLimit
    )

    struct RetargetingConfig
    {
        //20250401,新增机器人尺寸适配参数
        float hipsWith = 0;
        float hipsHeight = 0;
        float legLength = 0;
        float ankleHeight = 0;

        float shoulderAngle = 0.0f;        
        int lowerBodyMode = 0; // 0默认全身，1三折后弯,2两折前弯
        bool useSpeedLimit = true;
        bool useAngleLimit = true;
        float moveSpeed = 1.0f; // 最大行进速度m/s
        float rotateSpeed = 1.5f; // 最大转身速度rad/s
        float bendStartAngle = 20.0f; // 开始弯腰阈值
        float bendMaxAngle = 10.0f; // 最大弯腰幅度
        float bendScale = 0.3f; // 机器人弯腰和真人的缩放比
        float centroidHorizontalOffset = 0.2f; // 重心向后偏移值（和urdf初始姿态比）
        // 设定一个hip下蹲高度比例阈值（相对站立时）
        float hipMax = 0.9f;
        float hipMin = 0.25f;

        // 升降阈值（和人站立高度的比例，用于控制升降滑轨）
        float slideThresholdMax = 1.05f; 
        float slideThresholdMin = 0.86f;
        float slideHeightMin = 0.3f; // 最低升降高度
        float slideHeightMax = 1.7f; // 最高升降高度
        float slideHeightInit = 1.55f; // 初始升降高度
        float slideSpeed = 1.0f; // 升降速度(m/s)
        float handMode = 0;
        bool dataSmooth = false;

        EulerOrder shoulderOrder = { 1, 0, 2 };
        EulerOrder wristOrder = { 1, 0, 2 };
        EulerOrder hipOrder = { 2, 1, 0 };
        EulerOrder ankelOrder = { 1, 0, 2 };
        EulerOrder spineOrder = { 2, 0, 1 };
        EulerOrder headOrder = { 2, 0, 1 };

        std::vector<std::string> urdfJointNames; // 机器人关节名，按照urdf顺序

        std::vector<RetargetJoint> retargetJoints;
    };

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RetargetingConfig,

        hipsWith,
        hipsHeight,
        legLength,
        ankleHeight,
        shoulderAngle,
        lowerBodyMode,
        handMode,
        useSpeedLimit,
        useAngleLimit,
        shoulderOrder,
        wristOrder,
        hipOrder,
        ankelOrder,
        spineOrder,
        headOrder,
        retargetJoints,
        urdfJointNames,
        moveSpeed,
        rotateSpeed,
        bendStartAngle,
        bendMaxAngle,
        bendScale,
        hipMax,
        hipMin,
        slideThresholdMax,
        slideThresholdMin,
        slideHeightMin,
        slideHeightMax,
        slideHeightInit,
        slideSpeed,
        dataSmooth
    )

    struct MocapData
    {
        Quat localRot;
        Vector3 localPos;
        Quat worldRot;
        Vector3 worldPos;
    };

    class Robot
    {
    public:
        Robot();
        virtual ~Robot();

        // 加载配置json字符串（注意不是文件，而是内容）
        bool Load(const char* jsonConfigStr);
        void Unload();

        // 需要和上位机发送帧率一致
        void SetFPS(int fps);

        void Update(const MocapData* frameData);

        // 每帧更新，从mocapAPI取数并计算
        MocapApi::EMCPError Update(MocapApi::IMCPAvatar* mcpAvatar, MocapApi::MCPAvatarHandle_t avatarHandle, MocapApi::IMCPJoint* jointInterface);

        void Step(float fixedDelta = 0);

        // 获取标准关节角
        // jointType: 标准关节类型（不是动捕BVH，而是转换为标准关节定义所计算的中间结果，A-Pose为零位）
        // value: [in out]值（弧度）
        MocapApi::EMCPError GetRawJointAngle(RobotJointType jointType, float* value) const;

        // 获取映射后的关节角，和GetRawJointAngle比，这个接口是最终对应urdf的关节角，包含了名字映射和偏移量
        // jointName对应urdf中的关节名
        // value: [in out]值（弧度）
        MocapApi::EMCPError GetRetargetJointAngle(const std::string& jointName, float* value) const;

        // 获取映射后的关节角速度
        // jointName对应urdf中的关节名
        // value: [in out]值（弧度/秒）
        MocapApi::EMCPError GetRetargetJointAngleVelocity(const std::string& jointName, float* value) const;


        // 获取ros帧数据
        // compress: 输出为压缩格式还是人类可读格式json
        void GetRosFrameJson(std::string& jsonStr, bool compress) const;

        // 获取动捕根节点位姿（已转换为标准前-左-上坐标系）
        void GetRootRotation(float* x, float* y, float* z, float* w) const;
        void GetRootPosition(float* x, float* y, float* z) const;

        // 获取升降滑轨速度
        MocapApi::EMCPError GetSlideSpeed(float* value) const;

        // 获取升降滑轨高度
        MocapApi::EMCPError GetSlideHeight(float* value) const;

    private:
        friend class RobotEditor;
        friend class JointTree;

        void solveDeltaTime();
        void InfoDisp();
        void SetUpdateWithDisp();
        Vector3 RotateVectorManually(const eba::Quat& q, const eba::Vector3& v);

        void SetRawJointAngle(RobotJointType, float value);
        MocapApi::EMCPError GetRawJointAngleInternal(RobotJointType, float* value) const;
        RetargetingConfig& GetConfig() { return m_config; }
        float GetRawJointAngle(RobotJointType) const;
        const std::map<RobotJointType, float>& GetRawJointsAngle() const { return m_rawJointsAngle; }
        float GetRetargetJointAngle(const std::string& jointName) const;
        const std::map<std::string, float>& GetRetargetJointsAngle() const;

        void calcWheeledPosition();

        void preprocess();

        void solveHumanoid();

        void onFirstFrame();

        void solveVirtualBones();

        void postprocess();

    private:
        RetargetingConfig m_config;
        std::mutex m_queueLock;
        std::queue<MocapData> m_mocapInputCache[MocapApi::EMCPJointTag::JointTag_JointsCount];
        Quat m_mocapInputRot[MocapApi::EMCPJointTag::JointTag_JointsCount];
        Vector3 m_mocapInputPos[MocapApi::EMCPJointTag::JointTag_JointsCount];
        Quat m_mocapInputRotWorld[MocapApi::EMCPJointTag::JointTag_JointsCount];
        Vector3 m_mocapInputPosWorld[MocapApi::EMCPJointTag::JointTag_JointsCount];

        float targetHipsWidth;
        float targetHipsHeight;
        float targetAnkleHeight;
        float targetLegLength;
        
        enum DummyNodeType
        {
            DummyRoot,
            DummyLeg1,
            DummyLeg2,
            DummySpine,
            DummyHead,
            DummyNodeCount
        };
        std::shared_ptr<JointNode> m_dummyNodes[DummyNodeCount]; // 用于特殊处理根节点
        std::shared_ptr<JointNode> m_dummyKneeCtrl; // 虚拟膝盖控制点
        std::shared_ptr<JointNode> m_dummyHipCtrl; // 虚拟髋控制点
        float m_centroidHorizontalOffset;
        float m_initCentroidHorizontalOffset;

        std::map<RobotJointType, float> m_rawJointsAngle;
        std::map<std::string, float> m_retargetJointsAngle;
        std::map<std::string, float> m_retargetJointsAngleSmooth;
        std::map<std::string, std::deque<float>> m_smoothCacheAngleData;
        std::map<std::string, float> m_retargetJointsAngleLastFrame; // for speed
        std::map<std::string, float> m_retargetJointsAngleLastFrameSmooth; // for speed
        std::map<std::string, float> m_retargetJointsAngleSpeed;
        std::map<std::string, float> m_retargetJointsAngleSpeedSmooth;
        std::map<std::string, std::deque<float>> m_smoothCacheSpeedData;

        Quat m_rootRotation;
        Vector3 m_rootPosition;
        float m_shoulderAngle;
        std::chrono::time_point<std::chrono::steady_clock> m_startRecordTime;
        int m_fps = 90;

        float m_deltaTime = 0.0f;

        // 下半身计算#1
        float m_lowerLegLength = 0; // 大腿长度
        float m_upperLegLength = 0; // 小腿长度
        float m_squatThresholdMax = 0; // 下蹲上限
        float m_squatThresholdMin = 0; // 下蹲上限

        float m_lastRightShoulderYaw = 0;
        float m_lastLeftShoulderYaw = 0;
        float m_lastRightShoulderPitch = 0;
        float m_lastLeftShoulderPitch = 0;
        float m_lastRightShoulderRoll = 0;
        float m_lastLeftShoulderRoll = 0;

        std::chrono::time_point<std::chrono::steady_clock> m_lastTickTime;
        float locked_right_j1_pitch_value = 0.0f;
        float locked_left_j1_pitch_value = 0.0f;

        bool m_firstUpdate = true;
        bool m_enableScale = false;
        Vector3 m_TargetDir;
        Vector3 m_TargetPos;
        float m_hipsHeightStand = 0; // 正常站立跨高
        float m_hipsHeightHighest = 0;
        float m_hipsHeightLowest = 0;
        float m_robotLegLength1 = 0;
        float m_robotLegLength2 = 0;
        float m_robotLegLength = 0;
        float m_baseHeight = 0; // 底座高
        float m_lowestHipHeight = 0;
        float m_lowestKneeHeight = 0;
        FscLowerBodyInfo bodyInfo;
        FscAlgorithm* fsc;
        int m_timestempS;
        int m_timestemp1;

        float m_slideSpeed = 0;
        float m_standHeight = 0;
        float m_slideHeight = 0;
    };

}

#endif // ROBOT_H