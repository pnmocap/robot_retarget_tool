#include "robot.h"
#include "ik.h"
#include "mathutils.h"
#include"FscAlgorithmImpl.h"
#include"MotionAdaptorLib.h"
#include <iostream>
#include <sstream>
#include <Eigen/Eigen>
#ifndef WIN32
#define sscanf_s sscanf
#endif
using namespace MocapApi;

#define DATA_CACHE_SIZE 3

namespace eba {

    static const char* NoitomBoneNames[59] =
    {
        "Hips",
        "RightUpLeg",
        "RightLeg",
        "RightFoot",
        "LeftUpLeg",
        "LeftLeg",
        "LeftFoot",
        "Spine",
        "Spine1",
        "Spine2",
        "Neck",
        "Neck1",
        "Head",
        "RightShoulder",
        "RightArm",
        "RightForeArm",
        "RightHand",
        "RightHandThumb1",
        "RightHandThumb2",
        "RightHandThumb3",
        "RightInHandIndex",
        "RightHandIndex1",
        "RightHandIndex2",
        "RightHandIndex3",
        "RightInHandMiddle",
        "RightHandMiddle1",
        "RightHandMiddle2",
        "RightHandMiddle3",
        "RightInHandRing",
        "RightHandRing1",
        "RightHandRing2",
        "RightHandRing3",
        "RightInHandPinky",
        "RightHandPinky1",
        "RightHandPinky2",
        "RightHandPinky3",
        "LeftShoulder",
        "LeftArm",
        "LeftForeArm",
        "LeftHand",
        "LeftHandThumb1",
        "LeftHandThumb2",
        "LeftHandThumb3",
        "LeftInHandIndex",
        "LeftHandIndex1",
        "LeftHandIndex2",
        "LeftHandIndex3",
        "LeftInHandMiddle",
        "LeftHandMiddle1",
        "LeftHandMiddle2",
        "LeftHandMiddle3",
        "LeftInHandRing",
        "LeftHandRing1",
        "LeftHandRing2",
        "LeftHandRing3",
        "LeftInHandPinky",
        "LeftHandPinky1",
        "LeftHandPinky2",
        "LeftHandPinky3",
    };


    Robot::Robot()
        : m_rootRotation(glm::identity<Quat>())
        , m_rootPosition(Vector3(0))
        , m_shoulderAngle(0)
        , m_lowerLegLength(0)
        , m_upperLegLength(0)
    {
    }

    Robot::~Robot()
    {

    }

    bool Robot::Load(const char* jsonConfigStr)
    {
        Unload();
        try {
            auto j = nlohmann::json::parse(jsonConfigStr);
            m_config = j.get<RetargetingConfig>();
        }
        catch (...) {
            return false;
        }
        m_shoulderAngle = m_config.shoulderAngle;
        m_startRecordTime = std::chrono::steady_clock::now();

        m_lowerLegLength = 0; // reset
        m_upperLegLength = 0; // reset

        locked_left_j1_pitch_value = 0;
        locked_right_j1_pitch_value = 0;

        if (m_config.lowerBodyMode == 1)
        {
            for (int i = 0; i < DummyNodeCount; ++i)
            {
                m_dummyNodes[i] = std::make_shared<JointNode>();
            }

            // 根据urdf构建骨骼树
            m_dummyNodes[DummyLeg1]->SetParent(m_dummyNodes[DummyRoot].get());
            m_dummyNodes[DummyLeg2]->SetParent(m_dummyNodes[DummyLeg1].get());
            m_dummyNodes[DummySpine]->SetParent(m_dummyNodes[DummyLeg2].get());
            m_dummyNodes[DummyHead]->SetParent(m_dummyNodes[DummySpine].get());

            m_dummyNodes[DummyLeg1]->SetPosition(0.134f, 0, 0.13f);
            //m_dummyNodes[DummyLeg1]->SetRotation(Quat(0.13528575f, -0.13528575f, -0.6940445f, -0.6940445f));

            m_dummyNodes[DummyLeg2]->SetPosition(-0.28305584f, 0, 0.29900977f);
            //m_dummyNodes[DummyLeg2]->SetRotation(Quat(-0.70710677f, -0.70710677f, 8.130836e-18f, 0));

            auto combineSpine = Quat(-5.1428638e-17f, 4.3297803e-17f, -0.70710677f, -0.70710677f) * Quat(-0.99999994f, 0, 5.7493685e-18f, 5.7493685e-18f);
            //m_dummyNodes[DummySpine]->SetRotation(combineSpine);
            m_dummyNodes[DummySpine]->SetPosition(0.106944144f, -4.484508e-18f, 0.29900977f);

            auto combineHead = Quat(-0.99999994f, 0, 5.7493685e-18f, 5.7493685e-18f) * Quat(-0.7071067f, 0.7071067f, 0, 8.130835e-18f);
            //m_dummyNodes[DummyHead]->SetRotation(combineHead);
            m_dummyNodes[DummyHead]->SetPosition(0.09877724f, 5e-05f, 0.8829498f);

            // 系数防止折叠过直跳变
            m_baseHeight = m_dummyNodes[DummyLeg1]->GetPosition().z;

            m_lowestHipHeight = m_dummyNodes[DummySpine]->GetPosition().z;
            m_lowestKneeHeight = m_dummyNodes[DummyLeg2]->GetPosition().z;

            m_dummyKneeCtrl = std::make_shared<JointNode>();
            m_dummyHipCtrl = std::make_shared<JointNode>();

            m_dummyKneeCtrl->SetParent(m_dummyNodes[DummyRoot].get());
            m_dummyHipCtrl->SetParent(m_dummyNodes[DummyRoot].get());

            m_dummyKneeCtrl->SetPosition(m_dummyNodes[DummyLeg2]->GetPosition() - Vector3(0.5f, 0, 0)); // bind point
            m_dummyHipCtrl->SetPosition(m_dummyNodes[DummySpine]->GetPosition()); // target point
            m_initCentroidHorizontalOffset = m_dummyHipCtrl->GetPosition().x;

            m_robotLegLength1 = glm::length(m_dummyNodes[DummyLeg2]->GetPosition() - m_dummyNodes[DummyLeg1]->GetPosition());
            m_robotLegLength2 = glm::length(m_dummyNodes[DummySpine]->GetPosition() - m_dummyNodes[DummyLeg2]->GetPosition());
            m_robotLegLength = m_robotLegLength1 + m_robotLegLength2;

        }

        if (m_config.hipsHeight!=0)
        {
            targetHipsWidth = m_config.hipsHeight;
            targetHipsHeight = m_config.hipsHeight;
            targetLegLength = m_config.legLength;
            targetAnkleHeight = m_config.ankleHeight;
            m_enableScale = true;
        }
        else
        {
            m_enableScale = false;
        }


        m_firstUpdate = true;

        return true;
    }

    MocapApi::EMCPError Robot::Update(MocapApi::IMCPAvatar* mcpAvatar, MocapApi::MCPAvatarHandle_t avatarHandle, MocapApi::IMCPJoint* jointInterface)
    {
        if (mcpAvatar == nullptr)
            return MocapApi::Error_InvalidObject;

        MocapApi::EMCPError result = MocapApi::EMCPError::Error_None;
        uint32_t JointCount = 0;
        MocapApi::EMCPJointTag jointTag;
        auto error = mcpAvatar->GetAvatarJoints(nullptr, &JointCount, avatarHandle);

        std::vector<MocapApi::MCPJointHandle_t> jointHandles(JointCount, 0);
        error = mcpAvatar->GetAvatarJoints(jointHandles.data(), &JointCount, avatarHandle);
        for (auto& jointHandle : jointHandles)
        {
            jointInterface->GetJointTag(&jointTag, jointHandle);

            MocapData mocapData;

            // world
            {
                Quat& rot = mocapData.worldRot;
                Vector3& pos = mocapData.worldPos;

                float r_x, r_y, r_z, r_w;
                error = jointInterface->GetJointGlobalRotation(&r_x, &r_y, &r_z, &r_w, jointHandle);
                if (error != MocapApi::Error_None)
                    result = error;

                // 动捕转内部计算坐标系
                rot.x = -r_z;
                rot.y = r_x;
                rot.z = -r_y;
                rot.w = r_w;

//                 assert(!glm::isnan(rot.x));
//                 assert(!glm::isnan(rot.y));
//                 assert(!glm::isnan(rot.z));
//                 assert(!glm::isnan(rot.w));

                float p_x, p_y, p_z;
                error = jointInterface->GetJointGlobalPosition(&p_x, &p_y, &p_z, jointHandle);
                if (error != MocapApi::Error_None)
                    result = error;

                pos.x = p_z * 0.01;
                pos.y = p_x * 0.01;
                pos.z = p_y * 0.01;

                // convert to robot coordinate
                if (!m_enableScale)
                {
                    if (m_config.lowerBodyMode==0)
                    {
                        if (jointTag == MocapApi::JointTag_Hips)
                        {
                            m_rootPosition = pos;
                            m_rootRotation = rot;
                        }
                    }
                   
                }
            }
            // local
            {
                Quat& rot = mocapData.localRot;
                Vector3& pos = mocapData.localPos;

                float r_x, r_y, r_z, r_w;
                error = jointInterface->GetJointLocalRotation(&r_x, &r_y, &r_z, &r_w, jointHandle);
                if (error != MocapApi::Error_None)
                    result = error;

                rot.x = -r_z;
                rot.y = r_x;
                rot.z = -r_y;
                rot.w = r_w;

//                 assert(!glm::isnan(rot.x));
//                 assert(!glm::isnan(rot.y));
//                 assert(!glm::isnan(rot.z));
//                 assert(!glm::isnan(rot.w));

                float p_x, p_y, p_z;
                error = jointInterface->GetJointLocalPosition(&p_x, &p_y, &p_z, jointHandle);
                if (error != MocapApi::Error_None)
                    result = error;

                pos.x = p_z * 0.01;
                pos.y = p_x * 0.01;
                pos.z = p_y * 0.01;
            }

            if (m_mocapInputCache[jointTag].size() == 0)
            {
                for (int i = 0; i < DATA_CACHE_SIZE; ++i)
                    m_mocapInputCache[jointTag].push(mocapData); // 直接填满
            }
            else
                m_mocapInputCache[jointTag].push(std::move(mocapData));

            while (m_mocapInputCache[jointTag].size() > DATA_CACHE_SIZE)
                m_mocapInputCache[jointTag].pop();
        }

        return result;
    }

    void Robot::Update(const MocapData* frameData)
    {
        for (int joint = 0; joint < JOINT_COUNT; ++joint)
        {
            MocapData mocapData;

            // world
            {
                Quat& rot = mocapData.worldRot;
                Vector3& pos = mocapData.worldPos;

                float r_x = frameData[joint].worldRot.x;
                float r_y = frameData[joint].worldRot.y;
                float r_z = frameData[joint].worldRot.z;
                float r_w = frameData[joint].worldRot.w;

                // 动捕转内部计算坐标系
                rot.x = -r_z;
                rot.y = r_x;
                rot.z = -r_y;
                rot.w = r_w;

//                 assert(!glm::isnan(rot.x));
//                 assert(!glm::isnan(rot.y));
//                 assert(!glm::isnan(rot.z));
//                 assert(!glm::isnan(rot.w));

                float p_x, p_y, p_z;
                p_x = frameData[joint].worldPos.x;
                p_y = frameData[joint].worldPos.y;
                p_z = frameData[joint].worldPos.z;

                pos.x = p_z * 0.01;
                pos.y = p_x * 0.01;
                pos.z = p_y * 0.01;

                // convert to robot coordinate
                if (!m_enableScale)
                {
                    if (joint == 0)
                    {
                        m_rootPosition = pos;
                        m_rootRotation = rot;
                    }
                }
            }
            // local
            {
                Quat& rot = mocapData.localRot;
                Vector3& pos = mocapData.localPos;

                float r_x, r_y, r_z, r_w;
                r_x = frameData[joint].localRot.x;
                r_y = frameData[joint].localRot.y;
                r_z = frameData[joint].localRot.z;
                r_w = frameData[joint].localRot.w;

                rot.x = -r_z;
                rot.y = r_x;
                rot.z = -r_y;
                rot.w = r_w;

//                 assert(!glm::isnan(rot.x));
//                 assert(!glm::isnan(rot.y));
//                 assert(!glm::isnan(rot.z));
//                 assert(!glm::isnan(rot.w));

                float p_x, p_y, p_z;
                p_x = frameData[joint].localPos.x;
                p_y = frameData[joint].localPos.y;
                p_z = frameData[joint].localPos.z;

                pos.x = p_z * 0.01;
                pos.y = p_x * 0.01;
                pos.z = p_y * 0.01;
            }

            if (m_mocapInputCache[joint].size() == 0)
            {
                for (int i = 0; i < DATA_CACHE_SIZE; ++i)
                    m_mocapInputCache[joint].push(mocapData); // 直接填满
            }
            else
                m_mocapInputCache[joint].push(std::move(mocapData));

            while (m_mocapInputCache[joint].size() > DATA_CACHE_SIZE)
                m_mocapInputCache[joint].pop();
        }
    }

    void Robot::Step(float fixedDelta /*= 0*/)
    {

        bool gotData = false;
        {
            std::lock_guard<std::mutex> lock(m_queueLock);

            for (int i = 0; i < (int)MocapApi::EMCPJointTag::JointTag_JointsCount; ++i)
            {
                if (m_mocapInputCache[i].size() >= 3)
                {
                    auto& mocapData = m_mocapInputCache[i].front();
                    m_mocapInputRot[i] = mocapData.localRot;
                    m_mocapInputPos[i] = mocapData.localPos;
                    m_mocapInputRotWorld[i] = mocapData.worldRot;
                    m_mocapInputPosWorld[i] = mocapData.worldPos;
                    m_mocapInputCache[i].pop();
                    gotData = true;
                }
            }
        }

        if (!gotData)
            return;


        if (fixedDelta <= 0)
        {
            solveDeltaTime();
        }
        else
        {
            m_deltaTime = fixedDelta;
        }

        preprocess();

        if (m_firstUpdate)
        {
            m_firstUpdate = false;
            onFirstFrame();
        }
        if (!m_firstUpdate)
        {
            if (m_enableScale)
            {
                SetUpdateWithDisp();
            }
        }

        solveVirtualBones();

        solveHumanoid();

        postprocess();

        // 计算时间戳
        if (fixedDelta <= 0)
        {
            auto now = std::chrono::steady_clock::now();
            auto stamp_s = std::chrono::duration_cast<std::chrono::seconds>(now - m_startRecordTime).count();
            auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - m_startRecordTime).count();
            auto stamp_1 = (float)stamp_ns * 1e-9 - stamp_s;
            m_timestempS = (int)stamp_s;
            m_timestemp1 = (int)(stamp_1 * 1e9);
        }
    }

    void Robot::preprocess()
    {
        // 因为大腿肌肉在抬腿时候有内旋，需要用小腿先约束一下
         // left leg fix
        {
            auto left_upleg_rot = m_mocapInputRot[JointTag_LeftUpLeg];
            auto left_leg_rot = left_upleg_rot * m_mocapInputRot[JointTag_LeftLeg];
            auto left_leg_Y = left_leg_rot * Vector3(0, 1, 0);
            auto left_leg_Z = left_upleg_rot * Vector3(0, 0, 1);
            auto left_leg_X = glm::normalize(glm::cross(left_leg_Y, left_leg_Z));
            auto left_leg_fixed_rot = quatFromXZ(left_leg_X, left_leg_Z);
            m_mocapInputRot[JointTag_LeftUpLeg] = left_leg_fixed_rot;
        }

        // right leg fix
        {
            auto right_upleg_rot = m_mocapInputRot[JointTag_RightUpLeg];
            auto right_leg_rot = right_upleg_rot * m_mocapInputRot[JointTag_RightLeg];
            auto right_leg_Y = right_leg_rot * Vector3(0, 1, 0);
            auto right_leg_Z = right_upleg_rot * Vector3(0, 0, 1);
            auto right_leg_X = glm::normalize(glm::cross(right_leg_Y, right_leg_Z));
            auto right_leg_fixed_rot = quatFromXZ(right_leg_X, right_leg_Z);
            m_mocapInputRot[JointTag_RightUpLeg] = right_leg_fixed_rot;
        }

        // do left arm IK，用于去掉反关节
        if (1)
        {
            const auto& leftArmRot = m_mocapInputRot[JointTag_LeftArm];
            const auto& leftForeArmRot = m_mocapInputRot[JointTag_LeftForeArm];
            const auto& leftForeArmLoc = m_mocapInputPos[JointTag_LeftForeArm];
            const auto& leftHandRot = m_mocapInputRot[JointTag_LeftHand];
            const auto& leftHandLoc = m_mocapInputPos[JointTag_LeftHand];

            IkTransform InOutRootTransform;
            InOutRootTransform.position = { 0, 0, 0 };
            InOutRootTransform.rotation = leftArmRot;

            IkTransform InOutJointTransform;
            InOutJointTransform.position = leftForeArmLoc;
            InOutJointTransform.rotation = leftForeArmRot;
            InOutJointTransform = InOutRootTransform * InOutJointTransform;

            IkTransform IKJointTargetTransform;
            IKJointTargetTransform.position = leftForeArmLoc + Vector3(0.1, 0, 0);
            IKJointTargetTransform.rotation = leftForeArmRot;
            auto JointTarget = (InOutRootTransform * IKJointTargetTransform).position;

            IkTransform InOutEndTransform;
            InOutEndTransform.position = leftHandLoc;
            InOutEndTransform.rotation = leftHandRot;
            InOutEndTransform = InOutJointTransform * InOutEndTransform;

            IkTransform InOutEndTargetTransform;
            InOutEndTargetTransform.position = leftHandLoc;
            InOutEndTargetTransform.rotation = leftHandRot;
            InOutEndTargetTransform = InOutJointTransform * InOutEndTargetTransform;

            auto Effector = InOutEndTargetTransform.position;
            bool bAllowStretching = true;
            float StartStretchRatio = 0.98f;
            float MaxStretchScale = 1.0;
            TwoBonesIK(InOutRootTransform, InOutJointTransform, InOutEndTransform, JointTarget, Effector, bAllowStretching, StartStretchRatio, MaxStretchScale);

            //m_mocapInputRot[JointTag_LeftArm] = InOutRootTransform.rotation;

            IkTransform forarmRel;
            forarmRel = InOutJointTransform.GetRelativeTransform(InOutRootTransform);

            m_mocapInputRot[JointTag_LeftForeArm] = forarmRel.rotation;
            //m_mocapInputPos[JointTag_LeftForeArm] = forarmRel.position;

            IkTransform handRel;
            handRel = InOutEndTransform.GetRelativeTransform(InOutJointTransform);

            //m_mocapInputRot[JointTag_LeftHand] = handRel.rotation;
            //m_mocapInputPos[JointTag_LeftHand] = handRel.position;

//             assert(!glm::isnan(forarmRel.rotation.x));
//             assert(!glm::isnan(forarmRel.rotation.y));
//             assert(!glm::isnan(forarmRel.rotation.z));
//             assert(!glm::isnan(forarmRel.rotation.w));
        }

        // do right arm IK
        if (1)
        {
            const auto& rightArmRot = m_mocapInputRot[JointTag_RightArm];
            const auto& rightForeArmRot = m_mocapInputRot[JointTag_RightForeArm];
            const auto& rightForeArmLoc = m_mocapInputPos[JointTag_RightForeArm];
            const auto& rightHandRot = m_mocapInputRot[JointTag_RightHand];
            const auto& rightHandLoc = m_mocapInputPos[JointTag_RightHand];

            IkTransform InOutRootTransform;
            InOutRootTransform.position = { 0, 0, 0 };
            InOutRootTransform.rotation = rightArmRot;

            IkTransform InOutJointTransform;
            InOutJointTransform.position = rightForeArmLoc;
            InOutJointTransform.rotation = rightForeArmRot;
            InOutJointTransform = InOutRootTransform * InOutJointTransform;

            IkTransform IKJointTargetTransform;
            IKJointTargetTransform.position = rightForeArmLoc + Vector3(0.1, 0, 0);
            IKJointTargetTransform.rotation = rightForeArmRot;
            auto JointTarget = (InOutRootTransform * IKJointTargetTransform).position;

            IkTransform InOutEndTransform;
            InOutEndTransform.position = rightHandLoc;
            InOutEndTransform.rotation = rightHandRot;
            InOutEndTransform = InOutJointTransform * InOutEndTransform;

            IkTransform InOutEndTargetTransform;
            InOutEndTargetTransform.position = rightHandLoc;
            InOutEndTargetTransform.rotation = rightHandRot;
            InOutEndTargetTransform = InOutJointTransform * InOutEndTargetTransform;

            auto Effector = InOutEndTargetTransform.position;
            bool bAllowStretching = true;
            float StartStretchRatio = 0.98f;
            float MaxStretchScale = 1.0;
            TwoBonesIK(InOutRootTransform, InOutJointTransform, InOutEndTransform, JointTarget, Effector, bAllowStretching, StartStretchRatio, MaxStretchScale);

            //m_mocapInputRot[JointTag_RightArm] = InOutRootTransform.rotation;

            IkTransform forarmRel;
            forarmRel = InOutJointTransform.GetRelativeTransform(InOutRootTransform);

            m_mocapInputRot[JointTag_RightForeArm] = forarmRel.rotation;
            //m_mocapInputPos[JointTag_RightForeArm] = forarmRel.position;

            IkTransform handRel;
            handRel = InOutEndTransform.GetRelativeTransform(InOutJointTransform);

            //m_mocapInputRot[JointTag_RightHand] = handRel.rotation;
            //m_mocapInputPos[JointTag_RightHand] = handRel.position;

//             assert(!glm::isnan(forarmRel.rotation.x));
//             assert(!glm::isnan(forarmRel.rotation.y));
//             assert(!glm::isnan(forarmRel.rotation.z));
//             assert(!glm::isnan(forarmRel.rotation.w));
        }
    }

    void Robot::solveHumanoid()
    {
        float temp_angle = 0.0f;
        // 躯干
        if (1)
        {
            auto mocap_spine_rot = m_mocapInputRot[JointTag_Spine];
            auto mocap_spine_rot1 = m_mocapInputRot[JointTag_Spine1];
            auto mocap_spine_rot2 = m_mocapInputRot[JointTag_Spine2];
            auto mocap_spine_total_rot = mocap_spine_rot * mocap_spine_rot1 * mocap_spine_rot2;
            Vector3 axis(0);
            axis[m_config.spineOrder.axis1] = 1;
            if (m_config.spineOrder == EulerOrder{ 2, 0, 1 })
            {
                mocap_spine_total_rot = glm::angleAxis(glm::radians(90.0f), axis) * mocap_spine_total_rot;
                auto mocap_spine_total_euler = QuatToEuler(mocap_spine_total_rot, m_config.spineOrder);
                SetRawJointAngle(JointSpine_Yaw, mocap_spine_total_euler[0] - glm::radians(90.0));
                SetRawJointAngle(JointSpine_Roll, mocap_spine_total_euler[1]);
                SetRawJointAngle(JointSpine_Pitch, mocap_spine_total_euler[2]);
                temp_angle= mocap_spine_total_euler[2];
            }
            else if (m_config.spineOrder == EulerOrder{ 0, 2, 1 })
            {
                mocap_spine_total_rot = glm::angleAxis(glm::radians(90.0f), axis) * mocap_spine_total_rot;
                auto mocap_spine_total_euler = QuatToEuler(mocap_spine_total_rot, m_config.spineOrder);
                SetRawJointAngle(JointSpine_Roll, mocap_spine_total_euler[0] - glm::radians(90.0));
                SetRawJointAngle(JointSpine_Yaw, mocap_spine_total_euler[1]);
                SetRawJointAngle(JointSpine_Pitch, mocap_spine_total_euler[2]);
                temp_angle = mocap_spine_total_euler[2];
            }
            else if (m_config.spineOrder == EulerOrder{ 1, 0, 2 })
            {
                mocap_spine_total_rot = glm::angleAxis(glm::radians(90.0f), axis) * mocap_spine_total_rot;
                auto mocap_spine_total_euler = QuatToEuler(mocap_spine_total_rot, m_config.spineOrder);
                SetRawJointAngle(JointSpine_Pitch, mocap_spine_total_euler[0] - glm::radians(90.0));
                SetRawJointAngle(JointSpine_Roll, mocap_spine_total_euler[1]);
                SetRawJointAngle(JointSpine_Yaw, mocap_spine_total_euler[2]);
                temp_angle = mocap_spine_total_euler[2];
            }
        }

        // 头部
        if (1)
        {
            //             auto mocap_neck_rot = m_mocapInputRot[JointTag_Neck];
            //             auto mocap_neck1_rot = m_mocapInputRot[JointTag_Neck1];
            //             auto mocap_head_rot = m_mocapInputRot[JointTag_Head];
            auto mocap_neck_total_rot = glm::inverse(m_mocapInputRotWorld[JointTag_Spine2]) * m_mocapInputRotWorld[JointTag_Head];
            //mocap_neck_rot * mocap_neck1_rot * mocap_head_rot;

            Matrix3 rotMat = glm::toMat3(mocap_neck_total_rot);
            Vector3 xAxis = rotMat[0];

            float offset = 0.0;
            if (xAxis.y >= 0) // 右旋
            {
                offset = 30;
            }
            else // 左旋
            {
                offset = 150.0;
            }

            Vector3 axis(0);
            axis[m_config.headOrder.axis1] = 1;
            mocap_neck_total_rot = glm::angleAxis(glm::radians(offset), axis) * mocap_neck_total_rot;
            auto mocap_neck_total_euler = QuatToEuler(mocap_neck_total_rot, m_config.headOrder);
            SetRawJointAngle(JointHead_Yaw, -mocap_neck_total_euler[0] + glm::radians(offset));
            SetRawJointAngle(JointHead_Roll, mocap_neck_total_euler[1]);
            SetRawJointAngle(JointHead_Pitch, mocap_neck_total_euler[2]);
        }

        // 左大腿
        if (1)
        {
            auto mocap_upleg_rot = m_mocapInputRot[JointTag_LeftUpLeg];
            Vector3 axisOffset(0);
            axisOffset[m_config.hipOrder.axis1] = 1;
            if (m_config.hipOrder == EulerOrder{ 2, 1, 0 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset;
                if (xAxis.y <= 0) // 外旋
                {
                    offset = 175.0;
                }
                else // 内旋
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot; // 哪个轴开始有跳变，就转个90规避
                auto mocap_upleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder); // 根据机械结构确定轴序
                SetRawJointAngle(JointLeftHip_Yaw, mocap_upleg_euler[0] - glm::radians(offset)); // 相应的折算回偏移
                SetRawJointAngle(JointLeftHip_Pitch, mocap_upleg_euler[1]);
                SetRawJointAngle(JointLeftHip_Roll, mocap_upleg_euler[2]);
            }
            else if (m_config.hipOrder == EulerOrder{ 2, 0, 1 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset;
                if (xAxis.y <= 0) // 外旋
                {
                    offset = 175.0;
                }
                else // 内旋
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot; // 哪个轴开始有跳变，就转个90规避
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder); // 根据机械结构确定轴序
                SetRawJointAngle(JointLeftHip_Yaw, mocap_uleg_euler[0] - glm::radians(offset)); // 相应的折算回偏移
                SetRawJointAngle(JointLeftHip_Roll, mocap_uleg_euler[1]);
                SetRawJointAngle(JointLeftHip_Pitch, mocap_uleg_euler[2]);
            }
            else if (m_config.hipOrder == EulerOrder{ 1, 2, 0 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 175.0;
                }
                else // 后压腿
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointLeftHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset) - temp_angle);
                SetRawJointAngle(JointLeftHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointLeftHip_Roll, mocap_uleg_euler[2]);
            }
            else if (m_config.hipOrder == EulerOrder{ 0, 2, 1 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 150.0;
                }
                else // 后压腿
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointLeftHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset));
                SetRawJointAngle(JointLeftHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointLeftHip_Roll, mocap_uleg_euler[2]);
            }
            //增加1  0 2结构
            else if (m_config.hipOrder == EulerOrder{ 1, 0, 2 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 150.0;
                }
                else // 后压腿
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointLeftHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset));
                SetRawJointAngle(JointLeftHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointLeftHip_Roll, mocap_uleg_euler[2]);


            }
        }

        // 左小腿
        if (1)
        {
            auto mocap_leg_rot = m_mocapInputRot[JointTag_LeftLeg];
            mocap_leg_rot = glm::angleAxis(glm::radians(30.0f), Vector3(0, 1, 0)) * mocap_leg_rot;
            auto mocap_leg_euler = QuatToEuler(mocap_leg_rot, { 1, 0, 2 });
            SetRawJointAngle(JointLeftKnee, -mocap_leg_euler[0] + glm::radians(30.0));
        }

        // 左脚
        if (1)
        {
            auto mocap_left_foot_rot = m_mocapInputRot[JointTag_LeftFoot];
            Vector3 axis(0);
            axis[m_config.ankelOrder.axis1] = 1;
            mocap_left_foot_rot = glm::angleAxis(glm::radians(90.0f), axis) * mocap_left_foot_rot;
            auto mocap_left_foot_euler = QuatToEuler(mocap_left_foot_rot, m_config.ankelOrder);
            SetRawJointAngle(JointLeftAnkle_Pitch, mocap_left_foot_euler[0] - glm::radians(90.0));
            SetRawJointAngle(JointLeftAnkle_Roll, mocap_left_foot_euler[1]);
        }

        float threshold_angle = 5.0f;

        // 左上臂 #方案1
        if (1)
        {
            // 先计算bvh的上臂相对躯干的旋转
            auto arm_q_2_spine = glm::inverse(m_mocapInputRotWorld[JointTag_Spine2]) * m_mocapInputRotWorld[JointTag_LeftArm];

            // 再计算机器人肩部相对于躯干的旋转（上旋30°）
            auto robot_shoulder = glm::angleAxis(glm::radians(-(float)m_shoulderAngle), Vector3(1, 0, 0));

            // 计算bvh相对于机器人的旋转
            auto robot_arm = glm::inverse(robot_shoulder) * arm_q_2_spine;

            // 防止跳变
            auto arm_dir = arm_q_2_spine * Vector3(0, -1, 0);
            float offset = 0;
            if (arm_dir.x > 0)
            {
                offset = 175.0;
            }
            else
            {
                offset = 90.0;
            }
            robot_arm = glm::angleAxis(glm::radians(offset), Vector3(0, 1, 0)) * robot_arm;

            auto robot_arm_euler = QuatToEuler(robot_arm, { 1, 0, 1 });

            float pitch = robot_arm_euler[0] - glm::radians(offset);
            float roll = -robot_arm_euler[1] + glm::radians(90.0 + m_shoulderAngle);
            float yaw = robot_arm_euler[2] + glm::radians(90.0);

            SetRawJointAngle(JointLeftShoulder_Pitch, pitch);
            SetRawJointAngle(JointLeftShoulder_Roll, roll);
            SetRawJointAngle(JointLeftShoulder_Yaw, yaw);

            // 平举奇异解处理
            if (m_shoulderAngle <= 0)
            {
                float j1_angle = roll - glm::radians(90.0);
                if (glm::abs(j1_angle) < glm::radians(threshold_angle) && glm::abs(locked_left_j1_pitch_value) > 0)
                {
                    SetRawJointAngle(JointLeftShoulder_Pitch, locked_left_j1_pitch_value);
                    float delta = pitch - locked_left_j1_pitch_value;
                    SetRawJointAngle(JointLeftShoulder_Yaw, yaw + delta);
                }
                else
                {
                    locked_left_j1_pitch_value = pitch;
                }
            }

        }

        // 左前臂
        if (1)
        {
            auto offset = glm::radians(100.0f);
            auto mocap_forearm_rot = m_mocapInputRot[JointTag_LeftForeArm];
            mocap_forearm_rot = glm::angleAxis(offset, Vector3(0, 1, 0)) * mocap_forearm_rot;
            auto mocap_forearm_euler = QuatToEuler(mocap_forearm_rot, { 1, 2, 1 });
            float elbowAngle = mocap_forearm_euler[1]; //CalculElbowAngle(true, mocap_forearm_rot);
            SetRawJointAngle(JointLeftElbow, elbowAngle);
            // 胳膊肘奇异
            if (glm::abs(elbowAngle) < glm::radians(1.0f))
            {
                float j3;
                GetRawJointAngleInternal(JointLeftShoulder_Yaw, &j3);
                SetRawJointAngle(JointLeftShoulder_Yaw, Clamp180(0 + j3 - glm::radians(90.0)));
                //  SetRawJointAngle(JointLeftElbow, 0);

                SetRawJointAngle(JointLeftWrist_Yaw, 0);
            }
            else
            {
                float j3;
                GetRawJointAngleInternal(JointLeftShoulder_Yaw, &j3);
                SetRawJointAngle(JointLeftShoulder_Yaw, Clamp180(mocap_forearm_euler[0] - offset + j3 - glm::radians(90.0)));


                SetRawJointAngle(JointLeftWrist_Yaw, mocap_forearm_euler[2]);
            }
        }

        // 左手
        if (1)
        {
            auto mocap_hand_rot = m_mocapInputRot[JointTag_LeftHand];
            Vector3 axis(0);
            axis[m_config.wristOrder.axis1] = 1;

            auto hand_rot_mtx = glm::toMat3(mocap_hand_rot);
            auto hand_x = hand_rot_mtx[0];

            float offset;
            if (hand_x.z > 0.0f) // 外旋
            {
                offset = 170.0;
            }
            else
            {
                offset = 90.0;
            }
            mocap_hand_rot = glm::angleAxis(glm::radians(offset), axis) * mocap_hand_rot;
            auto mocap_hand_euler = QuatToEuler(mocap_hand_rot, m_config.wristOrder);

            float j5;
            GetRawJointAngleInternal(JointLeftWrist_Yaw, &j5);
            j5 = Clamp180((float)(mocap_hand_euler[0] - glm::radians(offset)) + j5);
            SetRawJointAngle(JointLeftWrist_Yaw, j5);
            SetRawJointAngle(JointLeftWrist_Roll, -mocap_hand_euler[1]);
            SetRawJointAngle(JointLeftWrist_Pitch, mocap_hand_euler[2]);
        }

        // 右大腿
        if (1)
        {
            auto mocap_upleg_rot = m_mocapInputRot[JointTag_RightUpLeg];
            Vector3 axisOffset(0);
            axisOffset[m_config.hipOrder.axis1] = 1;
            if (m_config.hipOrder == EulerOrder{ 2, 1, 0 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset;
                if (xAxis.y > 0) // 外旋
                {
                    offset = 5.0;
                }
                else // 内旋
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_right_upleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointRightHip_Yaw, (mocap_right_upleg_euler[0] - glm::radians(offset)));
                SetRawJointAngle(JointRightHip_Pitch,mocap_right_upleg_euler[1]);
                SetRawJointAngle(JointRightHip_Roll, mocap_right_upleg_euler[2]);
            }
            else if (m_config.hipOrder == EulerOrder{ 2, 0, 1 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset;
                if (xAxis.y > 0) // 外旋
                {
                    offset = 5.0;
                }
                else // 内旋
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_right_upleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointRightHip_Yaw, (mocap_right_upleg_euler[0] - glm::radians(offset)));
                SetRawJointAngle(JointRightHip_Roll, mocap_right_upleg_euler[1]);
                SetRawJointAngle(JointRightHip_Pitch, mocap_right_upleg_euler[2]);
            }
            else if (m_config.hipOrder == EulerOrder{ 1, 2, 0 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 175.0;
                }
                else // 后压腿
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointRightHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset) -temp_angle);
                SetRawJointAngle(JointRightHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointRightHip_Roll, mocap_uleg_euler[2] );
                

                // hips反向补偿pitch角度
                if (temp_angle != 0.0f) {

                    // 1) 反向补偿 hips 的旋转（绕 pitch 轴）
                    m_mocapInputRot[JointTag_Hips] = glm::angleAxis(temp_angle*0.7f, axisOffset) * m_mocapInputRot[JointTag_Hips];

                    // 2) 用 hips->leftUpLeg 的向量计算旋转前后差值，并同时补偿 y、z 到 hips 位置
                    Vector3 hipsPos = m_mocapInputPosWorld[JointTag_Hips];
                    Vector3 upLegPos = m_mocapInputPosWorld[JointTag_LeftUpLeg];
                    Vector3 hipsToUpLegBefore = upLegPos - hipsPos;
                    Vector3 hipsToUpLegAfter = glm::angleAxis(temp_angle, axisOffset) * hipsToUpLegBefore;

                    // 差值（旋转导致的位移变化）
                    Vector3 d = hipsToUpLegAfter - hipsToUpLegBefore;

                    // 补偿 hips，使旋转后与上腿基点的相对关系在 y、z 两个方向保持一致
                    m_mocapInputPosWorld[JointTag_Hips].y -= d.y*2;
                    m_mocapInputPosWorld[JointTag_Hips].z -= d.z*2;

                    // 若需要同时考虑 x（一般绕 pitch 轴不会显著影响 x，但可按需加上）
                    // m_mocapInputPosWorld[JointTag_Hips].x -= d.x;

                    // 3) 更新根姿态
                    m_rootPosition = m_mocapInputPosWorld[JointTag_Hips];
                  
                    bodyInfo.hips.position.value[0] = m_rootPosition.x;
                    bodyInfo.hips.position.value[1] = m_rootPosition.y;
                    bodyInfo.hips.position.value[2] = m_rootPosition.z;

                    m_rootRotation = m_mocapInputRot[JointTag_Hips];

            }
            else if (m_config.hipOrder == EulerOrder{ 0, 2, 1 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 150.0;
                }
                else // 后压腿
                {
                    offset = 90.0;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointRightHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset)- temp_angle);
                SetRawJointAngle(JointRightHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointRightHip_Roll, mocap_uleg_euler[2] );

                

                }
            }
            //增加1 0 2结构
            else if (m_config.hipOrder == EulerOrder{ 0, 2, 1 })
            {
                Matrix3 rotMat = glm::toMat3(mocap_upleg_rot);
                Vector3 xAxis = rotMat[0];

                float offset = 150.0f;
                if (xAxis.z >= 0) // 前抬腿
                {
                    offset = 150.0f;
                }
                else // 后压腿
                {
                    offset = 90.0f;
                }

                mocap_upleg_rot = glm::angleAxis(glm::radians(offset), axisOffset) * mocap_upleg_rot;
                auto mocap_uleg_euler = QuatToEuler(mocap_upleg_rot, m_config.hipOrder);
                SetRawJointAngle(JointRightHip_Pitch, mocap_uleg_euler[0] - glm::radians(offset));
                SetRawJointAngle(JointRightHip_Yaw, mocap_uleg_euler[1]);
                SetRawJointAngle(JointRightHip_Roll, mocap_uleg_euler[2] );
            }
        }

        // 右小腿
        if (1)
        {
            auto mocap_leg_rot = m_mocapInputRot[JointTag_RightLeg];
            mocap_leg_rot = glm::angleAxis(glm::radians(30.0f), Vector3(0, 1, 0)) * mocap_leg_rot;
            auto mocap_leg_euler = QuatToEuler(mocap_leg_rot, { 1, 0, 2 });
            SetRawJointAngle(JointRightKnee, -mocap_leg_euler[0] + glm::radians(30.0f));
        }

        // 右脚
        if (1)
        {
            auto mocap_right_foot_rot = m_mocapInputRot[JointTag_RightFoot];
            Vector3 axis(0);
            axis[m_config.ankelOrder.axis1] = 1;
            mocap_right_foot_rot = glm::angleAxis(glm::radians(90.0f), axis) * mocap_right_foot_rot;
            auto mocap_right_foot_euler = QuatToEuler(mocap_right_foot_rot, m_config.ankelOrder);
            SetRawJointAngle(JointRightAnkle_Pitch, mocap_right_foot_euler[0] - glm::radians(90.0f));
            SetRawJointAngle(JointRightAnkle_Roll, mocap_right_foot_euler[1]);
        }

        // 右上臂 #方案1
        if (1)
        {
            // 先计算bvh的上臂相对躯干的旋转
            auto arm_q_2_spine = glm::inverse(m_mocapInputRotWorld[JointTag_Spine2]) * m_mocapInputRotWorld[JointTag_RightArm];
            //m_mocapInputRot[MocapApi::JointTag_RightShoulder] * m_mocapInputRot[MocapApi::JointTag_RightArm];

            // 再计算机器人肩部相对于躯干的旋转（上旋30°）
            auto robot_shoulder = glm::angleAxis(glm::radians((float)m_shoulderAngle), Vector3(1, 0, 0));

            // 计算bvh相对于机器人的旋转
            auto robot_arm = glm::inverse(robot_shoulder) * arm_q_2_spine;

            // 防止跳变
            auto arm_dir = arm_q_2_spine * Vector3(0, 1, 0);
            float offset = 0;
            if (arm_dir.x > 0)
            {
                offset = 175.0;
            }
            else
            {
                offset = 90.0;
            }
            robot_arm = glm::angleAxis(glm::radians(offset), Vector3(0, 1, 0)) * robot_arm;

            auto robot_arm_euler = QuatToEuler(robot_arm, { 1, 0, 1 });

            float pitch = -robot_arm_euler[0] + glm::radians(offset);
            float roll = -robot_arm_euler[1] - glm::radians(90.0 + m_shoulderAngle);
            float yaw = -robot_arm_euler[2] - glm::radians(90.0);

            SetRawJointAngle(JointRightShoulder_Pitch, pitch);
            SetRawJointAngle(JointRightShoulder_Roll, roll);
            SetRawJointAngle(JointRightShoulder_Yaw, yaw);

            // 平举奇异解处理
            if (m_shoulderAngle <= 0)
            {
                float j1_angle = roll + glm::radians(90.0);
                if (glm::abs(j1_angle) < glm::radians(threshold_angle) && glm::abs(locked_right_j1_pitch_value) > 0)
                {
                    SetRawJointAngle(JointRightShoulder_Pitch, locked_right_j1_pitch_value);
                    float delta = pitch - locked_right_j1_pitch_value;
                    SetRawJointAngle(JointRightShoulder_Yaw, yaw + delta);
                }
                else
                {
                    locked_right_j1_pitch_value = pitch;
                }
            }
        }

        // 右前臂
        if (1)
        {
            auto offset = glm::radians(80.0f);
            auto mocap_forearm_rot = m_mocapInputRot[JointTag_RightForeArm];
            mocap_forearm_rot = glm::angleAxis(offset, Vector3(0, 1, 0)) * mocap_forearm_rot;
            auto mocap_forearm_euler = QuatToEuler(mocap_forearm_rot, { 1, 2, 1 }); // yzy
            float elbowAngle = -mocap_forearm_euler[1]; //CalculElbowAngle(false, mocap_forearm_rot);
            SetRawJointAngle(JointRightElbow, elbowAngle);
            // float wristAngle = CalculWristAngle(false, mocap_forearm_rot);
            // SetRawJointAngle(JointRightWrist_Yaw, wristAngle);

             //用左胳膊比对右胳膊
             //  SetRawJointAngle(JointLeftWrist_Yaw, -mocap_forearm_euler[2]);
             // 胳膊肘奇异
            if (glm::abs(elbowAngle) < glm::radians(1.0f))
            {
                float j3;
                GetRawJointAngleInternal(JointRightShoulder_Yaw, &j3);
                SetRawJointAngle(JointRightShoulder_Yaw, Clamp180(0 + j3 + glm::radians(90.0)));

                SetRawJointAngle(JointRightWrist_Yaw, 0);
            }
            else
            {
                float j3;
                GetRawJointAngleInternal(JointRightShoulder_Yaw, &j3);
                SetRawJointAngle(JointRightShoulder_Yaw, Clamp180(-mocap_forearm_euler[0] + offset + j3 + glm::radians(90.0)));

                SetRawJointAngle(JointRightWrist_Yaw, -mocap_forearm_euler[2]);
            }
        }

        // 右手
        if (1)
        {
            auto mocap_hand_rot = m_mocapInputRot[JointTag_RightHand];
            Vector3 axis(0);
            axis[m_config.wristOrder.axis1] = 1;

            auto hand_rot_mtx = glm::toMat3(mocap_hand_rot);
            auto hand_x = hand_rot_mtx[0];
            float offset;
            if (hand_x.z > 0.0f) // 外旋
            {
                offset = 170.0;

            }
            else
            {
                offset = 90.0;
            }


            mocap_hand_rot = glm::angleAxis(glm::radians(offset), axis) * mocap_hand_rot;
            auto mocap_hand_euler = QuatToEuler(mocap_hand_rot, m_config.wristOrder);
            float j5;

            GetRawJointAngleInternal(JointRightWrist_Yaw, &j5);
            j5 = Clamp180((float)(-mocap_hand_euler[0] + glm::radians(offset)) + j5);
            SetRawJointAngle(JointRightWrist_Yaw, j5);
            SetRawJointAngle(JointRightWrist_Roll, -mocap_hand_euler[1]);
            SetRawJointAngle(JointRightWrist_Pitch, mocap_hand_euler[2]);


        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~左手~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 拇指1节2自由度
        {
            //------------------------------左拇指------------------------------
            //新的方法，将人体拇指有3个关节，机器人有四个。人体第一关节的Z轴旋转为机器人的第一关节旋转度，人体第一关节的Y轴为机器人第二关节旋转角度。
            auto thumb1_p = m_mocapInputPos[JointTag_LeftHandThumb1];
            auto thumb1_r = m_mocapInputRot[JointTag_LeftHandThumb1];
            auto thumb2_p = m_mocapInputPos[JointTag_LeftHandThumb2];
            auto thumb2_r = m_mocapInputRot[JointTag_LeftHandThumb2];
            auto thumb3_p = m_mocapInputPos[JointTag_LeftHandThumb3];
            auto thumb3_r = m_mocapInputRot[JointTag_LeftHandThumb3];
            glm::mat4 MatLocalA = glm::translate(thumb1_p) * glm::toMat4(thumb1_r);
            glm::mat4 MatLocalB = glm::translate(thumb2_p) * glm::toMat4(thumb2_r);
            glm::mat4 MatLocalC = glm::translate(thumb3_p) * glm::toMat4(thumb3_r);
            glm::mat4 leftOffsetMat = MatLocalA * MatLocalB * MatLocalC;
            glm::vec3 Pos3ByTthumb1 = leftOffsetMat * glm::vec4(0, 0.02f, 0, 1);
            Pos3ByTthumb1 = glm::normalize(Pos3ByTthumb1);
            //求第一关节与第二关节的角度y轴角度，同时也是机器人第一关节该旋转的角度
            glm::vec3 targetPosByJ1XZ = glm::vec3(Pos3ByTthumb1.x, 0, Pos3ByTthumb1.z);
            //代表是第一关节点的右方向
            glm::vec3 left = glm::vec3(-1.0f, 0.0f, 0.0f);
            float angle0 = angleBetween(left, targetPosByJ1XZ);

            //20250409针对不同机械手，拆分方式不一样。（因时，奥义使用0，哈工大使用1）
            if (m_config.handMode==0)
            {
                SetRawJointAngle(JointLeftThumb1_1, angle0);
            }
            else//针对中间为0，摆动为负、正的手
            {
                auto thumb1_1_euler = QuatToEuler(thumb1_r, { 0, 2, 1 });
                if (thumb1_1_euler[0]>=1.57)
                {
                    thumb1_1_euler[0] =   thumb1_1_euler[0]-3.14;
                }
                SetRawJointAngle(JointLeftThumb1_1, thumb1_1_euler[0] );
            }

            if (m_config.handMode == 0)
            {
                auto thumb1_2 = thumb1_r * thumb2_r;
                auto thumb1_2_euler = QuatToEuler(thumb1_2, { 1, 2, 0 });
                SetRawJointAngle(JointLeftThumb1_2, thumb1_2_euler[1]*3);
            }
            else
            {
                auto thumb1_2 = thumb1_r * thumb2_r;
               
                auto thumb1_2_euler = QuatToEuler(thumb1_2, { 0, 2, 1 });

                if (thumb1_2_euler[1] >= 1.57)
                {
                    thumb1_2_euler[1] = 3.14 - thumb1_2_euler[1];
                }
                SetRawJointAngle(JointLeftThumb1_2, thumb1_2_euler[1]);
            }
            //20250219
           



        }
        SetRawJointAngle(JointLeftThumb2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandThumb2]))));
        SetRawJointAngle(JointLeftThumb3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandThumb3]))));

        //------------------------------左食指------------------------------
        //四肢第一关节求XY平面投影角度
        auto leftIndexRot1 = m_mocapInputRot[JointTag_LeftHandIndex1];
        SetRawJointAngle(JointLeftIndex1, CalculXzPlanAngle(leftIndexRot1));
        if (m_config.handMode == 0)
        {
            //****因时、傲意无分指
            SetRawJointAngle(JointLeftIndex2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandIndex2]))));
        }
        else
        {
            //*******如果是哈工大手，则第二节点当做分指参数
            SetRawJointAngle(JointLeftIndex2, CalculXyPlanAngle(leftIndexRot1));
        }
        SetRawJointAngle(JointLeftIndex3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandIndex3]))));


        //------------------------------左中指------------------------------
        auto leftMiddleRot1 = m_mocapInputRot[JointTag_LeftHandMiddle1];
        SetRawJointAngle(JointLeftMiddle1, CalculXzPlanAngle(leftMiddleRot1));
        if (m_config.handMode==0)
        {
            SetRawJointAngle(JointLeftMiddle2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandMiddle2]))));
        }
        else
        {
            SetRawJointAngle(JointLeftMiddle2,CalculXyPlanAngle(leftMiddleRot1));
        }
        SetRawJointAngle(JointLeftMiddle3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandMiddle3]))));

        //------------------------------左无名指------------------------------
        auto leftRingRot1 = m_mocapInputRot[JointTag_LeftHandRing1];
        SetRawJointAngle(JointLeftRing1, CalculXzPlanAngle(leftRingRot1));
        if (m_config.handMode == 0)
        {
            SetRawJointAngle(JointLeftRing2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandRing2]))));
        }
        else
        {
            SetRawJointAngle(JointLeftRing2, CalculXyPlanAngle(leftRingRot1));
        }
        SetRawJointAngle(JointLeftRing3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandRing3]))));

        //------------------------------左小指------------------------------
        auto leftPinkyRot1 = m_mocapInputRot[JointTag_LeftHandPinky1];
        SetRawJointAngle(JointLeftPinky1, CalculXzPlanAngle(leftPinkyRot1));
        if (m_config.handMode==0)
        {
            SetRawJointAngle(JointLeftPinky2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandPinky2]))));
        }
        else
        {
            SetRawJointAngle(JointLeftPinky2, CalculXyPlanAngle(leftPinkyRot1));
        }
        SetRawJointAngle(JointLeftPinky3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_LeftHandPinky3]))));

          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~右手~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // 拇指1节2自由度
        {
            // //------------------------------右拇指------------------------------
            auto thumb1_p = m_mocapInputPos[JointTag_RightHandThumb1];
            auto thumb1_r = m_mocapInputRot[JointTag_RightHandThumb1];

            auto thumb2_p = m_mocapInputPos[JointTag_RightHandThumb2];
            auto thumb2_r = m_mocapInputRot[JointTag_RightHandThumb2];

            auto thumb3_p = m_mocapInputPos[JointTag_RightHandThumb3];
            auto thumb3_r = m_mocapInputRot[JointTag_RightHandThumb3];

            glm::mat4 MatLocalA = glm::translate(thumb1_p) * glm::toMat4(thumb1_r);
            glm::mat4 MatLocalB = glm::translate(thumb2_p) * glm::toMat4(thumb2_r);
            glm::mat4 MatLocalC = glm::translate(thumb3_p) * glm::toMat4(thumb3_r);
            glm::mat4 rightOffsetMat = MatLocalA * MatLocalB * MatLocalC;
            glm::vec3 Pos3ByTthumb1 = rightOffsetMat * glm::vec4(0, -0.02f, 0, 1);
            glm::vec3 targetPosByJ1XZ = glm::vec3(Pos3ByTthumb1.x, 0, Pos3ByTthumb1.z);
            //代表是第一关节点的右方向
            glm::vec3 right = glm::vec3(-1.0f, 0.0f, 0.0f);
            float angle0 = angleBetween(right, targetPosByJ1XZ);
            auto thumb1_q = m_mocapInputRot[JointTag_RightHandThumb1];

            if (m_config.handMode== 0)
            {
                SetRawJointAngle(JointRightThumb1_1, angle0);
              
            }
            else
            {
                auto thumb1_1_euler = QuatToEuler(thumb1_r, { 0, 2, 1 });
                if (thumb1_1_euler[0] >= 1.57)
                {
                    thumb1_1_euler[0] = thumb1_1_euler[0] - 3.14;
                }
                SetRawJointAngle(JointRightThumb1_1, thumb1_1_euler[0]);

              //  auto thumb1_1_euler = QuatToEuler(thumb1_r, { 1, 2, 0 });
              //  SetRawJointAngle(JointRightThumb1_1, thumb1_1_euler[2] );
            }
           
            if (m_config.handMode==0)
            {
                auto thumb1_2 = thumb1_r * thumb2_r;
                auto thumb1_2_euler = QuatToEuler(thumb1_2, { 1, 2, 0 });
               SetRawJointAngle(JointRightThumb1_2, thumb1_2_euler[1] * 3);
              
            }
            else
            {
                auto thumb1_2 = thumb1_r * thumb2_r;
               
                auto thumb1_2_euler = QuatToEuler(thumb1_2, { 0, 2, 1 });

                if (thumb1_2_euler[1]>0)
                {
                    thumb1_2_euler[1] = -thumb1_2_euler[1];
                }


                SetRawJointAngle(JointRightThumb1_2, thumb1_2_euler[1]);
            }

        }
        SetRawJointAngle(JointRightThumb2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandThumb2]))));
        SetRawJointAngle(JointRightThumb3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandThumb3]))));
        auto RightIndexRot1 = m_mocapInputRot[JointTag_RightHandIndex1];

        //------------------------------右食指------------------------------
        //与左手相反加-号
        SetRawJointAngle(JointRightIndex1, -CalculXzPlanAngle(RightIndexRot1));
        if (m_config.handMode == 0)
        {
            //****因时、傲意无分指
            SetRawJointAngle(JointRightIndex2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandIndex2]))));
        }
        else
        {
            //*******如果是哈工大手，则第二节点当做分指参数
            SetRawJointAngle(JointRightIndex2, CalculXyPlanAngle(RightIndexRot1));
        }
        SetRawJointAngle(JointRightIndex3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandIndex3]))));

        //------------------------------右中指------------------------------
        auto RightMiddleRot1 = m_mocapInputRot[JointTag_RightHandMiddle1];
        SetRawJointAngle(JointRightMiddle1, -CalculXzPlanAngle(RightMiddleRot1));
        if (m_config.handMode == 0)
        {
            //****因时、傲意无分指
            SetRawJointAngle(JointRightMiddle2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandMiddle2]))));
        }
        else
        {
            //*******如果是哈工大手，则第二节点当做分指参数
            SetRawJointAngle(JointRightMiddle2, CalculXyPlanAngle(RightMiddleRot1));
        }

        SetRawJointAngle(JointRightMiddle3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandMiddle3]))));

        //------------------------------右无名指------------------------------
        auto RightRingRot1 = m_mocapInputRot[JointTag_RightHandRing1];
        SetRawJointAngle(JointRightRing1, -CalculXzPlanAngle(RightRingRot1));
        if (m_config.handMode == 0)
        {
            //****因时、傲意无分指
            SetRawJointAngle(JointRightRing2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandRing2]))));
        }
        else
        {
            //*******如果是哈工大手，则第二节点当做分指参数
            SetRawJointAngle(JointRightRing2, CalculXyPlanAngle(RightRingRot1));
        }
        SetRawJointAngle(JointRightRing3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandRing3]))));

        //------------------------------右小指------------------------------
        auto RightPinkyRot1 = m_mocapInputRot[JointTag_RightHandPinky1];
        SetRawJointAngle(JointRightPinky1, -CalculXzPlanAngle(RightPinkyRot1));
        if (m_config.handMode == 0)
        {
            //****因时、傲意无分指
            SetRawJointAngle(JointRightPinky2, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandPinky2]))));
        }
        else
        {
            //*******如果是哈工大手，则第二节点当做分指参数
            SetRawJointAngle(JointRightPinky2, CalculXyPlanAngle(RightPinkyRot1));
        }
        SetRawJointAngle(JointRightPinky3, glm::abs(Angle360To180(glm::angle(m_mocapInputRot[JointTag_RightHandPinky3]))));
    }

    void Robot::onFirstFrame()
    {
        if (m_config.lowerBodyMode == 1)
        {
            calcWheeledPosition();
            m_dummyNodes[DummyRoot]->SetPosition(m_TargetPos);

            Quat rot = glm::angleAxis(std::atan2(-m_TargetDir.y, m_TargetDir.x), Vector3(0, 0, 1));
            m_dummyNodes[DummyRoot]->SetRotation(rot);

            m_hipsHeightStand = m_mocapInputPosWorld[JointTag_Hips].z;
            m_hipsHeightHighest = m_hipsHeightStand * m_config.hipMax;
            m_hipsHeightLowest = m_hipsHeightStand * m_config.hipMin;
        }
        else if (m_config.lowerBodyMode == 3)
        {
            m_standHeight = m_mocapInputPosWorld[JointTag_Hips].z;
            m_slideHeight = m_config.slideHeightInit;
        }
        else if (m_config.lowerBodyMode == 4)
        {
            m_standHeight = m_mocapInputPosWorld[JointTag_Hips].z;            
        }

        if (m_enableScale)
        {
            //初始化参数坐标
            bodyInfo.hips.position.value[0] = m_mocapInputPosWorld[JointTag_Hips].x;
            bodyInfo.hips.position.value[1] = m_mocapInputPosWorld[JointTag_Hips].y;
            bodyInfo.hips.position.value[2] = m_mocapInputPosWorld[JointTag_Hips].z;
            bodyInfo.hips.rotation.value[0] = m_mocapInputRotWorld[JointTag_Hips].x;
            bodyInfo.hips.rotation.value[1] = m_mocapInputRotWorld[JointTag_Hips].y;
            bodyInfo.hips.rotation.value[2] = m_mocapInputRotWorld[JointTag_Hips].z;
            bodyInfo.hips.rotation.value[3] = m_mocapInputRotWorld[JointTag_Hips].w;
            // 初始化 leftUpLeg 成员
            bodyInfo.leftUpLeg.position.value[0] = m_mocapInputPosWorld[JointTag_LeftUpLeg].x;
            bodyInfo.leftUpLeg.position.value[1] = m_mocapInputPosWorld[JointTag_LeftUpLeg].y;
            bodyInfo.leftUpLeg.position.value[2] = m_mocapInputPosWorld[JointTag_LeftUpLeg].z;
            bodyInfo.leftUpLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftUpLeg].x;
            bodyInfo.leftUpLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftUpLeg].y;
            bodyInfo.leftUpLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftUpLeg].z;
            bodyInfo.leftUpLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftUpLeg].w;

            // 初始化 leftLeg 成员
            bodyInfo.leftLeg.position.value[0] = m_mocapInputPosWorld[JointTag_LeftLeg].x;
            bodyInfo.leftLeg.position.value[1] = m_mocapInputPosWorld[JointTag_LeftLeg].y;
            bodyInfo.leftLeg.position.value[2] = m_mocapInputPosWorld[JointTag_LeftLeg].z;
            bodyInfo.leftLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftLeg].x;
            bodyInfo.leftLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftLeg].y;
            bodyInfo.leftLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftLeg].z;
            bodyInfo.leftLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftLeg].w;

            // 初始化 leftFoot 成员
            bodyInfo.leftFoot.position.value[0] = m_mocapInputPosWorld[JointTag_LeftFoot].x;
            bodyInfo.leftFoot.position.value[1] = m_mocapInputPosWorld[JointTag_LeftFoot].y;
            bodyInfo.leftFoot.position.value[2] = m_mocapInputPosWorld[JointTag_LeftFoot].z;
            bodyInfo.leftFoot.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftFoot].x;
            bodyInfo.leftFoot.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftFoot].y;
            bodyInfo.leftFoot.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftFoot].z;
            bodyInfo.leftFoot.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftFoot].w;

            // 初始化 rightUpLeg 成员
            bodyInfo.rightUpLeg.position.value[0] = m_mocapInputPosWorld[JointTag_RightUpLeg].x;
            bodyInfo.rightUpLeg.position.value[1] = m_mocapInputPosWorld[JointTag_RightUpLeg].y;
            bodyInfo.rightUpLeg.position.value[2] = m_mocapInputPosWorld[JointTag_RightUpLeg].z;
            bodyInfo.rightUpLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightUpLeg].x;
            bodyInfo.rightUpLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightUpLeg].y;
            bodyInfo.rightUpLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightUpLeg].z;
            bodyInfo.rightUpLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightUpLeg].w;

            // 初始化 rightLeg 成员
            bodyInfo.rightLeg.position.value[0] = m_mocapInputPosWorld[JointTag_RightLeg].x;
            bodyInfo.rightLeg.position.value[1] = m_mocapInputPosWorld[JointTag_RightLeg].y;
            bodyInfo.rightLeg.position.value[2] = m_mocapInputPosWorld[JointTag_RightLeg].z;
            bodyInfo.rightLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightLeg].x;
            bodyInfo.rightLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightLeg].y;
            bodyInfo.rightLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightLeg].z;
            bodyInfo.rightLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightLeg].w;

            // 初始化 rightFoot 成员
            bodyInfo.rightFoot.position.value[0] = m_mocapInputPosWorld[JointTag_RightFoot].x;
            bodyInfo.rightFoot.position.value[1] = m_mocapInputPosWorld[JointTag_RightFoot].y;
            bodyInfo.rightFoot.position.value[2] = m_mocapInputPosWorld[JointTag_RightFoot].z;
            bodyInfo.rightFoot.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightFoot].x;
            bodyInfo.rightFoot.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightFoot].y;
            bodyInfo.rightFoot.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightFoot].z;
            bodyInfo.rightFoot.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightFoot].w;

            fsc = CreateFscAlgorithm();
            fsc->Init(targetHipsWidth, targetHipsHeight, targetLegLength, targetAnkleHeight, bodyInfo);

        }
    }

    void Robot::solveVirtualBones()
    {
        if (m_config.lowerBodyMode == 1) // galbot
        {
            calcWheeledPosition();

            Vector3 pos = VInterpConstantTo(
                m_dummyNodes[DummyRoot]->GetPosition(),
                m_TargetPos,
                m_deltaTime,
                m_config.moveSpeed
            );

            m_dummyNodes[DummyRoot]->SetPosition(pos);

            Vector3 dir = VInterpNormalRotationTo(
                m_dummyNodes[DummyRoot]->GetDirX(),
                m_TargetDir,
                m_deltaTime,
                m_config.rotateSpeed
            );

            Quat rot = glm::angleAxis(std::atan2(-m_TargetDir.y, m_TargetDir.x), Vector3(0, 0, 1));
            m_dummyNodes[DummyRoot]->SetRotation(rot);

            m_rootPosition.x = m_dummyNodes[DummyRoot]->GetPosition().x;
            m_rootPosition.y = m_dummyNodes[DummyRoot]->GetPosition().y;
            m_rootPosition.z = m_dummyNodes[DummyRoot]->GetPosition().z;
            m_rootRotation.x = -m_dummyNodes[DummyRoot]->GetRotation().x;
            m_rootRotation.y = m_dummyNodes[DummyRoot]->GetRotation().y;
            m_rootRotation.z = -m_dummyNodes[DummyRoot]->GetRotation().z;
            m_rootRotation.w = m_dummyNodes[DummyRoot]->GetRotation().w;

            float hipsHeight = m_mocapInputPosWorld[JointTag_Hips].z;
            float alpha = (glm::min<float>(m_hipsHeightHighest, hipsHeight) - m_hipsHeightLowest) / (m_hipsHeightHighest - m_hipsHeightLowest);

            alpha = glm::clamp(alpha, 0.f, 0.98f); // 系数防止绷直

            auto kneePos = m_dummyKneeCtrl->GetLocalPosition();
            kneePos.z = m_robotLegLength1 * alpha;
            kneePos.z = glm::max<float>(kneePos.z, m_lowestKneeHeight);
            m_dummyKneeCtrl->SetLocalPosition(kneePos);

            m_centroidHorizontalOffset = m_initCentroidHorizontalOffset
                - m_config.centroidHorizontalOffset * (alpha - m_config.hipMin) / (m_config.hipMax - m_config.hipMin);

            Vector3 hipPos(0);
            hipPos.z = m_robotLegLength * alpha;
            hipPos.z = glm::max<float>(hipPos.z, m_lowestHipHeight);
            hipPos.x = m_centroidHorizontalOffset;
            m_dummyHipCtrl->SetLocalPosition(hipPos);

            // 更新三折IK
            IkTransform RootTransform;
            RootTransform.position = m_dummyNodes[DummyLeg1]->GetPosition();
            RootTransform.rotation = m_dummyNodes[DummyLeg1]->GetRotation();

            IkTransform JointTransform;
            JointTransform.position = m_dummyNodes[DummyLeg2]->GetPosition();
            JointTransform.rotation = m_dummyNodes[DummyLeg2]->GetRotation();

            IkTransform EndTransform;
            EndTransform.position = m_dummyNodes[DummySpine]->GetPosition();
            EndTransform.rotation = m_dummyNodes[DummySpine]->GetRotation();

            auto JointTarget = m_dummyKneeCtrl->GetPosition();
            auto Effector = m_dummyHipCtrl->GetPosition();

            bool bAllowStretching = false;
            float StartStretchRatio = 0.99f;
            float MaxStretchScale = 1.01f;
            TwoBonesIK(RootTransform, JointTransform, EndTransform, JointTarget, Effector, bAllowStretching, StartStretchRatio, MaxStretchScale);

            m_dummyNodes[DummyLeg1]->SetPosition(RootTransform.position);
            m_dummyNodes[DummyLeg1]->SetRotation(RootTransform.rotation);

            m_dummyNodes[DummyLeg2]->SetPosition(JointTransform.position);
            m_dummyNodes[DummyLeg2]->SetRotation(JointTransform.rotation);

            auto spine2RealRot = m_mocapInputRotWorld[JointTag_Spine2];
            auto spine2UpDir = spine2RealRot * Vector3(0, 0, 1);
            auto side = m_dummyNodes[DummySpine]->GetDirY();
            spine2UpDir = vecProjectOnPlane(spine2UpDir, side);
            float spine2Angle = angleBetween(Vector3(0, 0, 1), spine2UpDir, side); //QuatDistance(spine2RealRot, Quat(1, 0, 0, 0));

            if (spine2Angle > 0)
            {
                spine2Angle -= glm::radians(m_config.bendStartAngle);
                spine2Angle = glm::max<float>(0, spine2Angle);
            }
            else if (spine2Angle <= 0)
            {
                spine2Angle += glm::radians(m_config.bendStartAngle);
                spine2Angle = glm::min<float>(0, -spine2Angle);
            }

            spine2Angle *= m_config.bendScale;
            spine2Angle = glm::clamp(spine2Angle, -glm::radians(m_config.bendMaxAngle), glm::radians(m_config.bendMaxAngle));

            auto dummySpineQ = glm::angleAxis(spine2Angle, side);

            auto up = dummySpineQ * Vector3(0, 0, 1);
            auto spine2SideDir = spine2RealRot * Vector3(0, 1, 0);
            side = vecProjectOnPlane(spine2SideDir, up);
            dummySpineQ = quatFromYZ(side, up);

            m_dummyNodes[DummySpine]->SetRotation(Quat(dummySpineQ.w, -dummySpineQ.x, dummySpineQ.y, -dummySpineQ.z));

            // 补偿spine2
            //m_mocapInputRotWorld[JointTag_Spine2] = dummySpineQ;
        }
        else if (m_config.lowerBodyMode == 3) // zwwl
        {
            calcWheeledPosition();
            m_rootPosition.x = m_TargetPos.x;
            m_rootPosition.y = m_TargetPos.y;
            m_rootPosition.z = 0;

            Quat rot = glm::angleAxis(std::atan2(m_TargetDir.y, m_TargetDir.x) - glm::pi<float>() / 2, Vector3(0, 0, 1));
            m_rootRotation = rot;

            // 身体高度控制垂直滑轨，踮脚和下蹲
            if (m_mocapInputPosWorld[JointTag_Hips].z / m_standHeight >= m_config.slideThresholdMax)
            {
                m_slideSpeed = m_config.slideSpeed;
            }
            else if (m_mocapInputPosWorld[JointTag_Hips].z / m_standHeight <= m_config.slideThresholdMin)
            {
                m_slideSpeed = -m_config.slideSpeed;
            }
            else
            {
                m_slideSpeed = 0.0f;
            }

            m_slideHeight = glm::clamp(m_slideHeight + m_slideSpeed * m_deltaTime,
                m_config.slideHeightMin, m_config.slideHeightMax);
        }
    }

    struct CurveFitParams
    {
        // 是否启动qcf
        bool enableQcf = true;
        // 用于拟合的点数
        int curveLen = 40;
        // n次拟合
        int fitDegree = 7;
        // n帧平滑
        int smoothDegree = 7;
        // 二次微分变化量大于此值时，将tracking state值为未追踪
        float angleLimit = 3.0;
    } qcfParams;

    // n次多项式拟合函数：y = at^3 + bt^2 + ct + d
    void fitCurve(std::deque<float>& data, const float& x, float& result)
    {
        // 保存到输出
        result = x;

        // 队列：缓存数据
        if (data.size() > 0) data.pop_front();
        do data.push_back(x); while (data.size() < qcfParams.curveLen);

        int N = qcfParams.curveLen;
        Eigen::MatrixXd A(N, qcfParams.fitDegree + 1);
        Eigen::VectorXd bx(N);

        // 构建 A 矩阵和 b 向量
        for (int i = 0; i < N; ++i)
        {
            double t = i + 1;  // 时间变量
            for (int j = 0; j <= qcfParams.fitDegree; ++j)
            {
                A(i, j) = std::pow(t, qcfParams.fitDegree - j);
            }
            bx(i) = data[i];
        }

        // 计算最小二乘拟合系数
        Eigen::VectorXd coeffs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bx);

#if 0
        // 计算当前帧的拟合值（平滑值）
        double t_last = N + 1;
        double xx = 0;
        for (int j = 0; j <= qcfParams.fitDegree; ++j)
        {
            xx += coeffs(j) * std::pow(t_last, qcfParams.fitDegree - j);
        }
#endif

        // 使用拟合系数对原始数据进行曲线预测
        std::vector<double> predicted;
        double t = 0;
        double y = 0;
        for (size_t i = 0; i < data.size(); ++i)
        {
            t = i + 1;
            y = 0;
            for (int j = 0; j <= qcfParams.fitDegree; ++j)
            {
                y += coeffs(j) * std::pow(t, qcfParams.fitDegree - j);
            }
            predicted.push_back(y);
        }

        // 输出（取平滑曲线上的倒数第2帧）
        //int valuePos = std::max(qcfParams.curveLen / 2, qcfParams.curveLen - 2);
        int valuePos = int(qcfParams.curveLen / 2);
        result = predicted[valuePos];
    }

    void Robot::postprocess()
    {
        if (m_config.lowerBodyMode == 1)
        {
            auto l1 = m_dummyNodes[DummyLeg1]->GetLocalEulerAngles().y;
            auto l2 = m_dummyNodes[DummyLeg2]->GetLocalEulerAngles().y;
            auto l3 = m_dummyNodes[DummySpine]->GetLocalEulerAngles().y;
            auto l4 = m_dummyNodes[DummySpine]->GetLocalEulerAngles().z;

            SetRawJointAngle(JointRightAnkle_Pitch, glm::radians(l1));
            SetRawJointAngle(JointRightKnee, glm::radians(l2));
            SetRawJointAngle(JointRightHip_Pitch, glm::radians(l3));
            SetRawJointAngle(JointRightHip_Yaw, glm::radians(l4));
        }
        else if (m_config.lowerBodyMode == 4) // HIT
        {
            calcWheeledPosition();
            m_rootPosition.x = 0;//m_TargetPos.x;
            m_rootPosition.y = 0;//m_TargetPos.y;
            m_rootPosition.z = 0;

            Quat rot = glm::angleAxis(std::atan2(m_TargetDir.y, m_TargetDir.x) - glm::pi<float>() / 2, Vector3(0, 0, 1));
            m_rootRotation = Quat(1, 0, 0, 0);//rot;

            float heightMin = m_standHeight * m_config.hipMin;
            float heightMax = m_standHeight * m_config.hipMax;
            float height = glm::clamp<float>(m_mocapInputPosWorld[JointTag_Hips].z, heightMin, heightMax);
            float leg_0_min = 0;
            constexpr float leg_0_max = glm::pi<float>() / 2.0f;
            float leg_0 = glm::mix<float>(leg_0_min, leg_0_max, 1.0f - (height - heightMin) / (heightMax - heightMin));

            SetRawJointAngle(JointLeftAnkle_Pitch, leg_0);
            SetRawJointAngle(JointLeftKnee, leg_0 * 2);
            SetRawJointAngle(JointRightAnkle_Pitch, leg_0);
            SetRawJointAngle(JointRightKnee, leg_0 * 2);

            auto spine2RealRot = m_mocapInputRotWorld[JointTag_Spine2];
            auto spine2UpDir = spine2RealRot * Vector3(0, 0, 1);
            auto spine2Side = spine2RealRot * Vector3(0, 1, 0);
            spine2UpDir = vecProjectOnPlane(spine2UpDir, spine2Side);
            float spine2Angle = angleBetween(Vector3(0, 0, 1), spine2UpDir, spine2Side);

            SetRawJointAngle(JointSpine_Pitch, spine2Angle - leg_0);
        }

        // remapping
        for (const auto& retarget : m_config.retargetJoints)
        {
            float oldValue = m_retargetJointsAngle[retarget.retargetJoint];

            float rawValue = GetRawJointAngle((RobotJointType)retarget.rawJoint);
            //assert(!glm::isnan(rawValue));

            float newValue = Clamp180(rawValue * retarget.sign + glm::radians(retarget.offset));
            if (m_config.useAngleLimit && retarget.useAngleLimit)
                newValue = glm::clamp<float>(newValue, retarget.min, retarget.max);

            float maxSpeed = retarget.maxSpeed;
            float dist = std::fabs(newValue - oldValue);
            if (m_config.useSpeedLimit && retarget.useSpeedLimit)
            {
                if (dist > maxSpeed * m_deltaTime)
                {
                    if (newValue > oldValue)
                        newValue = oldValue + maxSpeed * m_deltaTime;
                    else
                        newValue = oldValue - maxSpeed * m_deltaTime;
                }
            }
            m_retargetJointsAngle[retarget.retargetJoint] = newValue;

            // 角度平滑处理
            if (m_config.dataSmooth)
                fitCurve(m_smoothCacheAngleData[retarget.retargetJoint],
                    m_retargetJointsAngle[retarget.retargetJoint],
                    m_retargetJointsAngleSmooth[retarget.retargetJoint]);

            // 速度
            if (m_config.dataSmooth) {
                // 直接差分，不太理想
                m_retargetJointsAngleSpeedSmooth[retarget.retargetJoint] = (m_retargetJointsAngleSmooth[retarget.retargetJoint] - m_retargetJointsAngleLastFrameSmooth[retarget.retargetJoint]) / m_deltaTime;
                // 二次曲线拟合
                //fitCurve(m_smoothCacheSpeedData[retarget.retargetJoint],
                //    m_retargetJointsAngleSpeed[retarget.retargetJoint],
                //    m_retargetJointsAngleSpeedSmooth[retarget.retargetJoint]);
            }

            // 速度（非平滑）
            m_retargetJointsAngleSpeed[retarget.retargetJoint] = (m_retargetJointsAngle[retarget.retargetJoint] - m_retargetJointsAngleLastFrame[retarget.retargetJoint]) / m_deltaTime;

            // last frame smooth angle
            if (m_config.dataSmooth) {
                m_retargetJointsAngleLastFrameSmooth[retarget.retargetJoint] = m_retargetJointsAngleSmooth[retarget.retargetJoint];
            }
            // non-smooth angle
            m_retargetJointsAngleLastFrame[retarget.retargetJoint] = m_retargetJointsAngle[retarget.retargetJoint];
        }

        if (m_config.lowerBodyMode == 2) // bytedance
        {

            m_rawJointsAngle[RobotJointType::JointRightKnee] -= m_rawJointsAngle[RobotJointType::JointRightAnkle_Pitch];
            m_rawJointsAngle[RobotJointType::JointLeftKnee] -= m_rawJointsAngle[RobotJointType::JointLeftAnkle_Pitch];
        }
    }

    void Robot::InfoDisp()
    {
        //targetHipsWidth= glm::length(m_mocapInputPosWorld[JointTag_LeftUpLeg] - m_mocapInputPosWorld[JointTag_RightUpLeg]);
        //targetAnkleHeight = m_mocapInputRotWorld[JointTag_RightFoot].z;
        //targetLegLength = m_mocapInputRotWorld[JointTag_Hips].z - targetAnkleHeight;

        //targetAnkleHeight = 0.02f;
        //targetLegLength = 0.8f;
        //sourceHipsWidth = 0.25f;
        //sourceLegLength = 0.6f;
        //sourceAnkleHeight = 0.01f;
        //leftFootOffsetDir = Vector3(1, 0, 0);
        //rightFootOffsetDir = Vector3(1, 0, 0);
        //leftKneeOffsetDir = Vector3(1, 0, 0);
        //rightKneeOffsetDir = Vector3(1, 0, 0);
    }

    void Robot::SetUpdateWithDisp()
    {
        
        //机器人最终髋高， 真人髋高*（机器人髋部长度+机器人大腿长+机器人小腿长）/（真人髋长+真人大腿长+真人小腿长）

         //在此处调整机器人大小
        bodyInfo.hips.position.value[0] = m_mocapInputPosWorld[JointTag_Hips].x;
        bodyInfo.hips.position.value[1] = m_mocapInputPosWorld[JointTag_Hips].y;
        bodyInfo.hips.position.value[2] = m_mocapInputPosWorld[JointTag_Hips].z;
        bodyInfo.hips.rotation.value[0] = m_mocapInputRotWorld[JointTag_Hips].x;
        bodyInfo.hips.rotation.value[1] = m_mocapInputRotWorld[JointTag_Hips].y;
        bodyInfo.hips.rotation.value[2] = m_mocapInputRotWorld[JointTag_Hips].z;
        bodyInfo.hips.rotation.value[3] = m_mocapInputRotWorld[JointTag_Hips].w;
        // 初始化 leftUpLeg 成员
        bodyInfo.leftUpLeg.position.value[0] = m_mocapInputPosWorld[JointTag_LeftUpLeg].x;
        bodyInfo.leftUpLeg.position.value[1] = m_mocapInputPosWorld[JointTag_LeftUpLeg].y;
        bodyInfo.leftUpLeg.position.value[2] = m_mocapInputPosWorld[JointTag_LeftUpLeg].z;
        bodyInfo.leftUpLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftUpLeg].x;
        bodyInfo.leftUpLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftUpLeg].y;
        bodyInfo.leftUpLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftUpLeg].z;
        bodyInfo.leftUpLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftUpLeg].w;

        // 初始化 leftLeg 成员
        bodyInfo.leftLeg.position.value[0] = m_mocapInputPosWorld[JointTag_LeftLeg].x;
        bodyInfo.leftLeg.position.value[1] = m_mocapInputPosWorld[JointTag_LeftLeg].y;
        bodyInfo.leftLeg.position.value[2] = m_mocapInputPosWorld[JointTag_LeftLeg].z;
        bodyInfo.leftLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftLeg].x;
        bodyInfo.leftLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftLeg].y;
        bodyInfo.leftLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftLeg].z;
        bodyInfo.leftLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftLeg].w;

        // 初始化 leftFoot 成员
        bodyInfo.leftFoot.position.value[0] = m_mocapInputPosWorld[JointTag_LeftFoot].x;
        bodyInfo.leftFoot.position.value[1] = m_mocapInputPosWorld[JointTag_LeftFoot].y;
        bodyInfo.leftFoot.position.value[2] = m_mocapInputPosWorld[JointTag_LeftFoot].z;
        bodyInfo.leftFoot.rotation.value[0] = m_mocapInputRotWorld[JointTag_LeftFoot].x;
        bodyInfo.leftFoot.rotation.value[1] = m_mocapInputRotWorld[JointTag_LeftFoot].y;
        bodyInfo.leftFoot.rotation.value[2] = m_mocapInputRotWorld[JointTag_LeftFoot].z;
        bodyInfo.leftFoot.rotation.value[3] = m_mocapInputRotWorld[JointTag_LeftFoot].w;

        // 初始化 rightUpLeg 成员
        bodyInfo.rightUpLeg.position.value[0] = m_mocapInputPosWorld[JointTag_RightUpLeg].x;
        bodyInfo.rightUpLeg.position.value[1] = m_mocapInputPosWorld[JointTag_RightUpLeg].y;
        bodyInfo.rightUpLeg.position.value[2] = m_mocapInputPosWorld[JointTag_RightUpLeg].z;
        bodyInfo.rightUpLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightUpLeg].x;
        bodyInfo.rightUpLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightUpLeg].y;
        bodyInfo.rightUpLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightUpLeg].z;
        bodyInfo.rightUpLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightUpLeg].w;

        // 初始化 rightLeg 成员
        bodyInfo.rightLeg.position.value[0] = m_mocapInputPosWorld[JointTag_RightLeg].x;
        bodyInfo.rightLeg.position.value[1] = m_mocapInputPosWorld[JointTag_RightLeg].y;
        bodyInfo.rightLeg.position.value[2] = m_mocapInputPosWorld[JointTag_RightLeg].z;
        bodyInfo.rightLeg.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightLeg].x;
        bodyInfo.rightLeg.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightLeg].y;
        bodyInfo.rightLeg.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightLeg].z;
        bodyInfo.rightLeg.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightLeg].w;

        // 初始化 rightFoot 成员
        bodyInfo.rightFoot.position.value[0] = m_mocapInputPosWorld[JointTag_RightFoot].x;
        bodyInfo.rightFoot.position.value[1] = m_mocapInputPosWorld[JointTag_RightFoot].y;
        bodyInfo.rightFoot.position.value[2] = m_mocapInputPosWorld[JointTag_RightFoot].z;
        bodyInfo.rightFoot.rotation.value[0] = m_mocapInputRotWorld[JointTag_RightFoot].x;
        bodyInfo.rightFoot.rotation.value[1] = m_mocapInputRotWorld[JointTag_RightFoot].y;
        bodyInfo.rightFoot.rotation.value[2] = m_mocapInputRotWorld[JointTag_RightFoot].z;
        bodyInfo.rightFoot.rotation.value[3] = m_mocapInputRotWorld[JointTag_RightFoot].w;

        FscLowerBodyInfo bodyDisp;
        fsc->UpdateWithDisp(60,bodyInfo,bodyDisp);

        m_mocapInputPosWorld[JointTag_Hips].x = bodyDisp.hips.position.value[0];
        m_mocapInputPosWorld[JointTag_Hips].y = bodyDisp.hips.position.value[1];
        m_mocapInputPosWorld[JointTag_Hips].z = bodyDisp.hips.position.value[2];
        m_mocapInputRotWorld[JointTag_Hips].x = bodyDisp.hips.rotation.value[0];
        m_mocapInputRotWorld[JointTag_Hips].y = bodyDisp.hips.rotation.value[1];
        m_mocapInputRotWorld[JointTag_Hips].z = bodyDisp.hips.rotation.value[2];
        m_mocapInputRotWorld[JointTag_Hips].w = bodyDisp.hips.rotation.value[3];
        //hips既是世界系，也是本体系
        m_rootPosition = m_mocapInputPosWorld[JointTag_Hips];
        m_rootRotation = m_mocapInputRotWorld[JointTag_Hips];
        m_mocapInputRot[JointTag_Hips] = m_mocapInputRotWorld[JointTag_Hips];
        m_mocapInputPos[JointTag_Hips] = m_mocapInputPosWorld[JointTag_Hips];

        //左半身
        //// 反向赋值 LeftUpLeg 关节信息
        m_mocapInputPosWorld[JointTag_LeftUpLeg].x = bodyDisp.leftUpLeg.position.value[0];
        m_mocapInputPosWorld[JointTag_LeftUpLeg].y = bodyDisp.leftUpLeg.position.value[1];
        m_mocapInputPosWorld[JointTag_LeftUpLeg].z = bodyDisp.leftUpLeg.position.value[2];
        m_mocapInputRotWorld[JointTag_LeftUpLeg].x = bodyDisp.leftUpLeg.rotation.value[0];
        m_mocapInputRotWorld[JointTag_LeftUpLeg].y = bodyDisp.leftUpLeg.rotation.value[1];
        m_mocapInputRotWorld[JointTag_LeftUpLeg].z = bodyDisp.leftUpLeg.rotation.value[2];
        m_mocapInputRotWorld[JointTag_LeftUpLeg].w = bodyDisp.leftUpLeg.rotation.value[3];
        //本体系旋转=父物体旋转的逆*世界系旋转
        Quat leftUplegOffset = glm::inverse(m_mocapInputRot[JointTag_Hips]);
        m_mocapInputRot[JointTag_LeftUpLeg] = leftUplegOffset * m_mocapInputRotWorld[JointTag_LeftUpLeg] ;
        //本体系坐标偏移
        eba::Vector3 leftUpLegInfoPos = Vector3(bodyInfo.leftUpLeg.position.value[0], bodyInfo.leftUpLeg.position.value[1], bodyInfo.leftUpLeg.position.value[2]);
        eba::Vector3 leftUpLegLocalPosOffset =RotateVectorManually (leftUplegOffset, m_mocapInputPos[JointTag_LeftUpLeg]) + m_mocapInputPosWorld[JointTag_LeftUpLeg] - leftUpLegInfoPos;
        m_mocapInputPos[JointTag_LeftUpLeg] += leftUpLegLocalPosOffset;
        //

        // 反向赋值 LeftLeg 关节信息
        m_mocapInputPosWorld[JointTag_LeftLeg].x = bodyDisp.leftLeg.position.value[0];
        m_mocapInputPosWorld[JointTag_LeftLeg].y = bodyDisp.leftLeg.position.value[1];
        m_mocapInputPosWorld[JointTag_LeftLeg].z = bodyDisp.leftLeg.position.value[2];
        m_mocapInputRotWorld[JointTag_LeftLeg].x = bodyDisp.leftLeg.rotation.value[0];
        m_mocapInputRotWorld[JointTag_LeftLeg].y = bodyDisp.leftLeg.rotation.value[1];
        m_mocapInputRotWorld[JointTag_LeftLeg].z = bodyDisp.leftLeg.rotation.value[2];
        m_mocapInputRotWorld[JointTag_LeftLeg].w = bodyDisp.leftLeg.rotation.value[3];

        //本体旋转
        Quat leftLegOffset = glm::inverse( m_mocapInputRot[JointTag_Hips]* m_mocapInputRot[JointTag_LeftUpLeg] );
        m_mocapInputRot[JointTag_LeftLeg] = leftLegOffset * m_mocapInputRotWorld[JointTag_LeftLeg] ;

        eba::Vector3 leftLegInfoPos = Vector3(bodyInfo.leftLeg.position.value[0], bodyInfo.leftLeg.position.value[1], bodyInfo.leftLeg.position.value[2]);
        eba::Vector3 leftLegLocalPosOffset = RotateVectorManually(leftLegOffset, m_mocapInputPos[JointTag_LeftLeg]) + m_mocapInputPosWorld[JointTag_LeftLeg] - leftLegInfoPos;
        m_mocapInputPos[JointTag_LeftLeg] += leftLegLocalPosOffset;

        // 反向赋值 LeftFoot 关节信息
        m_mocapInputPosWorld[JointTag_LeftFoot].x = bodyDisp.leftFoot.position.value[0];
        m_mocapInputPosWorld[JointTag_LeftFoot].y = bodyDisp.leftFoot.position.value[1];
        m_mocapInputPosWorld[JointTag_LeftFoot].z = bodyDisp.leftFoot.position.value[2];
        m_mocapInputRotWorld[JointTag_LeftFoot].x = bodyDisp.leftFoot.rotation.value[0];
        m_mocapInputRotWorld[JointTag_LeftFoot].y = bodyDisp.leftFoot.rotation.value[1];
        m_mocapInputRotWorld[JointTag_LeftFoot].z = bodyDisp.leftFoot.rotation.value[2];
        m_mocapInputRotWorld[JointTag_LeftFoot].w = bodyDisp.leftFoot.rotation.value[3];

        //本体旋转
        Quat leftFootOffset= glm::inverse( m_mocapInputRot[JointTag_Hips]* m_mocapInputRot[JointTag_LeftUpLeg] * m_mocapInputRot[JointTag_LeftLeg] );
        m_mocapInputRot[JointTag_LeftFoot] = leftFootOffset * m_mocapInputRotWorld[JointTag_LeftFoot] ;
        //本体系坐标通过角度旋转偏移和移动偏移相加
        eba::Vector3 leftFootInfoPos = Vector3(bodyInfo.leftFoot.position.value[0], bodyInfo.leftFoot.position.value[1], bodyInfo.leftFoot.position.value[2]);
        eba::Vector3 leftUpLegPosOffset =RotateVectorManually(leftFootOffset, m_mocapInputPos[JointTag_LeftFoot])+ m_mocapInputPosWorld[JointTag_LeftFoot] - leftFootInfoPos;
        m_mocapInputPos[JointTag_LeftFoot] += leftUpLegPosOffset;

        //右下半身
        // 反向赋值 RightUpLeg 关节信息
        m_mocapInputPosWorld[JointTag_RightUpLeg].x = bodyDisp.rightUpLeg.position.value[0];
        m_mocapInputPosWorld[JointTag_RightUpLeg].y = bodyDisp.rightUpLeg.position.value[1];
        m_mocapInputPosWorld[JointTag_RightUpLeg].z = bodyDisp.rightUpLeg.position.value[2];
        m_mocapInputRotWorld[JointTag_RightUpLeg].x = bodyDisp.rightUpLeg.rotation.value[0];
        m_mocapInputRotWorld[JointTag_RightUpLeg].y = bodyDisp.rightUpLeg.rotation.value[1];
        m_mocapInputRotWorld[JointTag_RightUpLeg].z = bodyDisp.rightUpLeg.rotation.value[2];
        m_mocapInputRotWorld[JointTag_RightUpLeg].w = bodyDisp.rightUpLeg.rotation.value[3];
        //本体系旋转
        eba::Quat rightUpLegLocalRotOffset = glm::inverse(m_mocapInputRot[JointTag_Hips]);
        m_mocapInputRot[JointTag_RightUpLeg] = rightUpLegLocalRotOffset* m_mocapInputRotWorld[JointTag_RightUpLeg];

        eba::Vector3 rightUpLegInfoPos = Vector3(bodyInfo.rightUpLeg.position.value[0], bodyInfo.rightUpLeg.position.value[1], bodyInfo.rightUpLeg.position.value[2]);
        eba::Vector3 rightUpLegLocalPosOffset =RotateVectorManually(rightUpLegLocalRotOffset, m_mocapInputPos[JointTag_RightUpLeg]) + m_mocapInputPosWorld[JointTag_RightUpLeg] - rightUpLegInfoPos;
        m_mocapInputPos[JointTag_RightUpLeg] += rightUpLegLocalPosOffset;


        // 反向赋值 RightLeg 关节信息
        m_mocapInputPosWorld[JointTag_RightLeg].x = bodyDisp.rightLeg.position.value[0];
        m_mocapInputPosWorld[JointTag_RightLeg].y = bodyDisp.rightLeg.position.value[1];
        m_mocapInputPosWorld[JointTag_RightLeg].z = bodyDisp.rightLeg.position.value[2];
        m_mocapInputRotWorld[JointTag_RightLeg].x = bodyDisp.rightLeg.rotation.value[0];
        m_mocapInputRotWorld[JointTag_RightLeg].y = bodyDisp.rightLeg.rotation.value[1];
        m_mocapInputRotWorld[JointTag_RightLeg].z = bodyDisp.rightLeg.rotation.value[2];
        m_mocapInputRotWorld[JointTag_RightLeg].w = bodyDisp.rightLeg.rotation.value[3];
        //本体系旋转
        eba::Quat rightLegLocalRotOffset = glm::inverse(m_mocapInputRot[JointTag_Hips]*m_mocapInputRot[JointTag_RightUpLeg]);
        m_mocapInputRot[JointTag_RightLeg] = rightLegLocalRotOffset* m_mocapInputRotWorld[JointTag_RightLeg];

        eba::Vector3 rightLegInfoPos = Vector3(bodyInfo.rightLeg.position.value[0], bodyInfo.rightLeg.position.value[1], bodyInfo.rightLeg.position.value[2]);
        eba::Vector3 rightLegocalPosOffset = RotateVectorManually (rightLegLocalRotOffset, m_mocapInputPos[JointTag_RightLeg]) + m_mocapInputPosWorld[JointTag_RightLeg] - rightLegInfoPos;
        m_mocapInputPos[JointTag_RightLeg] += rightLegocalPosOffset;


        // 反向赋值 RightFoot 关节信息
        m_mocapInputPosWorld[JointTag_RightFoot].x = bodyDisp.rightFoot.position.value[0];
        m_mocapInputPosWorld[JointTag_RightFoot].y = bodyDisp.rightFoot.position.value[1];
        m_mocapInputPosWorld[JointTag_RightFoot].z = bodyDisp.rightFoot.position.value[2];
        m_mocapInputRotWorld[JointTag_RightFoot].x = bodyDisp.rightFoot.rotation.value[0];
        m_mocapInputRotWorld[JointTag_RightFoot].y = bodyDisp.rightFoot.rotation.value[1];
        m_mocapInputRotWorld[JointTag_RightFoot].z = bodyDisp.rightFoot.rotation.value[2];
        m_mocapInputRotWorld[JointTag_RightFoot].w = bodyDisp.rightFoot.rotation.value[3];
        //本体系旋转
        eba::Quat rightFootLocalRotOffset = glm::inverse(m_mocapInputRot[JointTag_Hips]*m_mocapInputRot[JointTag_RightUpLeg] *m_mocapInputRot[JointTag_RightLeg]);
        m_mocapInputRot[JointTag_RightFoot] = rightFootLocalRotOffset* m_mocapInputRotWorld[JointTag_RightFoot];
        //本体坐标偏移
        eba::Vector3 rightFootInfoPos = Vector3(bodyInfo.rightFoot.position.value[0], bodyInfo.rightFoot.position.value[1], bodyInfo.rightFoot.position.value[2]);
        eba::Vector3 rightFootLocalPosOffset =RotateVectorManually(rightFootLocalRotOffset, m_mocapInputPos[JointTag_RightFoot]) + m_mocapInputPosWorld[JointTag_RightFoot] - rightFootInfoPos;
        m_mocapInputPos[JointTag_RightFoot] += rightFootLocalPosOffset;


    }

    Vector3 Robot::RotateVectorManually(const eba::Quat& q, const eba::Vector3& v)
    {
        eba::Quat vecQuat(0, v.x, v.y, v.z);
        eba::Quat rotatedQuat = q * vecQuat * glm::inverse( q); // q * v * q^-1
        return eba::Vector3(rotatedQuat.x, rotatedQuat.y, rotatedQuat.z);
    }


    void Robot::solveDeltaTime()
    {
        auto nowtime = std::chrono::steady_clock::now();
        m_deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(nowtime - m_lastTickTime).count() * 0.001f * 0.001f; // 单位秒，精度毫秒(用微秒算）
        m_lastTickTime = nowtime;
    }

    MocapApi::EMCPError Robot::GetRawJointAngleInternal(RobotJointType jointType, float* pvalue) const
    {
        assert(pvalue);
        auto iter = m_rawJointsAngle.find(jointType);
        if (iter != m_rawJointsAngle.end())
        {
            float angle = iter->second;
            *pvalue = angle;
        }
        else
        {
            *pvalue = 0;
        }
        return MocapApi::EMCPError::Error_None;
    }

    void Robot::Unload()
    {
        m_rawJointsAngle.clear();
        m_retargetJointsAngle.clear();
        m_config.retargetJoints.clear();
    }

    void Robot::SetFPS(int fps)
    {
        m_fps = fps;
    }

    MocapApi::EMCPError Robot::GetRawJointAngle(RobotJointType jointType, float* value) const
    {
        auto ret = GetRawJointAngleInternal(jointType, value);
        *value = Angle360To180(*value);
        return ret;
    }

    float Robot::GetRawJointAngle(RobotJointType jointType) const
    {
        float value = 0;
        GetRawJointAngle(jointType, &value);
        return value;
    }

    void Robot::SetRawJointAngle(RobotJointType jointType, float value)
    {
        //assert(!glm::isnan(value));

        // 消除绕360
        value = Angle360To180(value);
        m_rawJointsAngle[jointType] = value;
    }

    MocapApi::EMCPError Robot::GetRetargetJointAngle(const std::string& jointName, float* value) const
    {
        if (m_config.dataSmooth)
        {
            auto iter = m_retargetJointsAngleSmooth.find(jointName);
            if (iter != m_retargetJointsAngleSmooth.end())
            {
                *value = iter->second;
                return MocapApi::EMCPError::Error_None;
            }
            else
            {
                return MocapApi::EMCPError::Error_InvalidParameter;
            }
        }
        else
        {
            auto iter = m_retargetJointsAngle.find(jointName);
            if (iter != m_retargetJointsAngle.end())
            {
                *value = iter->second;
                return MocapApi::EMCPError::Error_None;
            }
            else
            {
                return MocapApi::EMCPError::Error_InvalidParameter;
            }
        }
    }

    float Robot::GetRetargetJointAngle(const std::string& jointName) const
    {
        float value = 0;
        GetRetargetJointAngle(jointName, &value);
        return value;
    }

    MocapApi::EMCPError Robot::GetRetargetJointAngleVelocity(const std::string& jointName, float* value) const
    {
        if (m_config.dataSmooth)
        {
            auto iter = m_retargetJointsAngleSpeedSmooth.find(jointName);
            if (iter != m_retargetJointsAngleSpeedSmooth.end())
            {
                *value = iter->second;
                return MocapApi::EMCPError::Error_None;
            }
            else
            {
                return MocapApi::EMCPError::Error_InvalidParameter;
            }
        }
        else
        {
            auto iter = m_retargetJointsAngleSpeed.find(jointName);
            if (iter != m_retargetJointsAngleSpeed.end())
            {
                *value = iter->second;
                return MocapApi::EMCPError::Error_None;
            }
            else
            {
                return MocapApi::EMCPError::Error_InvalidParameter;
            }
        }
    }

    const std::map<std::string, float>& Robot::GetRetargetJointsAngle() const
    {
        if (m_config.dataSmooth)
        {
            return m_retargetJointsAngleSmooth;
        }
        else
        {
            return m_retargetJointsAngle;
        }
    }

    void Robot::calcWheeledPosition()
    {
        Vector3 leftFootDir = m_mocapInputRotWorld[JointTag_LeftFoot] * Vector3(1, 0, 0);
        Vector3 rightFootDir = m_mocapInputRotWorld[JointTag_RightFoot] * Vector3(1, 0, 0);
        leftFootDir.z = 0;
        rightFootDir.z = 0;
        leftFootDir = glm::normalize(leftFootDir);
        rightFootDir = glm::normalize(rightFootDir);
        m_TargetDir = glm::normalize(leftFootDir + rightFootDir);

        m_TargetPos = (m_mocapInputPosWorld[JointTag_LeftFoot] + m_mocapInputPosWorld[JointTag_RightFoot]) * 0.5f;
        m_TargetPos.z = 0;
    }

    void Robot::GetRosFrameJson(std::string& jsonStr, bool compress) const
    {
        // record
        using json = nlohmann::ordered_json;

        json j;
        j["sn"] = "xxxx";
        j["timestamp"][0] = m_timestempS;
        j["timestamp"][1] = m_timestemp1;

        j["delta_time"] = m_deltaTime;
        j["root_pos_x"] = m_rootPosition.x;
        j["root_pos_y"] = m_rootPosition.y;
        j["root_pos_z"] = m_rootPosition.z;
        j["root_rot_x"] = -m_rootRotation.x; // 转到ROS坐标系
        j["root_rot_y"] = m_rootRotation.y;
        j["root_rot_z"] = -m_rootRotation.z;
        j["root_rot_w"] = m_rootRotation.w;

        auto& positions = j["joint_positions"];
        const auto& angleResult = GetRetargetJointsAngle();
        for (const auto& jName : m_config.urdfJointNames)
        {
            auto iter = angleResult.find(jName);
            if (iter != angleResult.end())
            {
                positions[jName] = iter->second;// - joints[jName];
            }
        }

        if (m_config.dataSmooth) {
            auto& positions_raw = j["joint_positions_raw"];
            for (const auto& jName : m_config.urdfJointNames)
            {
                auto iter = angleResult.find(jName);
                if (iter != angleResult.end())
                {
                    positions_raw[jName] = m_retargetJointsAngle.at(jName);
                }
            }

            auto& speed = j["joint_speed"];
            for (const auto& jName : m_config.urdfJointNames)
            {
                auto iter = angleResult.find(jName);
                if (iter != angleResult.end())
                {
                    speed[jName] = m_retargetJointsAngleSpeedSmooth.at(jName);
                }
            }

            auto& speed_raw = j["joint_speed_raw"];
            for (const auto& jName : m_config.urdfJointNames)
            {
                auto iter = angleResult.find(jName);
                if (iter != angleResult.end())
                {
                    speed_raw[jName] = m_retargetJointsAngleSpeed.at(jName);
                }
            }
        }

        auto& poses = j["link_poses"];
        for (int i = 0; i < 59; ++i)
        {
            const auto& pos = m_mocapInputRot[i];
            const auto& rot = m_mocapInputPos[i];
            auto jointName = NoitomBoneNames[i];
            poses[jointName][0] = m_mocapInputPos[i].x;
            poses[jointName][1] = m_mocapInputPos[i].y;
            poses[jointName][2] = m_mocapInputPos[i].z;
            poses[jointName][3] = -m_mocapInputRot[i].x; // 转到ROS坐标系
            poses[jointName][4] = m_mocapInputRot[i].y;
            poses[jointName][5] = -m_mocapInputRot[i].z;
            poses[jointName][6] = m_mocapInputRot[i].w;
        }

        if (compress)
            jsonStr = j.dump();
        else
            jsonStr = j.dump(4);
    }


    void Robot::GetRootRotation(float* x, float* y, float* z, float* w) const
    {
        // 转换到机器人坐标系
        *x = -m_rootRotation.x; // 转到ROS坐标系
        *y = m_rootRotation.y;
        *z = -m_rootRotation.z;
        *w = m_rootRotation.w;
    }

    void Robot::GetRootPosition(float* x, float* y, float* z) const
    {
        // 转换到机器人坐标系
        *x = m_rootPosition.x;
        *y = m_rootPosition.y;
        *z = m_rootPosition.z;
    }


    MocapApi::EMCPError Robot::GetSlideSpeed(float* value) const
    {
        *value = m_slideSpeed;
        return MocapApi::EMCPError::Error_None;
    }

    MocapApi::EMCPError Robot::GetSlideHeight(float* value) const
    {
        *value = m_slideHeight;
        return MocapApi::EMCPError::Error_None;
    }

} // namespace n_robot