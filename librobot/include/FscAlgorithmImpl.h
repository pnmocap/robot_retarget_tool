#pragma once
#include "MotionAdaptorLib.h"
#include"mathutils.h"
#include"string"
#include"map"
#include"jointnode.h"
namespace eba
{

	/**
	 * FscAlgorithmImpl
	 */
	class FscAlgorithmImpl : public FscAlgorithm
	{
	public:
		FscAlgorithmImpl();
		virtual ~FscAlgorithmImpl();

		void Init(float robotHipsWidth, float robotHipsHeight, float robotLegLength, float robotAnkleHeight,const FscLowerBodyInfo& initPose);

        void UpdateWithDisp(int frame, const FscLowerBodyInfo& input, FscLowerBodyInfo& output) override;

        static void DrawSphere(const Vector3& pos, const Vector4& color);
		static Quat fromVectors(const glm::vec3& from, const glm::vec3& to);
	protected:
		JointNode* CreateBone(int index, const std::string& name, const std::string& parentName, const FscBoneInfo& boneInfo);

    protected:
		//输入源参数
		float sourceHipsWidth = 0.238f;
		float sourceHipsHeight = 0.0f;
        float sourceLegLength = 0.858f;
		float sourceAnkleHeight = 0.08f;
		//机器人实际参数
		float targetHipsWidth = 0.21f;
		float targetHipsHeight = 0.0f;
		float targetLegLength = 0.73f;
		float targetAnkleHeight = 0.05f;

		Vector3 modelForward;
		Vector3 modelLeft;

		Vector3 leftFootOffsetDir;
		Vector3 rightFootOffsetDir;
		Vector3 leftKneeOffsetDir;
		Vector3 rightKneeOffsetDir;

		JointNode* leftFootEffector;
		JointNode* rightFootEffector;

		std::map<std::string,JointNode*> boneMap;

		JointNode* hips = nullptr;
		JointNode* leftUpLeg = nullptr;
		JointNode* leftLeg = nullptr;
		JointNode* leftFoot = nullptr;
		//JointNode* leftBall = nullptr;
		JointNode* leftHeel = nullptr;
		JointNode* rightUpLeg = nullptr;
		JointNode* rightLeg = nullptr;
		JointNode* rightFoot = nullptr;
		//JointNode* rightBall = nullptr;
		JointNode* rightHeel = nullptr;

		// Foot lock (plant) state
		bool m_leftFootLocked = false;
		bool m_rightFootLocked = false;
		Vector3 m_leftFootLockPos = Vector3(0);
		Vector3 m_rightFootLockPos = Vector3(0);
		Vector3 m_lastLeftFootInputPos = Vector3(0);
		Vector3 m_lastRightFootInputPos = Vector3(0);

		// Tunables
		float m_footLockVelEnter = 0.03f; // m/s
		float m_footLockVelExit = 0.06f;  // m/s
		float m_footLockHeightThreshold = 0.08f; // scaled space
		int m_initedFrame = 0;

		// Limit for width compensation (meters in the scaled/output space). Helps prevent over-inward/outward swing.
		float m_maxHipsWidthComp = 0.03f;

		// IK knee pole distance (meters in scaled/output space)
		float m_kneePoleDist = 0.25f;
	};


}