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
		float sourceHipsWidth = 0.15f;
		float sourceHipsHeight = 0.0f;
        float sourceLegLength = 0.45f;
		float sourceAnkleHeight = 0.42f;
		//机器人实际参数
		float targetHipsWidth = 0.15f;
		float targetHipsHeight = 0.05f;
		float targetLegLength = 0.35f;
		float targetAnkleHeight = 0.3f;

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
	};


}