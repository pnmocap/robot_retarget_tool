#pragma once

namespace eba
{
	struct Vector
	{
		float value[3] = { 0, 0, 0 };
	};

	struct Quaternion
	{
		float value[4] = { 0, 0, 0, 1 };
	};

	struct FscBoneInfo
	{
		Vector position;
		Quaternion rotation;
	};

	struct FscContactInfo
	{
		bool leftBall = false;
		bool leftHeel = false;
		bool rightBall = false;
		bool rightHeel = false;
		float leftLowest = 0.0f;
		float rightLowest = 0.0f;
	};

	struct FscLowerBodyInfo
	{
		FscBoneInfo hips;
		FscBoneInfo leftUpLeg;
		FscBoneInfo leftLeg;
		FscBoneInfo leftFoot;
// 		FscBoneInfo leftBall;
// 		FscBoneInfo leftHeel;
		FscBoneInfo rightUpLeg;
		FscBoneInfo rightLeg;
		FscBoneInfo rightFoot;
// 		FscBoneInfo rightBall;
// 		FscBoneInfo rightHeel;
	};

	class  FscAlgorithm
	{
	public:
		virtual void Init(float robotHipsWidth, float robotHipsHeight, float robotLegLength, float robotAnkleHeight, const FscLowerBodyInfo& initPose) = 0;

		virtual void UpdateWithDisp(int frame, const FscLowerBodyInfo& input, FscLowerBodyInfo& output) = 0;

//		virtual bool UpdateContactInfo(int frame, const FscContactInfo& contactInfo, const FscLowerBodyInfo& input, FscLowerBodyInfo& output) = 0;
	};


	 FscAlgorithm* CreateFscAlgorithm();

	 void DestroyFscAlgorithm(FscAlgorithm* fsc);

}