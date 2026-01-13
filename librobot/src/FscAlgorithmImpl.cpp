#include "FscAlgorithmImpl.h"
#include "ik.h"
#include"jointnode.h"
#include"string"
#include"map"
#include <iostream>

namespace eba
{

	static Vector3 ConvertVec(const Vector& vec)
	{
		return Vector3(vec.value[0], vec.value[1], vec.value[2]);
	}

	static Quat ConvertQuat(const Quaternion& quat)
	{
		return Quat(quat.value[3], quat.value[0], quat.value[1], quat.value[2]);
	}

	static Vector ConvertVec(const Vector3& vec)
	{
		Vector result;
		result.value[0] = vec.x;
		result.value[1] = vec.y;
		result.value[2] = vec.z;
		return result;
	}

	static Quaternion ConvertQuat(const Quat& quat)
	{
		Quaternion result;
		result.value[0] = quat.x;
		result.value[1] = quat.y;
		result.value[2] = quat.z;
		result.value[3] = quat.w;
		return result;
	}

	FscAlgorithmImpl::FscAlgorithmImpl()
	{
	}

	FscAlgorithmImpl::~FscAlgorithmImpl()
	{
	}

	void FscAlgorithmImpl::Init(float robotHipsWidth, float robotHipsHeight, float robotLegLength, float robotAnkleHeight, const FscLowerBodyInfo& initPose)
	{
        //方向默认x为前
         Vector modelForward = { {1,0,0} };
         Vector modelLeft = { {0,-1,0} };
        //数据源参数暂时写死
         sourceHipsWidth = 0.15f;
         sourceHipsHeight = 0.0f;
         sourceLegLength = 0.45f;
         sourceAnkleHeight = 0.42f;

         //机器人参数
         targetHipsWidth = robotHipsWidth;
         targetHipsHeight = robotHipsHeight;
         targetLegLength = robotLegLength;
         targetAnkleHeight = robotAnkleHeight;


		hips = CreateBone(0, "Hips", "", initPose.hips);
		leftUpLeg = CreateBone(1, "LeftUpLeg", "Hips", initPose.leftUpLeg);
		leftLeg = CreateBone(2, "LeftLeg", "LeftUpLeg", initPose.leftLeg);
		leftFoot = CreateBone(3, "LeftFoot", "LeftLeg", initPose.leftFoot);
        rightUpLeg = CreateBone(4, "RightUpLeg", "Hips", initPose.rightUpLeg);
        rightLeg = CreateBone(5, "RightLeg", "RightUpLeg", initPose.rightLeg);
        rightFoot = CreateBone(6, "RightFoot", "RightLeg", initPose.rightFoot);

        this->sourceHipsWidth =glm::length(leftUpLeg->GetPosition() - rightUpLeg->GetPosition());

        this->modelForward = ConvertVec(modelForward);
        this->modelLeft = ConvertVec(modelLeft);

        leftFootEffector = CreateBone(7, "LeftFootEffector", "LeftFoot", initPose.leftFoot);
        rightFootEffector = CreateBone(8, "RightFootEffector", "RightFoot", initPose.rightFoot);
        leftFootEffector->Translate(-Vector3(this->modelForward) , eba::TransformSpace::World);
        rightFootEffector->Translate(-Vector3(this->modelForward), eba::TransformSpace::World);

        leftFootOffsetDir = Vector3(1, 0, 0);
        rightFootOffsetDir = Vector3(1, 0, 0);
        leftKneeOffsetDir = Vector3(1, 0, 0);
        leftKneeOffsetDir = Vector3(1, 0, 0);

	}

    void FscAlgorithmImpl::UpdateWithDisp(int frame, const FscLowerBodyInfo& input, FscLowerBodyInfo& output)
    {

		Vector3 inputHipsPos = ConvertVec(input.hips.position);
		Quat inputHipsRot = ConvertQuat(input.hips.rotation);

        Quat inputLeftUpLegRot = ConvertQuat(input.leftUpLeg.rotation);
        Quat inputLeftLegRot = ConvertQuat(input.leftLeg.rotation);
        Quat inputLeftFootRot = ConvertQuat(input.leftFoot.rotation);
        Quat inputRightUpLegRot = ConvertQuat(input.rightUpLeg.rotation);
        Quat inputRightLegRot = ConvertQuat(input.rightLeg.rotation);
        Quat inputRightFootRot = ConvertQuat(input.rightFoot.rotation);

		// do hip adjustment
        //获取身体尺寸，//机器人最终髋高， 真人髋高*（机器人髋部长度+机器人大腿长+机器人小腿长）/（真人髋长+真人大腿长+真人小腿长）
        float scale = (targetHipsHeight+targetLegLength + targetAnkleHeight) / (sourceHipsHeight+sourceLegLength + sourceAnkleHeight);
        //设置hips的位置和姿态
        hips->SetPositionAndRotation((inputHipsPos * scale), inputHipsRot);


		// do forward animation without displacement
        leftUpLeg->SetRotation(inputLeftUpLegRot);
        rightUpLeg->SetRotation(inputRightUpLegRot);

        leftLeg->SetRotation(inputLeftLegRot);
        rightLeg->SetRotation(inputRightLegRot);

        // do foot adjustment
        Vector3 inputLeftFootPos = ConvertVec(input.leftFoot.position);
        Vector3 inputRightFootPos = ConvertVec(input.rightFoot.position);

        Vector3 leftFootSideOffset = rotate(inputLeftFootRot, leftFootOffsetDir);
        Vector3 rightFootSideOffset = rotate(inputRightFootRot, rightFootOffsetDir);

		float vertOffset = ( sourceAnkleHeight- targetAnkleHeight * scale);
		float hipsWidthDelta = 0.5f * ( sourceHipsWidth -targetHipsWidth * scale);
        Vector3 leftFootAdjustPos = inputLeftFootPos * scale + Vector3(0,0 ,vertOffset) + leftFootSideOffset * hipsWidthDelta;
		Vector3 rightFootAdjustPos = inputRightFootPos * scale + Vector3(0,0,vertOffset) - rightFootSideOffset * hipsWidthDelta;

		// output
        //hips 只根据身体尺寸调整了下高度，
        output.hips.position = ConvertVec(hips->GetPosition());
        output.hips.rotation = ConvertQuat(hips->GetRotation());

		
#if 1 // no ik
        const Vector3& leftKneePos = leftLeg->GetPosition();
        const Vector3& rightKneePos = rightLeg->GetPosition();

        Vector3 leftLegOrigDir = normalize(leftFoot->GetPosition() - leftKneePos);
        Vector3 rightLegOrigDir = normalize(rightFoot->GetPosition() - rightKneePos);

        Vector3 leftLegDir = normalize(leftFootAdjustPos - leftKneePos);
        Vector3 rightLegDir = normalize(rightFootAdjustPos - rightKneePos);

        Quat leftLegDeltaQ = glm::rotation(leftLegOrigDir, leftLegDir);
        Quat rightLegDeltaQ = glm::rotation(rightLegOrigDir, rightLegDir);

        leftLeg->SetRotation(leftLegDeltaQ * inputLeftLegRot);
        rightLeg->SetRotation(rightLegDeltaQ * inputRightLegRot);

        leftFoot->SetPosition(leftFootAdjustPos);
        rightFoot->SetPosition(rightFootAdjustPos);
        leftFoot->SetRotation(inputLeftFootRot);
        rightFoot->SetRotation(inputRightFootRot);

        output.leftUpLeg.rotation = ConvertQuat(leftUpLeg->GetRotation());
        output.rightUpLeg.rotation = ConvertQuat(rightUpLeg->GetRotation());
        output.leftLeg.position = ConvertVec(leftLeg->GetPosition());
		output.leftLeg.rotation = ConvertQuat(leftLeg->GetRotation());
        output.rightLeg.position = ConvertVec(rightLeg->GetPosition());
		output.rightLeg.rotation = ConvertQuat(rightLeg->GetRotation());
#else

        // do leg ik
        IkTransform leftUpLegTrans;
        leftUpLegTrans.position = leftUpLeg->GetPosition();
        leftUpLegTrans.rotation = leftUpLeg->GetRotation();

        IkTransform leftLegTrans;
        leftLegTrans.position = leftLeg->GetPosition();
        leftLegTrans.rotation = leftLeg->GetRotation();

        IkTransform leftFootTrans;
        leftFootTrans.position = leftFoot->GetPosition();
        leftFootTrans.rotation = leftFoot->GetRotation();

        Vector3 leftKneeOffset = rotate(leftLeg->GetRotation(), leftKneeOffsetDir);
        Vector3 leftKneeEffector = leftLeg->GetPosition() + leftKneeOffset * 0.2f;
//         Vector3 leftFootForward = leftLeg->GetForward();
//         leftFootForward.setY(0);
//         leftFootForward = normalize(leftFootForward);
//         leftFootAdjustPos -= leftFootForward * 0.23f;

        //将大腿、小腿和脚，通过ik进行调整
        TwoBonesIK(leftUpLegTrans, leftLegTrans, leftFootTrans, leftKneeEffector, leftFootAdjustPos, true, 0.995f, 1.1f);

        IkTransform rightUpLegTrans;
        rightUpLegTrans.position = rightUpLeg->GetPosition();
        rightUpLegTrans.rotation = rightUpLeg->GetRotation();

        IkTransform rightLegTrans;
        rightLegTrans.position = rightLeg->GetPosition();
        rightLegTrans.rotation = rightLeg->GetRotation();

        IkTransform rightFootTrans;
        rightFootTrans.position = rightFoot->GetPosition();
        rightFootTrans.rotation = rightFoot->GetRotation();

        Vector3 rightKneeOffset = rotate(rightLeg->GetRotation(), rightKneeOffsetDir);
        Vector3 rightKneeEffector = rightLeg->GetPosition() + rightKneeOffset * 0.2f;

//         Vector3 rightFootForward = rightLeg->GetForward();
//         rightFootForward.setY(0);
//         rightFootForward = normalize(rightFootForward);
//         rightFootAdjustPos -= rightFootForward * 0.23f;

        TwoBonesIK(rightUpLegTrans, rightLegTrans, rightFootTrans, rightKneeEffector, rightFootAdjustPos, true, 0.995f, 1.1f);

        output.leftUpLeg.rotation = ConvertQuat(leftUpLegTrans.rotation);
        output.rightUpLeg.rotation = ConvertQuat(rightUpLegTrans.rotation);
        output.rightLeg.position = ConvertVec(rightLegTrans.position);
        output.rightLeg.rotation = ConvertQuat(rightLegTrans.rotation);
        output.leftLeg.position = ConvertVec(leftLegTrans.position);
        output.leftLeg.rotation = ConvertQuat(leftLegTrans.rotation);
#endif

        output.leftFoot.position = ConvertVec(leftFootAdjustPos);
        output.leftFoot.rotation = ConvertQuat(leftFoot->GetRotation());
        output.rightFoot.position = ConvertVec(rightFootAdjustPos);
        output.rightFoot.rotation = ConvertQuat(rightFoot->GetRotation());
    }

     Quat FscAlgorithmImpl::fromVectors(const glm::vec3& from, const glm::vec3& to) {
        // 计算两个向量的点积
        float cosTheta = glm::dot(from, to);
        glm::vec3 rotationAxis;

        if (cosTheta < -1 + 0.001f) {
            // 当两个向量方向完全相反时
            // 找到一个与 from 向量垂直的向量作为旋转轴
            glm::vec3 arbitraryVector(1.0f, 0.0f, 0.0f);
            if (glm::abs(glm::dot(from, arbitraryVector)) > 0.999f) {
                arbitraryVector = glm::vec3(0.0f, 1.0f, 0.0f);
            }
            rotationAxis = glm::normalize(glm::cross(from, arbitraryVector));
            return Quat(glm::angleAxis(glm::pi<float>(), rotationAxis));
        }

        // 正常情况，计算旋转轴
        rotationAxis = glm::cross(from, to);
        float s = glm::sqrt((1 + cosTheta) * 2);
        float invs = 1 / s;

        return Quat(glm::quat(
            s * 0.5f,
            rotationAxis.x * invs,
            rotationAxis.y * invs,
            rotationAxis.z * invs
        ));
    }

	void FscAlgorithmImpl::DrawSphere(const Vector3& pos, const Vector4& color)
	{
		/*auto dde = eba::instance().GetDDE();
		if (dde == nullptr)
			return;

		dde->push();
		dde->setState(false, true, true);
		dde->setColor(toABGR(color));
		bx::Sphere sp = { { pos.getX(), pos.getY(), pos.getZ() }, 0.02f };
		dde->draw(sp);
		dde->pop();*/
	}

	JointNode* FscAlgorithmImpl::CreateBone(int index, const std::string& name, const std::string& parentName, const FscBoneInfo& boneInfo)
	{
		JointNode* bone = new JointNode();

		bone->SetPosition(ConvertVec(boneInfo.position));
		bone->SetRotation(ConvertQuat(boneInfo.rotation));

		if (parentName != "")
		{
			JointNode* parentBone = boneMap[parentName];
			bone->SetParent(parentBone);
		}

	//	boneMap.insert({ name, bone });
	//	bone->SaveCurrentPose();
		return bone;
	}

}