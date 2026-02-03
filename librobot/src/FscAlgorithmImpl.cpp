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

         // 数据源（人体）参数：用于计算scale
         sourceHipsWidth = 0.23f;
         sourceHipsHeight = 0.0f;
         sourceLegLength = 0.858f;
         sourceAnkleHeight = 0.08f;

         // 机器人参数：优先使用外部传入（config），否则回落到URDF推导的默认值
         // Ultron_EVT_S11_V2 (from provided URDF):
         // hip lateral offset: +/-0.105 -> hipsWidth ~= 0.21
         // hip->knee: 0.369, knee->ankle: 0.369 -> legLength ~= 0.738
         // ankle->foot contact: 0.05
         constexpr float kUrdfHipsWidth = 0.21f;
         constexpr float kUrdfLegLength = 0.369f + 0.369f;
         constexpr float kUrdfAnkleHeight = 0.05f;

         targetHipsWidth = (robotHipsWidth > 1e-6f) ? robotHipsWidth : kUrdfHipsWidth;
         targetHipsHeight = (robotHipsHeight > 1e-6f) ? robotHipsHeight : 0.0f;
         targetLegLength = (robotLegLength > 1e-6f) ? robotLegLength : kUrdfLegLength;
         targetAnkleHeight = (robotAnkleHeight > 1e-6f) ? robotAnkleHeight : kUrdfAnkleHeight;


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

        // IK knee pole: use hips forward axis instead of a fixed world axis.
        // This is more stable when the pelvis yaws while walking.
        leftKneeOffsetDir = Vector3(1, 0, 0);
        rightKneeOffsetDir = Vector3(1, 0, 0);
	}

    void FscAlgorithmImpl::UpdateWithDisp(int frame, const FscLowerBodyInfo& input, FscLowerBodyInfo& output)
    {
        Vector3 inputHipsPos = ConvertVec(input.hips.position);
        Quat inputHipsRot = ConvertQuat(input.hips.rotation);

        Vector3 inputLeftUpLegPos = ConvertVec(input.leftUpLeg.position);
        Vector3 inputLeftLegPos = ConvertVec(input.leftLeg.position);
        Vector3 inputLeftFootPos = ConvertVec(input.leftFoot.position);

        Vector3 inputRightUpLegPos = ConvertVec(input.rightUpLeg.position);
        Vector3 inputRightLegPos = ConvertVec(input.rightLeg.position);
        Vector3 inputRightFootPos = ConvertVec(input.rightFoot.position);

        Quat inputLeftUpLegRot = ConvertQuat(input.leftUpLeg.rotation);
        Quat inputLeftLegRot = ConvertQuat(input.leftLeg.rotation);
        Quat inputLeftFootRot = ConvertQuat(input.leftFoot.rotation);
        Quat inputRightUpLegRot = ConvertQuat(input.rightUpLeg.rotation);
        Quat inputRightLegRot = ConvertQuat(input.rightLeg.rotation);
        Quat inputRightFootRot = ConvertQuat(input.rightFoot.rotation);

        // do hip adjustment
        float scale = (targetHipsHeight + targetLegLength + targetAnkleHeight) /
            (sourceHipsHeight + sourceLegLength + sourceAnkleHeight);

        hips->SetPositionAndRotation((inputHipsPos * scale), inputHipsRot);

        // Update forward animation (positions + rotations) in WORLD space.
        // Important: TwoBonesIK relies on correct world-space positions to compute bone lengths.
        leftUpLeg->SetPosition(inputLeftUpLegPos * scale);
        leftUpLeg->SetRotation(inputLeftUpLegRot);

        leftLeg->SetPosition(inputLeftLegPos * scale);
        leftLeg->SetRotation(inputLeftLegRot);

        leftFoot->SetPosition(inputLeftFootPos * scale);
        leftFoot->SetRotation(inputLeftFootRot);

        rightUpLeg->SetPosition(inputRightUpLegPos * scale);
        rightUpLeg->SetRotation(inputRightUpLegRot);

        rightLeg->SetPosition(inputRightLegPos * scale);
        rightLeg->SetRotation(inputRightLegRot);

        rightFoot->SetPosition(inputRightFootPos * scale);
        rightFoot->SetRotation(inputRightFootRot);

        // do foot adjustment
        // Use hips lateral axis for width compensation. Using foot orientation can introduce outward swing
        // during leg lift because foot yaw/roll changes the compensation direction.
        Vector3 hipsSide = hips->GetDirY();
        hipsSide.z = 0.0f;
        if (glm::length2(hipsSide) < 1e-6f)
        {
            hipsSide = Vector3(0, 1, 0);
        }
        hipsSide = glm::normalize(hipsSide);

        // Left and right use opposite lateral directions.
        Vector3 leftFootSideOffset = hipsSide;
        Vector3 rightFootSideOffset = hipsSide;

         float vertOffset = (sourceAnkleHeight - targetAnkleHeight * scale);
         float hipsWidthDelta = 0.5f * (sourceHipsWidth - targetHipsWidth * scale);
         hipsWidthDelta = glm::clamp(hipsWidthDelta, -m_maxHipsWidthComp, m_maxHipsWidthComp);

         Vector3 leftFootAdjustPos = inputLeftFootPos * scale + Vector3(0, 0, vertOffset) + leftFootSideOffset * hipsWidthDelta;
         Vector3 rightFootAdjustPos = inputRightFootPos * scale + Vector3(0, 0, vertOffset) - rightFootSideOffset * hipsWidthDelta;

    // --- Foot lock (simple plant) ---
    // Estimate foot speed from input positions. frame is provided by caller, assume fixed fps=60.
    // This is a lightweight stabilizer to reduce sliding when a foot is on the ground.
    const float dt = 1.0f / 60.0f;
    if (m_initedFrame == 0)
    {
        m_lastLeftFootInputPos = inputLeftFootPos;
        m_lastRightFootInputPos = inputRightFootPos;
        m_leftFootLocked = false;
        m_rightFootLocked = false;
        m_initedFrame = 1;
    }

    Vector3 leftVel = (inputLeftFootPos - m_lastLeftFootInputPos) / dt;
    Vector3 rightVel = (inputRightFootPos - m_lastRightFootInputPos) / dt;
    m_lastLeftFootInputPos = inputLeftFootPos;
    m_lastRightFootInputPos = inputRightFootPos;

    float leftSpeed = glm::length(leftVel);
    float rightSpeed = glm::length(rightVel);

    // Use scaled height for contact check; threshold is in scaled space.
    float leftHeight = leftFootAdjustPos.z;
    float rightHeight = rightFootAdjustPos.z;

    auto updateLock = [&](bool& locked, Vector3& lockPos, float speed, float height, const Vector3& currentAdjustPos)
    {
        if (!locked)
        {
            if (speed < m_footLockVelEnter && height < m_footLockHeightThreshold)
            {
                locked = true;
                lockPos = currentAdjustPos;
            }
        }
        else
        {
            if (speed > m_footLockVelExit || height > m_footLockHeightThreshold)
            {
                locked = false;
            }
        }

        return locked ? lockPos : currentAdjustPos;
    };

    leftFootAdjustPos = updateLock(m_leftFootLocked, m_leftFootLockPos, leftSpeed, leftHeight, leftFootAdjustPos);
    rightFootAdjustPos = updateLock(m_rightFootLocked, m_rightFootLockPos, rightSpeed, rightHeight, rightFootAdjustPos);

        // hips output
        output.hips.position = ConvertVec(hips->GetPosition());
        output.hips.rotation = ConvertQuat(hips->GetRotation());

#if 0 // no ik
        // ...existing code...
#else
    // do leg ik (world-space)
    IkTransform leftUpLegTrans;
    leftUpLegTrans.position = leftUpLeg->GetPosition();
    leftUpLegTrans.rotation = leftUpLeg->GetRotation();

    IkTransform leftLegTrans;
    leftLegTrans.position = leftLeg->GetPosition();
    leftLegTrans.rotation = leftLeg->GetRotation();

    IkTransform leftFootTrans;
    leftFootTrans.position = leftFoot->GetPosition();
    leftFootTrans.rotation = leftFoot->GetRotation();

    Vector3 leftKneeOffset = rotate(leftLegTrans.rotation, leftKneeOffsetDir);
    // Use pelvis-forward projected onto ground as a stable pole direction for walking.
    Vector3 pelvisFwd = hips->GetDirX();
    pelvisFwd.z = 0.0f;
    if (glm::length2(pelvisFwd) < 1e-6f)
        pelvisFwd = Vector3(1, 0, 0);
    pelvisFwd = glm::normalize(pelvisFwd);
    Vector3 leftKneeEffector = leftLegTrans.position + pelvisFwd * m_kneePoleDist;

    // Keep stretching disabled for stability with scaled data.
    TwoBonesIK(leftUpLegTrans, leftLegTrans, leftFootTrans, leftKneeEffector, leftFootAdjustPos,
        false, 0.995f, 1.1f);

    IkTransform rightUpLegTrans;
    rightUpLegTrans.position = rightUpLeg->GetPosition();
    rightUpLegTrans.rotation = rightUpLeg->GetRotation();

    IkTransform rightLegTrans;
    rightLegTrans.position = rightLeg->GetPosition();
    rightLegTrans.rotation = rightLeg->GetRotation();

    IkTransform rightFootTrans;
    rightFootTrans.position = rightFoot->GetPosition();
    rightFootTrans.rotation = rightFoot->GetRotation();

    Vector3 rightKneeOffset = rotate(rightLegTrans.rotation, rightKneeOffsetDir);
    Vector3 rightKneeEffector = rightLegTrans.position + pelvisFwd * m_kneePoleDist;

    TwoBonesIK(rightUpLegTrans, rightLegTrans, rightFootTrans, rightKneeEffector, rightFootAdjustPos,
        false, 0.995f, 1.1f);

    // Write back solved transforms to nodes so subsequent getters reflect IK results.
    leftUpLeg->SetPosition(leftUpLegTrans.position);
    leftUpLeg->SetRotation(leftUpLegTrans.rotation);
    leftLeg->SetPosition(leftLegTrans.position);
    leftLeg->SetRotation(leftLegTrans.rotation);
    leftFoot->SetPosition(leftFootTrans.position);
    leftFoot->SetRotation(inputLeftFootRot);

    rightUpLeg->SetPosition(rightUpLegTrans.position);
    rightUpLeg->SetRotation(rightUpLegTrans.rotation);
    rightLeg->SetPosition(rightLegTrans.position);
    rightLeg->SetRotation(rightLegTrans.rotation);
    rightFoot->SetPosition(rightFootTrans.position);
    rightFoot->SetRotation(inputRightFootRot);

    output.leftUpLeg.position = ConvertVec(leftUpLeg->GetPosition());
    output.leftUpLeg.rotation = ConvertQuat(leftUpLeg->GetRotation());

    output.leftLeg.position = ConvertVec(leftLeg->GetPosition());
    output.leftLeg.rotation = ConvertQuat(leftLeg->GetRotation());

    output.rightUpLeg.position = ConvertVec(rightUpLeg->GetPosition());
    output.rightUpLeg.rotation = ConvertQuat(rightUpLeg->GetRotation());

    output.rightLeg.position = ConvertVec(rightLeg->GetPosition());
    output.rightLeg.rotation = ConvertQuat(rightLeg->GetRotation());
#endif

    output.leftFoot.position = ConvertVec(leftFootAdjustPos);
    output.leftFoot.rotation = ConvertQuat(inputLeftFootRot);
    output.rightFoot.position = ConvertVec(rightFootAdjustPos);
    output.rightFoot.rotation = ConvertQuat(inputRightFootRot);
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

	    // Register first, so children can find parent in the same Init() call order.
	    boneMap[name] = bone;

	    if (!parentName.empty())
	    {
	        auto it = boneMap.find(parentName);
	        JointNode* parentBone = (it != boneMap.end()) ? it->second : nullptr;
	        bone->SetParent(parentBone);
	    }

	    return bone;
	}

}