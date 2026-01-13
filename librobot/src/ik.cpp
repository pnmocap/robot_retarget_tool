#include "ik.h"
#include "glm/ext/quaternion_trigonometric.hpp"

namespace eba
{

    IkTransform IkTransform::GetRelativeTransform(const IkTransform& Other) const
    {
        IkTransform result;
        Vector3 SafeRecipScale3D = getSafeScaleReciprocal(Other.scale);
        result.scale = scale * SafeRecipScale3D;
        Quat Inverse = glm::inverse(Other.rotation);
        result.rotation = Inverse * rotation;
        result.position = glm::rotate(Inverse, (position - Other.position)) * SafeRecipScale3D;
        return result;
    }

    IkTransform IkTransform::operator* (const IkTransform& Other) const
    {
        IkTransform result;
        result.rotation = rotation * Other.rotation;
        result.scale = scale * Other.scale;
        result.position = position + glm::rotate(rotation, scale * Other.position);
        return result;
    }

    void TwoBonesIK(IkTransform& InOutRootTransform, IkTransform& InOutJointTransform, IkTransform& InOutEndTransform, const Vector3& JointTarget,
        const Vector3& Effector, bool bAllowStretching, float StartStretchRatio, float MaxStretchScale)
    {
        float LowerLimbLength = glm::length(InOutEndTransform.position - InOutJointTransform.position);
        float UpperLimbLength = glm::length(InOutJointTransform.position - InOutRootTransform.position);

        Vector3 OutJointPos, OutEndPos;

        Vector3 RootPos = InOutRootTransform.position;
        Vector3 JointPos = InOutJointTransform.position;
        Vector3 EndPos = InOutEndTransform.position;

        // This is our reach goal.
        Vector3 DesiredPos = Effector;
        Vector3 DesiredDelta = DesiredPos - RootPos;
        float DesiredLength = glm::length(DesiredDelta);

        // Find lengths of upper and lower limb in the ref skeleton.
        // Use actual sizes instead of ref skeleton, so we take into account translation and scaling from other bone controllers.
        float MaxLimbLength = LowerLimbLength + UpperLimbLength;

        // Check to handle case where DesiredPos is the same as RootPos.
        Vector3	DesiredDir;
        if (DesiredLength < (float)KINDA_SMALL_NUMBER)
        {
            DesiredLength = (float)KINDA_SMALL_NUMBER;
            DesiredDir = Vector3(1, 0, 0);
        }
        else
        {
            DesiredDir = glm::normalize(DesiredDelta);
        }

        // Get joint target (used for defining plane that joint should be in).
        Vector3 JointTargetDelta = JointTarget - RootPos;
        const float JointTargetLengthSqr = glm::length2(JointTargetDelta);

        // Same check as above, to cover case when JointTarget position is the same as RootPos.
        Vector3 JointPlaneNormal, JointBendDir;
        if (JointTargetLengthSqr < pow((float)KINDA_SMALL_NUMBER, 2))
        {
            JointBendDir = Vector3(0, 1, 0);
            JointPlaneNormal = Vector3(0, 0, 1);
        }
        else
        {
            JointPlaneNormal = glm::cross(DesiredDir, JointTargetDelta);

            // If we are trying to point the limb in the same direction that we are supposed to displace the joint in, 
            // we have to just pick 2 random vector perp to DesiredDir and each other.
            if (glm::length2(JointPlaneNormal) < pow((float)KINDA_SMALL_NUMBER, 2))
            {
                FindBestAxisVectors(DesiredDir, JointPlaneNormal, JointBendDir);
            }
            else
            {
                JointPlaneNormal = glm::normalize(JointPlaneNormal);
                // Find the final member of the reference frame by removing any component of JointTargetDelta along DesiredDir.
                // This should never leave a zero vector, because we've checked DesiredDir and JointTargetDelta are not parallel.
                JointBendDir = JointTargetDelta - glm::dot(JointTargetDelta, DesiredDir) * DesiredDir;
                JointBendDir = glm::normalize(JointBendDir);
            }
        }

        //UE_LOG(LogAnimationCore, Log, TEXT("UpperLimb : %0.2f, LowerLimb : %0.2f, MaxLimb : %0.2f"), UpperLimbLength, LowerLimbLength, MaxLimbLength);

        if (bAllowStretching)
        {
            const float ScaleRange = MaxStretchScale - StartStretchRatio;
            if (ScaleRange > KINDA_SMALL_NUMBER && MaxLimbLength > KINDA_SMALL_NUMBER)
            {
                const float ReachRatio = DesiredLength / MaxLimbLength;
                const float ScalingFactor = (MaxStretchScale - 1.0) * glm::clamp((ReachRatio - StartStretchRatio) / ScaleRange, 0.0f, 1.0f);
                if (ScalingFactor > KINDA_SMALL_NUMBER)
                {
                    LowerLimbLength *= (1.0 + ScalingFactor);
                    UpperLimbLength *= (1.0 + ScalingFactor);
                    MaxLimbLength *= (1.0 + ScalingFactor);
                }
            }
        }

        OutEndPos = DesiredPos;
        OutJointPos = JointPos;

        // If we are trying to reach a goal beyond the length of the limb, clamp it to something solvable and extend limb fully.
        if (DesiredLength >= MaxLimbLength)
        {
            OutEndPos = RootPos + (MaxLimbLength * DesiredDir);
            OutJointPos = RootPos + (UpperLimbLength * DesiredDir);
        }
        else
        {
            // So we have a triangle we know the side lengths of. We can work out the angle between DesiredDir and the direction of the upper limb
            // using the sin rule:
            const float TwoAB = 2.0 * UpperLimbLength * DesiredLength;

            const float CosAngle = (TwoAB != 0.0) ? ((UpperLimbLength * UpperLimbLength) + (DesiredLength * DesiredLength) - (LowerLimbLength * LowerLimbLength)) / TwoAB : 0.0;

            // If CosAngle is less than 0, the upper arm actually points the opposite way to DesiredDir, so we handle that.
            const bool bReverseUpperBone = (CosAngle < 0.0);

            // Angle between upper limb and DesiredDir
            // ACos clamps internally so we donot need to worry about out-of-range values here.
            const float Angle = glm::acos(CosAngle);

            // Now we calculate the distance of the joint from the root -> effector line.
            // This forms a right-angle triangle, with the upper limb as the hypotenuse.
            const float JointLineDist = UpperLimbLength * sin(Angle);

            // And the final side of that triangle - distance along DesiredDir of perpendicular.
            // ProjJointDistSqr can't be neg, because JointLineDist must be <= UpperLimbLength because appSin(Angle) is <= 1.
            const float ProjJointDistSqr = (UpperLimbLength * UpperLimbLength) - (JointLineDist * JointLineDist);
            // although this shouldn't be ever negative, sometimes Xbox release produces -0.0, causing ProjJointDist to be NaN
            // so now I branch it. 						
            float ProjJointDist = (ProjJointDistSqr > 0.0) ? glm::sqrt(ProjJointDistSqr) : 0.0;
            if (bReverseUpperBone)
            {
                ProjJointDist *= -1.0;
            }

            // So now we can work out where to put the joint!
            OutJointPos = RootPos + (ProjJointDist * DesiredDir) + (JointLineDist * JointBendDir);
        }

        // Update transform for upper bone.
        {
            // Get difference in direction for old and new joint orientations
            Vector3 const OldDir = glm::normalize(JointPos - RootPos);
            Vector3 const NewDir = glm::normalize(OutJointPos - RootPos);
            // Find Delta rot take takes us from Old to New dir
            Quat const DeltaRotation = glm::rotation(OldDir, NewDir);
            // Rotate our Joint quaternion by this delta rotation
            InOutRootTransform.rotation = (DeltaRotation * InOutRootTransform.rotation);
            // And put joint where it should be.
            InOutRootTransform.position = RootPos;

        }

        // update transform for middle bone
        {
            // Get difference in direction for old and new joint orientations
            Vector3 const OldDir = glm::normalize(EndPos - JointPos);
            Vector3 const NewDir = glm::normalize(OutEndPos - OutJointPos);

            // Find Delta rot take takes us from Old to New dir
            Quat const DeltaRotation = glm::rotation(OldDir, NewDir);
            // Rotate our Joint quaternion by this delta rotation
            InOutJointTransform.rotation = (DeltaRotation * InOutJointTransform.rotation);
            // And put joint where it should be.
            InOutJointTransform.position = OutJointPos;

        }

        // Update transform for end bone.
        // currently not doing anything to rotation
        // keeping input rotation
        // Set correct location for end bone.
        InOutEndTransform.position = OutEndPos;
    }

    bool CCDIK(std::vector<CCDChainLink>& InOutChain, const Vector3& TargetPosition, float Precision, int MaxIteration, bool bStartFromTail, bool bEnableRotationLimit, const std::vector<float>& RotationLimitPerJoints)
    {
        struct Local
        {
            static bool UpdateChainLink(std::vector<CCDChainLink>& Chain, int LinkIndex, const Vector3& TargetPos, bool bInEnableRotationLimit, const std::vector<float>& InRotationLimitPerJoints)
            {
                int const TipBoneLinkIndex = Chain.size() - 1;

                assert(TipBoneLinkIndex >= 0 && TipBoneLinkIndex < Chain.size());
                CCDChainLink& CurrentLink = Chain[LinkIndex];

                // update new tip pos
                Vector3 TipPos = Chain[TipBoneLinkIndex].Transform.position;

                IkTransform& CurrentLinkTransform = CurrentLink.Transform;
                Vector3 ToEnd = TipPos - CurrentLinkTransform.position;
                Vector3 ToTarget = TargetPos - CurrentLinkTransform.position;

                ToEnd = normalize(ToEnd);
                ToTarget = normalize(ToTarget);

                float RotationLimitPerJointInRadian = glm::radians(InRotationLimitPerJoints[LinkIndex]);
                float Angle = ClampAngleRad(acosf(glm::dot(ToEnd, ToTarget)), -RotationLimitPerJointInRadian, RotationLimitPerJointInRadian);
                bool bCanRotate = (fabsf(Angle) > KINDA_SMALL_NUMBER) && (!bInEnableRotationLimit || RotationLimitPerJointInRadian > CurrentLink.CurrentAngleDelta);
                if (bCanRotate)
                {
                    // check rotation limit first, if fails, just abort
                    if (bInEnableRotationLimit)
                    {
                        if (RotationLimitPerJointInRadian < CurrentLink.CurrentAngleDelta + Angle)
                        {
                            Angle = RotationLimitPerJointInRadian - CurrentLink.CurrentAngleDelta;
                            if (Angle <= KINDA_SMALL_NUMBER)
                            {
                                return false;
                            }
                        }

                        CurrentLink.CurrentAngleDelta += Angle;
                    }

                    // continue with rotating toward to target
                    Vector3 RotationAxis = cross(ToEnd, ToTarget);
                    if (glm::length2(RotationAxis) > 0.0)
                    {
                        RotationAxis = glm::normalize(RotationAxis);
                        // Delta Rotation is the rotation to target
                        Quat DeltaRotation = glm::angleAxis(Angle, RotationAxis);

                        Quat NewRotation = DeltaRotation * CurrentLinkTransform.rotation;
                        NewRotation = glm::normalize(NewRotation);
                        CurrentLinkTransform.rotation = NewRotation;

                        // if I have parent, make sure to refresh local transform since my current transform has changed
                        if (LinkIndex > 0)
                        {
                            CCDChainLink const& Parent = Chain[LinkIndex - 1];
                            CurrentLink.LocalTransform = CurrentLinkTransform.GetRelativeTransform(Parent.Transform);
                            CurrentLink.LocalTransform.rotation = normalize(CurrentLink.LocalTransform.rotation);
                        }

                        // now update all my children to have proper transform
                        IkTransform CurrentParentTransform = CurrentLinkTransform;

                        // now update all chain
                        for (int ChildLinkIndex = LinkIndex + 1; ChildLinkIndex <= TipBoneLinkIndex; ++ChildLinkIndex)
                        {
                            CCDChainLink& ChildIterLink = Chain[ChildLinkIndex];
                            IkTransform LocalTransform = ChildIterLink.LocalTransform;
                            ChildIterLink.Transform = LocalTransform * CurrentParentTransform;
                            ChildIterLink.Transform.rotation = normalize(ChildIterLink.Transform.rotation);
                            CurrentParentTransform = ChildIterLink.Transform;
                        }

                        return true;
                    }
                }

                return false;
            }
        };

        bool bBoneLocationUpdated = false;
        int const NumChainLinks = InOutChain.size();

        // iterate
        {
            int const TipBoneLinkIndex = NumChainLinks - 1;

            // @todo optimize locally if no update, stop?
            bool bLocalUpdated = false;
            // check how far
            const Vector3 TargetPos = TargetPosition;
            Vector3 TipPos = InOutChain[TipBoneLinkIndex].Transform.position;
            float Distance = length(TargetPos - TipPos);
            int IterationCount = 0;
            while ((Distance > Precision) && (IterationCount++ < MaxIteration))
            {
                // iterate from tip to root
                if (bStartFromTail)
                {
                    for (int LinkIndex = TipBoneLinkIndex - 1; LinkIndex > 0; --LinkIndex)
                    {
                        bLocalUpdated |= Local::UpdateChainLink(InOutChain, LinkIndex, TargetPos, bEnableRotationLimit, RotationLimitPerJoints);
                    }
                }
                else
                {
                    for (int LinkIndex = 1; LinkIndex < TipBoneLinkIndex; ++LinkIndex)
                    {
                        bLocalUpdated |= Local::UpdateChainLink(InOutChain, LinkIndex, TargetPos, bEnableRotationLimit, RotationLimitPerJoints);
                    }
                }

                Distance = length(InOutChain[TipBoneLinkIndex].Transform.position - TargetPosition);

                bBoneLocationUpdated |= bLocalUpdated;

                // no more update in this iteration
                if (!bLocalUpdated)
                {
                    break;
                }
            }
        }

        return bBoneLocationUpdated;
    }
}