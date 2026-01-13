#ifndef ik_h__
#define ik_h__

#include "mathutils.h"
#include <vector>
namespace eba
{
    
    struct IkTransform
    {
        Vector3 position = Vector3(0);
        Quat rotation = glm::identity<Quat>();
        Vector3 scale = Vector3(1);

        IkTransform GetRelativeTransform(const IkTransform& Other) const;

        IkTransform operator* (const IkTransform& Other) const;
    };

    struct CCDChainLink
    {
        /** Transform of bone in component space. */
        IkTransform Transform;

        /** Transform of bone in local space. This is mutable as their component space changes or parents*/
        IkTransform LocalTransform;

        /** Transform Index that this control will output */
        int TransformIndex;

        /** Child bones which are overlapping this bone.
         * They have a zero length distance, so they will inherit this bone's transformation. */
        std::vector<int> ChildZeroLengthTransformIndices;

        float CurrentAngleDelta;

        CCDChainLink()
            : TransformIndex(-1)
            , CurrentAngleDelta(0.0)
        {
        }

        CCDChainLink(const IkTransform& InTransform, const IkTransform& InLocalTransform, int InTransformIndex)
            : Transform(InTransform)
            , LocalTransform(InLocalTransform)
            , TransformIndex(InTransformIndex)
            , CurrentAngleDelta(0.0)
        {
        }
    };


    void TwoBonesIK(IkTransform& InOutRootTransform, IkTransform& InOutJointTransform, IkTransform& InOutEndTransform, const Vector3& JointTarget,
        const Vector3& Effector, bool bAllowStretching, float StartStretchRatio, float MaxStretchScale);

    bool CCDIK(std::vector<CCDChainLink>& InOutChain, const Vector3& TargetPosition, float Precision, int MaxIteration,
        bool bStartFromTail, bool bEnableRotationLimit, const std::vector<float>& RotationLimitPerJoints);



}

#endif // ik_h__
