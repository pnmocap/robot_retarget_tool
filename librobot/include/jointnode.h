#ifndef jointnode_h__
#define jointnode_h__

#include "mathutils.h"
#include <memory>
#include <vector>


namespace eba
{
    
    enum class TransformSpace
    {
        World,
        Parent,
        Local,
    };

    class JointNode
    {
    public:
        JointNode();

        void SetParent(JointNode* parent);

        void DetachChildren();

        JointNode* GetParent() const
        {
            return m_Parent;
        }

        JointNode* GetChild(int index);
        size_t GetChildCount() const;
        bool IsChildOf(JointNode* parent);

        Vector3 TransformDirection(float x, float y, float z);
        Vector3 TransformDirection(const Vector3& direction);
        Vector3 TransformPoint(float x, float y, float z);
        Vector3 TransformPoint(const Vector3& position);

        Vector3 InverseTransformDirection(const Vector3& direction);
        Vector3 InverseTransformDirection(float x, float y, float z);
        Vector3 InverseTransformPoint(float x, float y, float z);
        Vector3 InverseTransformPoint(const Vector3& position);

        void Reset();

        void Rotate(const Quat& quat, TransformSpace relativeTo = TransformSpace::Local);
        void Rotate(const Vector3& eulers, TransformSpace relativeTo = TransformSpace::Local);
        void Rotate(float xAngle, float yAngle, float zAngle, TransformSpace relativeTo = TransformSpace::Local);
        void Rotate(const Vector3& axis, float angle, TransformSpace relativeTo = TransformSpace::Local);

        void SetPositionAndRotation(const Vector3& position, Quat rotation);
        void CopyPositionAndRotation(JointNode* other);
        void CopyTransform(JointNode* other);

        void Translate(float x, float y, float z, TransformSpace relativeTo = TransformSpace::Local);
        void Translate(const Vector3& translation, TransformSpace relativeTo = TransformSpace::Local);

        void SetLocalPosition(float x, float y, float z);
        void SetLocalPosition(const Vector3& position);
        const Vector3& GetLocalPosition() const { return m_LocalPosition; }

        void SetLocalEulerAngles(float pitch, float yaw, float roll);
        void SetLocalEulerAngles(const Vector3& euler);
        const Vector3& GetLocalEulerAngles() const { return m_LocalEulerAngles; }

        void SetLocalRotation(float x, float y, float z);
        void SetLocalRotation(const Quat& quat);
        const Quat& GetLocalRotation() const { return m_LocalRotation; }

        void SetLocalScale(const Vector3& scale);
        const Vector3& GetLocalScale() const { return m_LocalScale; }

        const Vector3 GetDirX() const;
        const Vector3 GetDirY() const;
        const Vector3 GetDirZ() const;

        void SetRotation(const Quat& quat);
        const Quat& GetRotation() const;

        void SetPosition(float x, float y, float z);
        void SetPosition(const Vector3& position);
        const Vector3& GetPosition() const;

        void SetEulerAngles(float pitch, float yaw, float roll);
        void SetEulerAngles(const Vector3& euler);
        const Vector3& GetEulerAngles() const;

        void SetScale(float value);
        void SetScale(float x, float y, float z);
        void SetScale(const Vector3& scale);
        const Vector3& GetScale() const;

        const Matrix4& GetWorldMatrix() const;
        void SetWorldMatrix(const Matrix4& m);

        void SetLocalIdentity();

        int GetHierarchyDeep();


        static void GetRelative(const Vector3& PosA, const Quat& RotA, const Vector3& ScaleA,
            const Vector3& PosB, const Quat& RotB, const Vector3& ScaleB,
            Vector3& OutPos, Quat& OutRot, Vector3& OutScale);

        static void DoTransform(const Vector3& Pos, const Quat& Rot, const Vector3& Scale,
            const Vector3& RelPos, const Quat& RelRot, const Vector3& RelScale,
            Vector3& OutPos, Quat& OutRot, Vector3& OutScale);

    protected:
        void AddChild(JointNode* child);
        void RemoveChild(JointNode* child);
        void MakeDirty();
        void UpdateLocal();
        void UpdateWorld();
        void UpdateDirections();
        void UpdateWorldMatrix();
        void UpdateTree();
        const JointNode* GetDirtyAncestor() const;
        void CheckUpdateTree() const;

    protected:
        JointNode* m_Parent;
        std::vector<JointNode*> m_Children;

        Vector3 m_Position;
        Quat m_Rotation;
        Vector3 m_Scale;
        Vector3 m_LocalPosition;
        Vector3 m_EulerAngles;
        Vector3 m_LocalEulerAngles;
        Vector3 m_LocalScale;
        Vector3 m_XDir;
        Vector3 m_YDir;
        Vector3 m_ZDir;
        Quat m_LocalRotation;
        Matrix4 m_WorldMatrix;
        bool m_IsDirty;
    };


}

#endif // jointnode_h__
