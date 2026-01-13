#include "jointnode.h"

namespace eba
{

    // roll-pitch-yaw, z-x-y
    static const Vector3 quatToEuler(const Quat& q)
    {
        Vector3 angles;

        // roll (z-axis rotation)
        float sinr_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
        angles.z = (glm::atan(sinr_cosp, cosr_cosp));

        // pitch (x-axis rotation)
        float sinp = 2.0f * (q.w * q.x - q.z * q.y);
        if (glm::abs(sinp) >= 1.0f)
            angles.x = (std::copysign(glm::pi<float>() / 2.0f, sinp)); // use 90 degrees if out of range
        else
            angles.x = (std::asin(sinp));

        // yaw (y-axis rotation)
        float siny_cosp = 2.0f * (q.w * q.y + q.x * q.z);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.x * q.x);
        angles.y = (glm::atan(siny_cosp, cosy_cosp));
        return glm::degrees(angles);
    }

    // roll-pitch-yaw, z-x-y
    static const Quat eularToQuat(const Vector3& e)
    {
        auto angle = glm::radians(e);

        float roll = angle.z;
        float pitch = angle.x;
        float yaw = angle.y;

        // Abbreviations for the various angular functions
        float cy = glm::cos(yaw * 0.5f);
        float sy = glm::sin(yaw * 0.5f);
        float cp = glm::cos(pitch * 0.5f);
        float sp = glm::sin(pitch * 0.5f);
        float cr = glm::cos(roll * 0.5f);
        float sr = glm::sin(roll * 0.5f);

        Quat q;
        q.x = (cr * sp * cy + sr * cp * sy);
        q.y = (cr * cp * sy - sr * sp * cy);
        q.z = (sr * cp * cy - cr * sp * sy);
        q.w = (cr * cp * cy + sr * sp * sy);

        return q;
    }

    static const Matrix4 matrix4FromTRS(const Vector3& position, const Quat& rotation, const Vector3& scale)
    {
        return glm::translate(position) * glm::toMat4(rotation) * glm::scale(scale);
    }

    JointNode::JointNode()
    {
        m_Parent = nullptr;
        m_Position = kVec3Zero;
        m_Rotation = kIdentQuat;
        m_Scale = kVec3One;
        m_EulerAngles = kVec3Zero;
        m_LocalRotation = m_Rotation;
        m_LocalPosition = m_Position;
        m_LocalScale = m_Scale;
        m_LocalEulerAngles = m_EulerAngles;
        m_ZDir = kAxisZ;
        m_YDir = kAxisY;
        m_XDir = kAxisX;
        m_IsDirty = false;
        m_WorldMatrix = kIdentMatrix4;
    }

    void JointNode::GetRelative(const Vector3& PosA, const Quat& RotA, const Vector3& ScaleA,
        const Vector3& PosB, const Quat& RotB, const Vector3& ScaleB,
        Vector3& OutPos, Quat& OutRot, Vector3& OutScale)
    {
        Vector3 SafeRecipScale3D = getSafeScaleReciprocal(ScaleB);
        OutScale = ScaleA * SafeRecipScale3D;
        Quat Inverse = glm::inverse(RotB);
        OutRot = Inverse * RotA;
        Vector3 pos = glm::rotate(Inverse, (PosA - PosB));
        OutPos = pos * SafeRecipScale3D;
    }

    void JointNode::DoTransform(const Vector3& Pos, const Quat& Rot, const Vector3& Scale,
        const Vector3& RelPos, const Quat& RelRot, const Vector3& RelScale,
        Vector3& OutPos, Quat& OutRot, Vector3& OutScale)
    {
        OutRot = Rot * RelRot;
        OutScale = Scale * RelScale;
        Vector3 delta = glm::rotate(Rot, Scale * RelPos);
        OutPos = delta + Pos;
    }

    void JointNode::UpdateLocal()
    {
        if (!m_Parent)
        {
            m_LocalRotation = m_Rotation;
            m_LocalScale = m_Scale;
            m_LocalPosition = m_Position;
            m_LocalEulerAngles = m_EulerAngles;
        }
        else
        {
            auto trans = m_Parent;
            GetRelative(m_Position, m_Rotation, m_Scale, trans->m_Position, trans->m_Rotation, trans->m_Scale, m_LocalPosition, m_LocalRotation, m_LocalScale);
            m_LocalEulerAngles = quatToEuler(m_LocalRotation);
        }
    }

    void JointNode::UpdateWorld()
    {
        if (!m_Parent)
        {
            m_Rotation = m_LocalRotation;
            m_EulerAngles = m_LocalEulerAngles;
            m_Scale = m_LocalScale;
            m_Position = m_LocalPosition;
        }
        else
        {
            auto trans = m_Parent;
            DoTransform(trans->m_Position, trans->m_Rotation, trans->m_Scale, m_LocalPosition, m_LocalRotation, m_LocalScale, m_Position, m_Rotation, m_Scale);
            m_EulerAngles = quatToEuler(m_Rotation);
        }

        UpdateDirections();
        UpdateWorldMatrix();
    }

    void JointNode::SetParent(JointNode* parent)
    {
        if (m_Parent == parent)
            return;

        if (m_Parent)
            m_Parent->RemoveChild(this);

        if (parent)
        {
            parent->AddChild(this);
            parent->CheckUpdateTree();
        }

        m_Parent = parent;

        UpdateLocal();
    }

    void JointNode::DetachChildren()
    {
        for (auto child : m_Children)
        {
            child->SetParent(nullptr);
        }
    }

    void JointNode::UpdateDirections()
    {
        m_ZDir = glm::rotate(m_Rotation, Vector3(0, 0, 1));
        m_YDir = glm::rotate(m_Rotation, Vector3(0, 1, 0));
        m_XDir = glm::rotate(m_Rotation, Vector3(1, 0, 0));
    }

    void JointNode::UpdateWorldMatrix()
    {
        m_WorldMatrix = matrix4FromTRS(m_Position, m_Rotation, m_Scale);
    }

    JointNode* JointNode::GetChild(int index)
    {
        if (index >= 0 && index < GetChildCount())
            return m_Children[index];
        else
            return nullptr;
    }

    size_t JointNode::GetChildCount() const
    {
        return m_Children.size();
    }

    Vector3 JointNode::TransformDirection(float x, float y, float z)
    {
        Vector3 dir = Vector3(x, y, z);
        float len = length(dir);
        Vector4 v = (m_WorldMatrix * Vector4(dir / len, 0.0f)) * len;
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 JointNode::TransformDirection(const Vector3& direction)
    {
        return TransformDirection(direction.x, direction.y, direction.z);
    }

    Vector3 JointNode::TransformPoint(float x, float y, float z)
    {
        Vector4 v = (m_WorldMatrix * Vector4(x, y, z, 0));
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 JointNode::TransformPoint(const Vector3& position)
    {
        Vector4 v = (m_WorldMatrix * Vector4(position, 0));
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 JointNode::InverseTransformDirection(float x, float y, float z)
    {
        Vector3 dir = Vector3(x, y, z);
        float len = length(dir);
        Vector4 v = (glm::inverse(m_WorldMatrix) * Vector4(dir / len, 0)) * len;
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 JointNode::InverseTransformDirection(const Vector3& direction)
    {
        return InverseTransformDirection(direction.x, direction.y, direction.z);
    }

    Vector3 JointNode::InverseTransformPoint(float x, float y, float z)
    {
        Vector4 v = (glm::inverse(m_WorldMatrix) * Vector4(x, y, z, 1.0f));
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 JointNode::InverseTransformPoint(const Vector3& position)
    {
        Vector4 v = (glm::inverse(m_WorldMatrix) * Vector4(position, 1.0f));
        return Vector3(v.x, v.y, v.z);
    }

    bool JointNode::IsChildOf(JointNode* parent)
    {
        if (!parent || !m_Parent)
            return false;
        else if (parent == m_Parent)
            return true;
        else
            return m_Parent->IsChildOf(parent);
    }

    void JointNode::Rotate(const Quat& quat, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        if (relativeTo == TransformSpace::Local)
        {
            SetRotation(GetRotation() * quat);
        }
        else
        {
            SetRotation(quat * GetRotation());
        }
    }

    void JointNode::Rotate(const Vector3& eulers, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        if (relativeTo == TransformSpace::Local)
        {
            SetRotation(GetRotation() * eularToQuat(eulers));
        }
        else
        {
            SetRotation(eularToQuat(eulers) * GetRotation());
        }
    }

    void JointNode::Rotate(float xAngle, float yAngle, float zAngle, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        Vector3 eulers(xAngle, yAngle, zAngle);
        if (relativeTo == TransformSpace::Local)
        {
            SetRotation(GetRotation() * eularToQuat(eulers));
        }
        else
        {
            SetRotation(eularToQuat(eulers) * GetRotation());
        }
    }

    void JointNode::Rotate(const Vector3& axis, float angle, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        Quat rot(glm::radians(angle), axis);
        if (relativeTo == TransformSpace::Local)
        {
            SetRotation(GetRotation() * rot);
        }
        else
        {
            SetRotation(rot * GetRotation());
        }
    }

    void JointNode::SetPositionAndRotation(const Vector3& position, Quat rotation)
    {
        SetPosition(position);
        SetRotation(rotation);
    }

    void JointNode::CopyPositionAndRotation(JointNode* other)
    {
        SetPosition(other->GetPosition());
        SetRotation(other->GetRotation());
    }

    void JointNode::CopyTransform(JointNode* other)
    {
        SetPosition(other->GetPosition());
        SetRotation(other->GetRotation());
        SetScale(other->GetScale());
    }

    void JointNode::Translate(float x, float y, float z, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        Translate(Vector3(x, y, z), relativeTo);
    }

    void JointNode::Translate(const Vector3& translation, TransformSpace relativeTo /*= TransformSpace::Local*/)
    {
        if (relativeTo == TransformSpace::Parent)
        {
            SetLocalPosition(GetLocalPosition() + translation);
        }
        else if (relativeTo == TransformSpace::World)
        {
            SetPosition(GetPosition() + translation);
        }
        else if (relativeTo == TransformSpace::Local)
        {
            SetPosition(GetPosition() + GetRotation() * translation);
        }
    }

    const Vector3 JointNode::GetDirX() const
    {
        CheckUpdateTree();
        return m_XDir;
    }

    const Vector3 JointNode::GetDirY() const
    {
        CheckUpdateTree();
        return m_YDir;
    }

    const Vector3 JointNode::GetDirZ() const
    {
        CheckUpdateTree();
        return m_ZDir;
    }

    void JointNode::SetLocalPosition(const Vector3& position)
    {
        if (nearlyEqual(position, m_LocalPosition))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
        }

        m_LocalPosition = position;
        MakeDirty();
    }

    void JointNode::SetLocalPosition(float x, float y, float z)
    {
        SetLocalPosition(Vector3(x, y, z));
    }

    void JointNode::SetLocalEulerAngles(const Vector3& euler)
    {
        if (nearlyEqual(euler, m_LocalEulerAngles))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
        }

        m_LocalEulerAngles = euler;
        m_LocalRotation = eularToQuat(euler);
        MakeDirty();
    }

    void JointNode::SetLocalEulerAngles(float pitch, float yaw, float roll)
    {
        SetLocalEulerAngles(Vector3(pitch, yaw, roll));
    }

    void JointNode::SetLocalRotation(const Quat& quat)
    {
        if (nearlyEqual(quat, m_LocalRotation))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
        }

        m_LocalRotation = quat;
        m_LocalEulerAngles = quatToEuler(m_LocalRotation);
        MakeDirty();
    }

    void JointNode::SetLocalRotation(float x, float y, float z)
    {
        SetLocalRotation(Vector3(x, y, z));
    }

    void JointNode::SetLocalScale(const Vector3& scale)
    {
        if (nearlyEqual(scale, m_LocalScale))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
        }

        m_LocalScale = scale;
        MakeDirty();
    }

    void JointNode::SetRotation(const Quat& quat)
    {
        if (nearlyEqual(quat, m_Rotation))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
            m_Rotation = quat;
            m_EulerAngles = quatToEuler(m_Rotation);
            m_LocalRotation = inverse(m_Parent->m_Rotation) * m_Rotation;
            m_LocalEulerAngles = quatToEuler(m_LocalRotation);
        }
        else
        {
            m_Rotation = quat;
            m_EulerAngles = quatToEuler(m_Rotation);
            m_LocalRotation = m_Rotation;
            m_LocalEulerAngles = m_EulerAngles;
        }

        MakeDirty();
    }

    const Quat& JointNode::GetRotation() const
    {
        CheckUpdateTree();
        return m_Rotation;
    }

    void JointNode::SetEulerAngles(const Vector3& euler)
    {
        if (nearlyEqual(euler, m_EulerAngles))
            return;

        if (m_Parent)
        {
            m_Parent->CheckUpdateTree();
            m_Rotation = eularToQuat(euler);
            m_EulerAngles = euler;
            m_LocalRotation = glm::inverse(m_Parent->GetRotation()) * m_Rotation;
            m_LocalEulerAngles = euler;
        }
        else
        {
            m_Rotation = eularToQuat(euler);
            m_EulerAngles = euler;
            m_LocalRotation = m_Rotation;
            m_LocalEulerAngles = m_EulerAngles;
        }

        MakeDirty();
    }

    void JointNode::SetEulerAngles(float pitch, float yaw, float roll)
    {
        SetEulerAngles(Vector3(pitch, yaw, roll));
    }

    const Vector3& JointNode::GetEulerAngles() const
    {
        CheckUpdateTree();
        return m_EulerAngles;
    }

    void JointNode::SetPosition(const Vector3& pos)
    {
        if (nearlyEqual(pos, m_Position))
            return;

        if (m_Parent)
        {
            auto trans = m_Parent;
            trans->CheckUpdateTree();
            Vector3 SafeRecipScale3D = getSafeScaleReciprocal(trans->m_Scale);
            Quat Inverse = glm::inverse(trans->m_Rotation);
            m_Position = pos;
            m_LocalPosition = glm::rotate(Inverse, (m_Position - trans->m_Position)) * SafeRecipScale3D;
        }
        else
        {
            m_Position = pos;
            m_LocalPosition = pos;
        }
        MakeDirty();
    }

    void JointNode::SetPosition(float x, float y, float z)
    {
        SetPosition(Vector3(x, y, z));
    }

    const Vector3& JointNode::GetPosition() const
    {
        CheckUpdateTree();
        return m_Position;
    }

    void JointNode::SetScale(const Vector3& sc)
    {
        if (nearlyEqual(sc, m_Scale))
            return;

        if (m_Parent)
        {
            auto trans = m_Parent;
            trans->CheckUpdateTree();
            m_Scale = sc;
            Vector3 SafeRecipScale3D = getSafeScaleReciprocal(trans->m_Scale);
            m_LocalScale = m_Scale * SafeRecipScale3D;
        }
        else
        {
            m_Scale = sc;
            m_LocalScale = m_Scale;
        }
        MakeDirty();
    }

    void JointNode::SetScale(float value)
    {
        SetScale(Vector3(value));
    }

    void JointNode::SetScale(float x, float y, float z)
    {
        SetScale(Vector3(x, y, z));
    }

    const Vector3& JointNode::GetScale() const
    {
        CheckUpdateTree();
        return m_Scale;
    }

    const Matrix4& JointNode::GetWorldMatrix() const
    {
        CheckUpdateTree();
        return m_WorldMatrix;
    }

    void JointNode::SetWorldMatrix(const Matrix4& m)
    {
        Vector3 pos, scale, skew;
        Quat rot;
        Vector4 perspective;
        glm::decompose(m, scale, rot, pos, skew, perspective);
        SetPosition(pos);
        SetRotation(rot);
        SetScale(scale);
    }

    void JointNode::SetLocalIdentity()
    {
        SetLocalScale(Vector3(1));
        SetLocalRotation(kIdentQuat);
        SetLocalPosition(Vector3(0));
    }

    int JointNode::GetHierarchyDeep()
    {
        if (!m_Parent)
        {
            return 0;
        }

        auto result = this;
        int deep = 0;
        while (result->GetParent())
        {
            result = result->GetParent();
            ++deep;
        }

        return deep;
    }

    void JointNode::AddChild(JointNode* child)
    {
        m_Children.push_back(child);
    }

    void JointNode::RemoveChild(JointNode* child)
    {
        for (auto iter = m_Children.begin(); iter != m_Children.end(); ++iter)
        {
            if (*iter == child)
            {
                m_Children.erase(iter);
                break;
            }
        }
    }

    void JointNode::MakeDirty()
    {
        m_IsDirty = true;
        for (auto child : m_Children)
        {
            auto trans = child;
            trans->m_IsDirty = true;
            trans->MakeDirty();
        }
    }

    void JointNode::CheckUpdateTree() const
    {
        if (m_IsDirty)
        {
            auto ancestor = GetDirtyAncestor();
            if (ancestor)
            {
                const_cast<JointNode*>(ancestor)->UpdateTree();
            }
        }
    }

    void JointNode::UpdateTree()
    {
        if (m_IsDirty)
        {
            UpdateWorld();
            m_IsDirty = false;
        }

        for (auto child : m_Children)
        {
            child->UpdateTree();
        }
    }

    const JointNode* JointNode::GetDirtyAncestor() const
    {
        const JointNode* result = this;
        if (!m_Parent && m_IsDirty)
        {
            return result;
        }

        while (result->GetParent() && result->GetParent()->m_IsDirty)
        {
            result = result->GetParent();
        }

        if (!result->m_IsDirty)
        {
            return nullptr;
        }

        return result;
    }

    void JointNode::Reset()
    {
        SetLocalPosition(Vector3(0));
        SetLocalRotation(kIdentQuat);
        SetLocalScale(Vector3(1));
    }

}