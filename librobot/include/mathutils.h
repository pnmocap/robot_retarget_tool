#ifndef mathutils_h__
#define mathutils_h__

#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/ext/quaternion_double_precision.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/gtx/extended_min_max.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_query.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

namespace eba
{
    typedef glm::vec2 Vector2;
    typedef glm::vec3 Vector3;
    typedef glm::vec4 Vector4;
    typedef glm::mat3 Matrix3;
    typedef glm::mat4 Matrix4;
    typedef glm::highp_quat Quat;

    inline constexpr Vector2 kVec2Zero = Vector2(0.0f);
    inline constexpr Vector3 kVec3Zero = Vector3(0.0f);
    inline constexpr Vector4 kVec4Zero = Vector4(0.0f);
    inline constexpr Vector2 kVec2One = Vector2(1.0f);
    inline constexpr Vector3 kVec3One = Vector3(1.0f);
    inline constexpr Vector4 kVec4One = Vector4(1.0f);
    inline constexpr Vector3 kAxisX = Vector3(1.0f, 0.0f, 0.0f);
    inline constexpr Vector3 kAxisY = Vector3(0.0f, 1.0f, 0.0f);
    inline constexpr Vector3 kAxisZ = Vector3(0.0f, 0.0f, 1.0f);
    inline constexpr Quat kIdentQuat = glm::identity<Quat>();
    inline constexpr Matrix3 kIdentMatrix3 = glm::identity<Matrix3>();
    inline constexpr Matrix4 kIdentMatrix4 = glm::identity<Matrix4>();

#define SMALL_NUMBER (1.e-8f)
#define KINDA_SMALL_NUMBER (1.e-4f)

    inline bool nearlyEqual(float A, float B)
    {
        return glm::abs(A - B) <= SMALL_NUMBER;
    }

    inline bool nearlyEqual(const Vector3& A, const Vector3& B)
    {
        return glm::all(glm::lessThanEqual(glm::abs(A - B), Vector3(SMALL_NUMBER)));
    }

    inline bool nearlyEqual(const Vector4& A, const Vector4& B)
    {
        return glm::all(glm::lessThanEqual(glm::abs(A - B), Vector4(SMALL_NUMBER)));
    }

    inline bool nearlyEqual(const Quat& A, const Quat& B)
    {
        return (nearlyEqual(A.x, B.x) && nearlyEqual(A.y, B.y) && nearlyEqual(A.z, B.z) && nearlyEqual(A.w, B.w))
            || (nearlyEqual(-A.x, B.x) && nearlyEqual(-A.y, B.y) && nearlyEqual(-A.z, B.z) && nearlyEqual(-A.w, B.w));
    }

    struct EulerOrder
    {
        int32_t axis1;
        int32_t axis2;
        int32_t axis3;

        bool operator==(const EulerOrder& other) const
        {
            return axis1 == other.axis1 && axis2 == other.axis2 && axis3 == other.axis3;
        }
    };

    struct EulerAngle
    {
        float angle[3];

        EulerAngle(float x, float y, float z)
        {
            angle[0] = x;
            angle[1] = y;
            angle[2] = z;
        }

        const float operator[](int index) const
        {
            return angle[index];
        }

        float& operator[](int index)
        {
            return angle[index];
        }
    };

    Quat EulerToQuat(const EulerAngle& angle, const EulerOrder& order);

    EulerAngle Mat3ToEular(const Matrix3& m, const EulerOrder& order = { 0, 1, 2 });

    EulerAngle QuatToEuler(const Quat& q, const EulerOrder& order = { 0, 1, 2 });

    float angleBetween(const Vector3& u, const Vector3& v);

    float angleBetween(const Vector3& v1, const Vector3& v2, const Vector3& normal);

    void FindBestAxisVectors(const Vector3& A, Vector3& Axis1, Vector3& Axis2);

    Vector3 vecProjectOnPlane(const Vector3& v, const Vector3& n);

    Vector3 getSafeScaleReciprocal(const Vector3& InScale);

    float ClampAngleRad(float AngleRad, float MinAngleRad, float MaxAngleRad);

    float ClampAxisRad(float AngleRad);

    inline float Angle360To180(float v)
    {
        // 消除绕360
        if (v > glm::radians(180.0f))
        {
            v -= glm::radians(360.0f);
        }
        else if (v < -glm::radians(180.0f))
        {
            v += glm::radians(360.0f);
        }
        return v;
    }

    inline float CalculXzPlanAngle(Quat qua) 
    {
        glm::vec3 forward = glm::vec3(0.0f, 0.0f, 1.0f);
        //计算对应的旋转向量，得到单位旋转向量
        glm::vec3 targetPos =  forward*qua;
        //计算对应YZ屏幕投影
        float angle = std::atan(targetPos.y / targetPos.z);
        return angle;
    }

    inline float CalculXyPlanAngle(Quat qua)
    {
        glm::vec3 forward = glm::vec3(1.0f, 0.0f, 0.0f);
        //计算对应的旋转向量，得到单位旋转向量
        glm::vec3 targetPos = forward * qua;
        //计算对应YZ屏幕投影
        float angle = std::atan(targetPos.y / targetPos.x);
        return angle;
    }

    inline float CalculElbowAngle(bool isLeft, const glm::quat& qua) {
        // 前向向量（假设为左臂/右臂的初始方向）
        glm::vec3 forward = glm::vec3(0.0f, 0.0f, 1.0f);

        // 计算旋转后的方向向量（注意乘法顺序）
        glm::vec3 rotatedForward = forward* qua;

        // 提取X-Y平面投影
        glm::vec2 projection = glm::vec2(rotatedForward.x, rotatedForward.y);

        // 计算基础角度（绕Z轴）
        float angle = std::atan2(projection.y, projection.x);
        //angle = glm::degrees(angle);

        if (projection.x<0)
        {
            angle = 3.14f - angle;
        }
        //// 根据X分量修正象限（判断方向）


        return angle;
    }



    inline float CalculWristAngle(bool isLeft, Quat qua)
    {
        glm::vec3 forward = glm::vec3(.0f, 0.0f, -1.0f);
        //计算对应的旋转向量，得到单位旋转向量
        glm::vec3 targetPos = qua * forward;
        //计算对应YZ屏幕投影
        float angle = std::asin(std::abs(targetPos.y) / std::sqrt(targetPos.y * targetPos.y + targetPos.x * targetPos.x));
        // float angle = std::atan(targetPos.y / targetPos.z);
        if (targetPos.x < 0)
        {
            angle = 3.1415f - angle;
        }

        if (isLeft)
        {
            if (targetPos.y > 0)
            {
                angle = -angle;
            }
        }
        else
        {
            if (targetPos.y < 0)
            {
                angle = -angle;
            }
        }
        return angle;
    }


    inline float Clamp90(float v)
    {
        return glm::clamp(v, -glm::radians(89.0f), glm::radians(89.0f));
    }

    inline float Clamp180(float v)
    {
        return glm::clamp(v, -glm::radians(179.0f), glm::radians(179.0f));
    }

    Matrix3 matrixFromXY(Vector3 const& XAxis, Vector3 const& YAxis);
    Matrix3 matrixFromXZ(Vector3 const& XAxis, Vector3 const& ZAxis);
    Matrix3 matrixFromYZ(Vector3 const& YAxis, Vector3 const& ZAxis);
    Quat quatFromXY(Vector3 const& XAxis, Vector3 const& YAxis);
    Quat quatFromYZ(Vector3 const& YAxis, Vector3 const& ZAxis);
    Quat quatFromXZ(Vector3 const& XAxis, Vector3 const& ZAxis);

    float QuatDistance(const Quat& A, const Quat& B);

    Vector3 VInterpNormalRotationTo(const Vector3& Current, const Vector3& Target, float DeltaTime, float RotationSpeedRad);
    Vector3 VInterpConstantTo(const Vector3& Current, const Vector3& Target, float DeltaTime, float InterpSpeed);
    Vector3 VInterpTo(const Vector3& current, const Vector3& target, float deltaTime, float interpSpeed);
    Quat QInterpTo(const Quat& Current, const Quat& Target, float DeltaTime, float InterpSpeed);

}

#endif // mathutils_h__
