#include "mathutils.h"

namespace eba
{

    Quat EulerToQuat(const EulerAngle& angle, const EulerOrder& order)
    {
        static const Vector3 axis_value[] =
        {
            Vector3(1, 0, 0),
            Vector3(0, 1, 0),
            Vector3(0, 0, 1)
        };

        Quat e_q =
            glm::angleAxis(angle[0], axis_value[order.axis1]) *
            glm::angleAxis(angle[1], axis_value[order.axis2]) *
            glm::angleAxis(angle[2], axis_value[order.axis3]);

        return e_q;
    }

    static float coeff(const Matrix3& m, const uint32_t i, const uint32_t j)
    {
        return m[i][j];
    }

    static float& coeffRef(Matrix3& m, const uint32_t i, const uint32_t j)
    {
        return m[i][j];
    }

    EulerAngle Mat3ToEular(const Matrix3& m, const EulerOrder& order /*= { 0, 1, 2 }*/)
    {
        uint32_t a0 = order.axis1;
        uint32_t a1 = order.axis2;
        uint32_t a2 = order.axis3;

        Vector3 res;

        const uint32_t odd = ((a0 + 1) % 3 == a1) ? 0 : 1;
        const uint32_t i = a0;
        const uint32_t j = (a0 + 1 + odd) % 3;
        const uint32_t k = (a0 + 2 - odd) % 3;

        if (a0 == a2)
        {
            res[0] = atan2(coeff(m, j, i), coeff(m, k, i));
            if ((odd && res[0] < (0)) || ((!odd) && res[0] > float(0)))
            {
                if (res[0] > float(0)) {
                    res[0] -= float(glm::pi<float>());
                }
                else {
                    res[0] += float(glm::pi<float>());
                }
                float s2 = glm::length(Vector2(coeff(m, j, i), coeff(m, k, i)));
                res[1] = -atan2(s2, coeff(m, i, i));
            }
            else
            {
                float s2 = glm::length(Vector2(coeff(m, j, i), coeff(m, k, i)));
                res[1] = atan2(s2, coeff(m, i, i));
            }

            // With a=(0,1,0), we have i=0; j=1; k=2, and after computing the first two angles,
            // we can compute their respective rotation, and apply its inverse to M. Since the result must
            // be a rotation around x, we have:
            //
            //  c2  s1.s2 c1.s2                   1  0   0 
            //  0   c1    -s1       *    M    =   0  c3  s3
            //  -s2 s1.c2 c1.c2                   0 -s3  c3
            //
            //  Thus:  m11.c1 - m21.s1 = c3  &   m12.c1 - m22.s1 = s3

            float s1 = sin(res[0]);
            float c1 = cos(res[0]);
            res[2] = atan2(c1 * coeff(m, j, k) - s1 * coeff(m, k, k), c1 * coeff(m, j, j) - s1 * coeff(m, k, j));
        }
        else
        {
            res[0] = atan2(coeff(m, j, k), coeff(m, k, k));
            float c2 = glm::length(Vector2(coeff(m, i, i), coeff(m, i, j)));
            if ((odd && res[0] < float(0)) || ((!odd) && res[0] > float(0))) {
                if (res[0] > float(0)) {
                    res[0] -= float(glm::pi<float>());
                }
                else {
                    res[0] += float(glm::pi<float>());
                }
                res[1] = atan2(-coeff(m, i, k), -c2);
            }
            else
                res[1] = atan2(-coeff(m, i, k), c2);
            float s1 = sin(res[0]);
            float c1 = cos(res[0]);
            res[2] = atan2(s1 * coeff(m, k, i) - c1 * coeff(m, j, i), c1 * coeff(m, j, j) - s1 * coeff(m, k, j));
        }
        if (!odd)
            res = -res;

        return EulerAngle(res.x, res.y, res.z);
    }

    EulerAngle QuatToEuler(const Quat& q, const EulerOrder& order /*= { 0, 1, 2 }*/)
    {
        // NOTE if inlined, then gcc 4.2 and 4.4 get rid of the temporary (not gcc 4.3 !!)
        // if not inlined then the cost of the return by value is huge ~ +35%,
        // however, not inlining this function is an order of magnitude slower, so
        // it has to be inlined, and so the return by value is not an issue
        Matrix3 res;

        const float tx = 2.0 * q.x;
        const float ty = 2.0 * q.y;
        const float tz = 2.0 * q.z;
        const float twx = tx * q.w;
        const float twy = ty * q.w;
        const float twz = tz * q.w;
        const float txx = tx * q.x;
        const float txy = ty * q.x;
        const float txz = tz * q.x;
        const float tyy = ty * q.y;
        const float tyz = tz * q.y;
        const float tzz = tz * q.z;

        coeffRef(res, 0, 0) = 1.0 - (tyy + tzz);
        coeffRef(res, 0, 1) = txy - twz;
        coeffRef(res, 0, 2) = txz + twy;
        coeffRef(res, 1, 0) = txy + twz;
        coeffRef(res, 1, 1) = 1.0 - (txx + tzz);
        coeffRef(res, 1, 2) = tyz - twx;
        coeffRef(res, 2, 0) = txz - twy;
        coeffRef(res, 2, 1) = tyz + twx;
        coeffRef(res, 2, 2) = 1.0 - (txx + tyy);

        return Mat3ToEular(res, order);
    }

    float angleBetween(const Vector3& u, const Vector3& v)
    {
        float AngleCos = glm::dot(glm::normalize(u), glm::normalize(v));
        AngleCos = glm::clamp(AngleCos, -1.0f, 1.0f);
        return glm::acos(AngleCos);
    }

    float angleBetween(const Vector3& v1, const Vector3& v2, const Vector3& normal)
    {
        // 计算点积
        float dot = glm::dot(v1, v2);

        // 计算夹角（无方向）
        float angle = glm::acos(dot / (glm::length(v1) * glm::length(v2)));

        // 计算叉积
        Vector3 cross = glm::cross(v1, v2);

        // 通过叉积和normal确定夹角的方向
        if (glm::dot(cross, normal) < 0) {
            angle = -angle;
        }

        // 返回角度（带方向）
        return angle;
    }

    Vector3 vecProjectOnPlane(const Vector3& v, const Vector3& n) 
    {
        // 计算法线方向的分量
        Vector3 proj_on_n = glm::dot(v, n) / glm::dot(n, n) * n;

        // 计算在平面上的投影
        Vector3 proj_on_plane = v - proj_on_n;

        return proj_on_plane;
    }

    float ClampAxis(float Angle)
    {
        // returns Angle in the range (-360,360)
        Angle = fmod(Angle, 360.0);

        if (Angle < 0.0)
        {
            // shift to [0,360) range
            Angle += 360.0;
        }

        return Angle;
    }

    float ClampAxisRad(float AngleRad)
    {
        // returns Angle in the range (-2PI,2PI)
        AngleRad = fmod(AngleRad, glm::two_pi<float>());
        if (AngleRad < 0.0)
        {
            // shift to [0,2PI) range
            AngleRad += glm::two_pi<float>();
        }
        return AngleRad;
    }

    float NormalizeAxis(float Angle)
    {
        // returns Angle in the range [0,360)
        Angle = ClampAxis(Angle);

        if (Angle > 180.0)
        {
            // shift to (-180,180]
            Angle -= 360.0;
        }

        return Angle;
    }

    float ClampAngle(float AngleDegrees, float MinAngleDegrees, float MaxAngleDegrees)
    {
        float const MaxDelta = ClampAxis(MaxAngleDegrees - MinAngleDegrees) * 0.5f;			// 0..180
        float const RangeCenter = ClampAxis(MinAngleDegrees + MaxDelta);						// 0..360
        float const DeltaFromCenter = NormalizeAxis(AngleDegrees - RangeCenter);				// -180..180

        // maybe clamp to nearest edge
        if (DeltaFromCenter > MaxDelta)
        {
            return NormalizeAxis(RangeCenter + MaxDelta);
        }
        else if (DeltaFromCenter < -MaxDelta)
        {
            return NormalizeAxis(RangeCenter - MaxDelta);
        }

        // already in range, just return it
        return NormalizeAxis(AngleDegrees);
    }

    float ClampAngleRad(float AngleRad, float MinAngleRad, float MaxAngleRad)
    {
        float AngleDegrees = glm::degrees(AngleRad);
        float MinAngleDegrees = glm::degrees(MinAngleRad);
        float MaxAngleDegrees = glm::degrees(MaxAngleRad);

        float const MaxDelta = ClampAxis(MaxAngleDegrees - MinAngleDegrees) * 0.5f;			// 0..180
        float const RangeCenter = ClampAxis(MinAngleDegrees + MaxDelta);						// 0..360
        float const DeltaFromCenter = NormalizeAxis(AngleDegrees - RangeCenter);				// -180..180

        // maybe clamp to nearest edge
        if (DeltaFromCenter > MaxDelta)
        {
            return NormalizeAxis(RangeCenter + MaxDelta);
        }
        else if (DeltaFromCenter < -MaxDelta)
        {
            return NormalizeAxis(RangeCenter - MaxDelta);
        }

        // already in range, just return it
        return glm::radians(NormalizeAxis(AngleDegrees));
    }

    void FindBestAxisVectors(const Vector3& A, Vector3& Axis1, Vector3& Axis2)
    {
        const float NX = fabsf(A.x);
        const float NY = fabsf(A.y);
        const float NZ = fabsf(A.z);

        // Find best basis vectors.
        if (NZ > NX && NZ > NY)	Axis1 = Vector3(1, 0, 0);
        else					Axis1 = Vector3(0, 0, 1);

        Axis1 = glm::normalize(Axis1 - A * glm::dot(Axis1, A));
        Axis2 = glm::cross(Axis1, A);
    }

    Vector3 getSafeScaleReciprocal(const Vector3& InScale)
    {
        Vector3 SafeReciprocalScale;
        if (std::fabs(InScale.x) <= SMALL_NUMBER)
        {
            SafeReciprocalScale.x = 0;
        }
        else
        {
            SafeReciprocalScale.x = (1.0 / InScale.x);
        }

        if (std::fabs(InScale.y) <= SMALL_NUMBER)
        {
            SafeReciprocalScale.y = (0);
        }
        else
        {
            SafeReciprocalScale.y = (1.0 / InScale.y);
        }

        if (std::fabs(InScale.z) <= SMALL_NUMBER)
        {
            SafeReciprocalScale.z = (0);
        }
        else
        {
            SafeReciprocalScale.z = (1.0 / InScale.z);
        }

        return SafeReciprocalScale;
    }

    Matrix3 matrixFromXY(Vector3 const& XAxis, Vector3 const& YAxis)
    {
        Vector3 NewX = normalize(XAxis);
        Vector3 Norm = normalize(YAxis);

        // if they're almost same, we need to find arbitrary vector
        if (nearlyEqual(fabs(dot(NewX, Norm)), 1.0))
        {
            // make sure we don't ever pick the same as NewX
            Norm = (fabs(NewX.z) < (1.0 - KINDA_SMALL_NUMBER)) ? Vector3(0, 0, 1.0) : Vector3(1.0, 0, 0);
        }

        const Vector3 NewZ = normalize(cross(NewX, Norm));
        const Vector3 NewY = cross(NewZ, NewX);

        return Matrix3(NewX, NewY, NewZ);
    }

    Matrix3 matrixFromXZ(Vector3 const& XAxis, Vector3 const& ZAxis)
    {
        Vector3 const NewX = normalize(XAxis);
        Vector3 Norm = normalize(ZAxis);

        // if they're almost same, we need to find arbitrary vector
        if (nearlyEqual(fabs(dot(NewX, Norm)), 1.0))
        {
            // make sure we don't ever pick the same as NewX
            Norm = (fabs(NewX.z) < (1.0 - KINDA_SMALL_NUMBER)) ? Vector3(0, 0, 1.0) : Vector3(1.0, 0, 0);
        }

        const Vector3 NewY = normalize(cross(Norm, NewX));
        const Vector3 NewZ = cross(NewX, NewY);

        return Matrix3(NewX, NewY, NewZ);
    }

    Matrix3 matrixFromYZ(Vector3 const& YAxis, Vector3 const& ZAxis)
    {
        Vector3 const NewY = normalize(YAxis);;
        Vector3 Norm = normalize(ZAxis);

        // if they're almost same, we need to find arbitrary vector
        if (nearlyEqual(fabs(dot(NewY, Norm)), 1.0))
        {
            // make sure we don't ever pick the same as NewX
            Norm = (fabs(NewY.z) < (1.0 - KINDA_SMALL_NUMBER)) ? Vector3(0, 0, 1.0) : Vector3(1.0, 0, 0);
        }

        const Vector3 NewX = normalize(cross(NewY, Norm));;
        const Vector3 NewZ = cross(NewX, NewY);

        return Matrix3(NewX, NewY, NewZ);
    }

    Quat quatFromXY(Vector3 const& XAxis, Vector3 const& YAxis)
    {
        return Quat(matrixFromXY(XAxis, YAxis));
    }

    Quat quatFromYZ(Vector3 const& YAxis, Vector3 const& ZAxis)
    {
        return Quat(matrixFromYZ(YAxis, ZAxis));
    }

    Quat quatFromXZ(Vector3 const& XAxis, Vector3 const& ZAxis)
    {
        return Quat(matrixFromXZ(XAxis, ZAxis));
    }

    float QuatDistance(const Quat& A, const Quat& B)
    {
        float InnerProd = A.x * B.x + A.y * B.y + A.z * B.z + A.w * B.w;
        return glm::acos((2 * InnerProd * InnerProd) - 1.f);
    }

    Vector3 VInterpNormalRotationTo(const Vector3& Current, const Vector3& Target, float DeltaTime, float RotationSpeedRad)
    {
        // Find delta rotation between both normals.
        Quat DeltaQuat = glm::rotation(Current, Target);

        // Decompose into an axis and angle for rotation
        Vector3 DeltaAxis(0.f);
        float DeltaAngle = 0.f;
        DeltaAxis = glm::axis(DeltaQuat);
        DeltaAngle = glm::angle(DeltaQuat);

        // Find rotation step for this frame
        const float RotationStepRadians = RotationSpeedRad * DeltaTime;

        if (glm::abs(DeltaAngle) > RotationStepRadians)
        {
            DeltaAngle = glm::clamp<float>(DeltaAngle, -RotationStepRadians, RotationStepRadians);
            DeltaQuat = glm::angleAxis(DeltaAngle, DeltaAxis);
            return DeltaQuat * Current;
        }
        return Target;
    }

    Vector3 VInterpConstantTo(const Vector3& Current, const Vector3& Target, float DeltaTime, float InterpSpeed)
    {
        const Vector3 Delta = Target - Current;
        const float DeltaM = glm::length(Delta);
        const float MaxStep = InterpSpeed * DeltaTime;

        if (DeltaM > MaxStep)
        {
            if (MaxStep > 0.f)
            {
                const Vector3 DeltaN = Delta / DeltaM;
                return Current + DeltaN * MaxStep;
            }
            else
            {
                return Current;
            }
        }

        return Target;
    }

    Vector3 VInterpTo(const Vector3& Current, const Vector3& Target, float DeltaTime, float InterpSpeed)
    {
        // If no interp speed, jump to target value
        if (InterpSpeed <= 0.f)
        {
            return Target;
        }

        // Distance to reach
        const Vector3 Dist = Target - Current;

        // If distance is too small, just set the desired location
        if (glm::length2(Dist) < KINDA_SMALL_NUMBER)
        {
            return Target;
        }

        // Delta Move, Clamp so we do not over shoot.
        const Vector3 DeltaMove = Dist * glm::clamp<float>(DeltaTime * InterpSpeed, 0.f, 1.f);

        return Current + DeltaMove;
    }

    Quat QInterpConstantTo(const Quat& Current, const Quat& Target, float DeltaTime, float InterpSpeed)
    {
        // If no interp speed, jump to target value
        if (InterpSpeed <= 0.f)
        {
            return Target;
        }

        // If the values are nearly equal, just return Target and assume we have reached our destination.
        if (nearlyEqual(Current, Target))
        {
            return Target;
        }

        float DeltaInterpSpeed = glm::clamp<float>(DeltaTime * InterpSpeed, 0.f, 1.f);
        float AngularDistance = glm::max<float>(SMALL_NUMBER, QuatDistance(Target, Current));
        float Alpha = glm::clamp<float>(DeltaInterpSpeed / AngularDistance, 0.f, 1.f);

        return glm::slerp(Current, Target, Alpha);
    }

    Quat QInterpTo(const Quat& Current, const Quat& Target, float DeltaTime, float InterpSpeed)
    {
        // If no interp speed, jump to target value
        if (InterpSpeed <= 0.f)
        {
            return Target;
        }

        // If the values are nearly equal, just return Target and assume we have reached our destination.
        if (nearlyEqual(Current, Target))
        {
            return Target;
        }

        return glm::slerp(Current, Target, glm::clamp<float>(InterpSpeed * DeltaTime, 0.f, 1.f));
    }


//     glm::vec3 euler_angle_modification(const glm::vec3& angles, const glm::vec3& last_angles) {
//         float pitch = angles.x;
//         float last_pitch = last_angles.x;
//         float roll = angles.y;
//         float last_roll = last_angles.y;
//         float yaw = angles.z;
//         float last_yaw = last_angles.z;
// 
//         // A. Resolve discontinuity due to Gimbal Lock when roll around +/- 90 degrees
//         float gl_tol = 30.0f / 180.0f * glm::pi<float>(); // +/- 30 degrees region to enter Gimbal Lock process
//         glm::vec3 result = angles;
// 
//         if (std::abs(roll - glm::half_pi<float>()) <= gl_tol) {
//             // Around +pi/2
//             float delta = yaw - pitch;
//             float roll_flip = glm::pi<float>() - roll;
//             float pitch_flip_1 = pitch - glm::pi<float>();
//             float pitch_flip_2 = pitch + glm::pi<float>();
// 
//             float pitch_diff, pitch_flip;
//             if (std::abs(pitch_flip_1 - last_pitch) < std::abs(pitch_flip_2 - last_pitch)) {
//                 pitch_diff = std::abs(pitch_flip_1 - last_pitch);
//                 pitch_flip = pitch_flip_1;
//             }
//             else {
//                 pitch_diff = std::abs(pitch_flip_2 - last_pitch);
//                 pitch_flip = pitch_flip_2;
//             }
// 
//             float yaw_flip = delta + pitch_flip;
//             if (pitch_diff < std::abs(pitch - last_pitch)) {
//                 result = glm::vec3(pitch_flip, roll_flip, yaw_flip);
//             }
//         }
//         else if (std::abs(roll + glm::half_pi<float>()) <= gl_tol) {
//             // Around -pi/2
//             float delta = yaw + pitch;
//             float roll_flip = -glm::pi<float>() - roll;
//             float pitch_flip_1 = pitch - glm::pi<float>();
//             float pitch_flip_2 = pitch + glm::pi<float>();
// 
//             float pitch_diff, pitch_flip;
//             if (std::abs(pitch_flip_1 - last_pitch) < std::abs(pitch_flip_2 - last_pitch)) {
//                 pitch_diff = std::abs(pitch_flip_1 - last_pitch);
//                 pitch_flip = pitch_flip_1;
//             }
//             else {
//                 pitch_diff = std::abs(pitch_flip_2 - last_pitch);
//                 pitch_flip = pitch_flip_2;
//             }
// 
//             float yaw_flip = delta - pitch_flip;
//             if (pitch_diff < std::abs(pitch - last_pitch)) {
//                 result = glm::vec3(pitch_flip, roll_flip, yaw_flip);
//             }
//         }
// 
//         // B. Resolve discontinuity due to +/- 2*pi
//         pitch = result.x;
//         roll = result.y;
//         yaw = result.z;
// 
//         // Handle pitch
//         float pitch_modi1 = pitch - 2.0f * glm::pi<float>();
//         float pitch_modi2 = pitch + 2.0f * glm::pi<float>();
//         if (std::abs(pitch_modi1 - last_pitch) <= std::abs(pitch_modi2 - last_pitch)) {
//             if (std::abs(pitch_modi1 - last_pitch) < std::abs(pitch - last_pitch)) {
//                 pitch = pitch_modi1;
//             }
//         }
//         else {
//             if (std::abs(pitch_modi2 - last_pitch) < std::abs(pitch - last_pitch)) {
//                 pitch = pitch_modi2;
//             }
//         }
// 
//         // Handle roll
//         float roll_modi1 = roll - 2.0f * glm::pi<float>();
//         float roll_modi2 = roll + 2.0f * glm::pi<float>();
//         if (std::abs(roll_modi1 - last_roll) <= std::abs(roll_modi2 - last_roll)) {
//             if (std::abs(roll_modi1 - last_roll) < std::abs(roll - last_roll)) {
//                 roll = roll_modi1;
//             }
//         }
//         else {
//             if (std::abs(roll_modi2 - last_roll) < std::abs(roll - last_roll)) {
//                 roll = roll_modi2;
//             }
//         }
// 
//         // Handle yaw
//         float yaw_modi1 = yaw - 2.0f * glm::pi<float>();
//         float yaw_modi2 = yaw + 2.0f * glm::pi<float>();
//         if (std::abs(yaw_modi1 - last_yaw) <= std::abs(yaw_modi2 - last_yaw)) {
//             if (std::abs(yaw_modi1 - last_yaw) < std::abs(yaw - last_yaw)) {
//                 yaw = yaw_modi1;
//             }
//         }
//         else {
//             if (std::abs(yaw_modi2 - last_yaw) < std::abs(yaw - last_yaw)) {
//                 yaw = yaw_modi2;
//             }
//         }
// 
//         return glm::vec3(pitch, roll, yaw);
//     }
}