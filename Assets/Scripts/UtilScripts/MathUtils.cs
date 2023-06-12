using System.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathUtils
{

    public static Vector3 quat_mul_vec3(Quaternion q, Vector3 v)
    {
        Vector3 t = 2.0f * Vector3.Cross( new Vector3(q.x, q.y, q.z), v);
        return v + q.w * t + Vector3.Cross(new Vector3(q.x, q.y, q.z), t);
    }

    public static Vector3 quat_inv_mul_vec3(Quaternion q, Vector3 v)
    {
        return quat_mul_vec3(quat_inv(q), v);
    }

    public static Quaternion quat_inv(Quaternion q)
    {
        return new Quaternion(q.x, q.y, q.z, -q.w);
    }
    public static Quaternion quat_neg(Quaternion q)
    {
        return new Quaternion(-q.x, -q.y, -q.z, -q.w);
    }
    public static Quaternion quat_abs(Quaternion x)
    {
        return x.w < 0.0 ? quat_neg(x) : x;
    }

    public static Quaternion quat_inv_mul(Quaternion q, Quaternion p)
    {
        return quat_inv(q) *  p;
    }
    public static Quaternion quat_between(Vector3 p, Vector3 q)
    {
        Vector3 c = Vector3.Cross(p, q);

        return new Quaternion(
            c.x,
            c.y,
            c.z,
            Mathf.Sqrt(Vector3.Dot(p, p) * Vector3.Dot(q, q)) + Vector3.Dot(p, q)).normalized;
    }
    public static Quaternion quat_from_angle_axis(float angle, Vector3 axis)
    {
        float c = Mathf.Cos(angle / 2.0f);
        float s = Mathf.Sin(angle / 2.0f);
        return new Quaternion(s * axis.x, s * axis.y, s * axis.z, c);
    }
    public static Quaternion quat_exp(Vector3 v, float eps = 1e-8f)
    {
        float halfangle = Mathf.Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

        if (halfangle < eps)
        {
            return Quaternion.Normalize(new Quaternion(v.x, v.y, v.z, 1.0f));
        }
        else
        {
            float c = Mathf.Cos(halfangle);
            float s = Mathf.Sin(halfangle) / halfangle;
            return new Quaternion(s * v.x, s * v.y, s * v.z, c);
        }
    }

    public static Quaternion to_unity_rot(Quaternion q)
    {
        return new Quaternion(q.x, -q.y, -q.z, q.w);
    }
    public static Vector3 quat_log(Quaternion q, float eps = 1e-8f)
    {
        float length = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z);

        if (length < eps)
        {
            return new Vector3(q.x, q.y, q.z);
        }
        else
        {
            float halfangle =  Mathf.Acos(Mathf.Clamp(q.w, -1.0f, 1.0f));
            return halfangle * (new Vector3(q.x, q.y, q.z) / length);
        }
    }

    public static Quaternion quat_from_scaled_angle_axis(Vector3 v, float eps = 1e-8f)
    {
        return quat_exp(v / 2.0f, eps);
    }

    public static Vector3 quat_to_scaled_angle_axis(Quaternion q, float eps = 1e-8f)
    {
        return 2.0f * quat_log(q, eps);
    }

    public static Quaternion quat_mul_inv(Quaternion q, Quaternion p)
    {
        return q * quat_inv(p);
    }

    public static Quaternion quat_from_stick_dir(float x, float y)
    {
        return Quaternion.AngleAxis(Mathf.Atan2(x, y) * Mathf.Rad2Deg, Vector3.up);
    }

    // 2D Rotations represented by a single angle are continous in the form [cos(theta), sin(theta)] 
    public static Vector2 getContinuousRepOf2DAngle(float thetaRads)
    {
        return new Vector2(Mathf.Cos(thetaRads), Mathf.Sin(thetaRads));
    }

    public static float geodesicBetweenTwoRotationMatrices(Matrix4x4 a, Matrix4x4 b, bool inDeg = false)
    {
        Matrix4x4 lossMat = (b * a.transpose);
        float trace = lossMat[0, 0] + lossMat[1, 1] + lossMat[2, 2];
        // Need clamping because Acos will throw NAN for values outside [-1, 1]
        float traceClamped = Mathf.Clamp((trace - 1) / 2, -1f, 1f);
        return Mathf.Acos(traceClamped) * (inDeg ? Mathf.Rad2Deg : 1f);
    }


    // Apply Gram-Schmidt process treating v1 - v2 as columns of new rotation matrix
    public static void GramSchmidtNormalization(Vector3 v1, Vector3 v2, out Vector3 e1, out Vector3 e2, out Vector3 e3)
    {
        e1 = v1.normalized;
        Vector3 u2 = v2 - (Vector3.Dot(e1, v2) * e1);
        e2 = u2.normalized;
        e3 = Vector3.Cross(e1, e2);
    }
    public static Matrix4x4 MatrixFrom6DRepresentation(Vector3 v1, Vector3 v2)
    {
        GramSchmidtNormalization(v1, v2, out Vector3 e1, out Vector3 e2, out Vector3 e3);
        Matrix4x4 rotMatrix = Matrix4x4.identity;
        rotMatrix[0, 0] = e1.x;
        rotMatrix[1, 0] = e1.y;
        rotMatrix[2, 0] = e1.z;

        rotMatrix[0, 1] = e2.x;
        rotMatrix[1, 1] = e2.y;
        rotMatrix[2, 1] = e2.z;

        rotMatrix[0, 2] = e3.x;
        rotMatrix[1, 2] = e3.y;
        rotMatrix[2, 2] = e3.z;
        return rotMatrix;
    }
    public static Quaternion RotateObjectWithOrthonormalVector(Vector3 v1, Vector3 v2)
    {
        GramSchmidtNormalization(v1, v2, out Vector3 e1, out _, out Vector3 e3);
        // Create quaternion from orthonormalized vectors
        Quaternion rotation = new Quaternion();
        rotation.SetFromToRotation(Vector3.right, e1);
        rotation *= Quaternion.FromToRotation(Vector3.up, e3);
        return rotation;
    }

    public static Quaternion QuatFrom6DRepresentation(Vector3 v1, Vector3 v2)
    {

        float qw, qx, qy, qz;
        Matrix4x4 mat = MatrixFrom6DRepresentation(v1, v2);
        float m00 = mat[0, 0];
        float m10 = mat[1, 0];
        float m20 = mat[2, 0];

        float m01 = mat[0, 1];
        float m11 = mat[1, 1];
        float m21 = mat[2, 1];

        float m02 = mat[0, 2];
        float m12 = mat[1, 2];
        float m22 = mat[2, 2];
        float tr = m00 + m11 + m22;

        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        if (tr > 0)
        {
            float S = Mathf.Sqrt(tr + 1.0f) * 2; // S=4*qw 
            qw = 0.25f * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
        }
        else if ((m00 > m11) & (m00 > m22))
        {
            float S = Mathf.Sqrt(1.0f + m00 - m11 - m22) * 2; // S=4*qx 
            qw = (m21 - m12) / S;
            qx = 0.25f * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        }
        else if (m11 > m22)
        {
            float S = Mathf.Sqrt(1.0f + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S;
            qy = 0.25f * S;
            qz = (m12 + m21) / S;
        }
        else
        {
            float S = Mathf.Sqrt(1.0f + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25f * S;
        }
        Quaternion q = new Quaternion(qx, qy, qz, qw);
        return q.normalized;
    }

    public static float GetYAngle(this Quaternion q, bool inDegrees = false)
    {
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
        return Mathf.Atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * Mathf.Pow(q.y, 2) - 2 * Mathf.Pow(q.z, 2)) * (inDegrees ? Mathf.Rad2Deg : 1);
    }
}
