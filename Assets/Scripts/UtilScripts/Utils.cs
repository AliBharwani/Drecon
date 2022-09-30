using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utils
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

    public static void debugArray<T>(T[] data, string name)
    {
        Debug.Log(name + string.Join(" , ", data));
    }
}
