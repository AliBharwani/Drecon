using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class SpringUtils
{

    // Daniel Holden related code
    // use taylor series to calculate e^(-x)
    public static float fast_negexp(float x)
    {
        return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
    }

    public static float halflife_to_damping(float halflife, float eps = 1e-5f)
    {
        return (4.0f * 0.69314718056f) / (halflife + eps);
    }



    public static void simple_spring_damper_exact(
        ref float x,
        ref float v,
        in float x_goal,
        in float halflife,
        in float dt)
    {
        float y = halflife_to_damping(halflife) / 2.0f;
        float j0 = x - x_goal;
        float j1 = v + j0 * y;
        float eydt = fast_negexp(y * dt);

        x = eydt* (j0 + j1* dt) + x_goal;
        v = eydt* (v - j1* y*dt);
    }

    public static void simple_spring_damper_exact(
        ref Vector3 x,
        ref Vector3 v,
        Vector3 x_goal,
        float halflife,
        float dt)
    {
        float y = halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = x - x_goal;
        Vector3 j1 = v + j0 * y;
        float eydt = fast_negexp(y * dt);

        x = eydt * (j0 + j1 * dt) + x_goal;
        v = eydt * (v - j1 * y * dt);
    }

    public static void spring_character_update(
        Vector2 x,
        Vector2 v,
        Vector2 x_goal,
        float halflife,
        float dt,
        out Vector2 x_out,
        out Vector2 v_out)
    {
        float y = halflife_to_damping(halflife) / 2.0f;
        Vector2 j0 = x - x_goal;
        Vector2 j1 = v + j0 * y;
        float eydt = fast_negexp(y * dt);

        x_out = eydt * (j0 + j1 * dt) + x_goal;
        v_out = eydt * (v - j1 * y * dt);
    }

    public static void spring_character_update(
        Vector2[] px,
        Vector2[] pv,
        Vector2[] pa,
        int idx,
        Vector2 v_goal,
        float halflife,
        float dt)
    {
        Vector2 x = px[idx];
        Vector2 v = pv[idx];
        Vector2 a = pa[idx];
        float y = halflife_to_damping(halflife) / 2.0f;
        Vector2 j0 = v - v_goal;
        Vector2 j1 = a + j0 * y;
        float eydt = fast_negexp(y * dt);

        px[idx] = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + v_goal * dt + x;
        pv[idx] = eydt * (j0 + j1 * dt) + v_goal;
        pa[idx] = eydt * (a - j1 * y * dt);
    }

    public static void spring_character_predict(
    Vector2[] px,
    Vector2[] pv,
    Vector2[] pa,
    int count,
    Vector2 x,
    Vector2 v,
    Vector2 a,
    Vector2 v_goal,
    float halflife,
    float dt)
    {
        for (int i = 0; i < count; i++)
        {
            px[i] = x;
            pv[i] = v;
            pa[i] = a;
        }

        for (int i = 0; i < count; i++)
        {
            spring_character_update(px, pv, pa, i, v_goal, halflife, (i + 1) * dt);
        }
    }
    public static void simple_spring_damper_implicit(
        ref Quaternion x,
        ref Vector3 v,
        in Quaternion x_goal,
        in float halflife,
        in float dt)
    {
        float y = halflife_to_damping(halflife) / 2.0f;

        Vector3 j0 = MathUtils.quat_to_scaled_angle_axis(MathUtils.quat_abs(x * MathUtils.quat_inv(x_goal)));
        Vector3 j1 = v + j0 * y;

        float eydt = fast_negexp(y * dt);

        x = MathUtils.quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * x_goal;
        v = eydt * (v - (j1 * y * dt));
    }
    public static void decay_spring_damper_exact(
        ref Vector3 x,
        ref Vector3 v,
        in float halflife,
        in float dt)
    {
        float y = halflife_to_damping(halflife) / 2.0f;
        Vector3 j1 = v + x * y;
        float eydt = fast_negexp(y * dt);

        x = eydt* (x + j1* dt);
        v = eydt* (v - j1* y*dt);
    }

    public static void decay_spring_damper_exact(
    ref Quaternion x,
    ref Vector3 v,
    in float halflife,
    in float dt)
    {
        float y = halflife_to_damping(halflife) / 2.0f;

        Vector3 j0 = MathUtils.quat_to_scaled_angle_axis(x);
        Vector3 j1 = v + j0 * y;

        float eydt = fast_negexp(y * dt);

        x = MathUtils.quat_from_scaled_angle_axis(eydt * (j0 + j1 * dt));
        v = eydt * (v - (j1 * y * dt));
    }

    public static void inertialize_transition(
        ref Vector3 off_x, 
        ref Vector3 off_v, 
        in Vector3 src_x,
        in Vector3 src_v,
        in Vector3 dst_x,
        in Vector3 dst_v)
    {
        off_x = (src_x + off_x) - dst_x;
        off_v = (src_v + off_v) - dst_v;
    }


    public static void inertialize_transition(
        ref Quaternion off_x,
        ref Vector3 off_v,
        in Quaternion src_x,
        in Vector3 src_v,
        in Quaternion dst_x,
        in Vector3 dst_v)
    {
        off_x = MathUtils.quat_abs( (off_x * src_x) * MathUtils.quat_inv(dst_x));
        off_v = (off_v + src_v) - dst_v;
    }

    public static void inertialize_update(
        ref Vector3 out_x,
        ref Vector3 out_v,
        ref Vector3 off_x,
        ref Vector3 off_v,
        in Vector3 in_x,
        in Vector3 in_v,
        in float halflife,
        in float dt)
    {
        decay_spring_damper_exact(ref off_x, ref off_v, halflife, dt);
        out_x = in_x + off_x;
        out_v = in_v + off_v;
    }

    public static void inertialize_update(
        ref Quaternion out_x,
        ref Vector3 out_v,
        ref Quaternion off_x,
        ref Vector3 off_v,
        in Quaternion in_x,
        in Vector3 in_v,
        in float halflife,
        in float dt)
    {
        decay_spring_damper_exact(ref off_x, ref off_v, halflife, dt);
        //off_x = Quaternion.identity;
        out_x = off_x * in_x;
        out_v = off_v + in_v;
    }
}
