using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System;
using static mm_v2.Bones;

public static class Utils
{
    public static List<Transform> getAllChildren(Transform transform)
    {
        List<Transform> all = new List<Transform>();
        Queue<Transform> queue = new Queue<Transform>();
        queue.Enqueue(transform);
        while (queue.Count > 0)
        {
            var c = queue.Dequeue();
            all.Add(c);
            foreach (Transform t in c)
                queue.Enqueue(t);
        }
        return all;
    }


    public static void debugArray<T>(T[] data, string name)
    {
        Debug.Log(name + string.Join(" , ", data));
    }

    public static void debugVector3Array(Vector3[] data, string name, string argument = "f6")
    {
        Debug.Log(name + string.Join(" , ", data.Select(vec => vec.ToString(argument))));
    }


    public static Quaternion[] identity_quat_arr(int size)
    {
        Quaternion[] ret = new Quaternion[size];
        for (int i = 0; i < size; i++)
            ret[i] = Quaternion.identity;
        return ret;
    }


    public static void copyVecIntoArray(ref float[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    public static void copyVecIntoArray(ref float[] state, ref int start_idx, Vector2 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        start_idx += 2;
    }

    public static Vector3 getChildColliderCenter(GameObject child)
    {
        foreach (Transform grandchild in child.transform)
        {
            if (grandchild.GetComponent<CapsuleCollider>() != null)
            {
                Vector3 center = grandchild.GetComponent<CapsuleCollider>().center;
                return grandchild.TransformPoint(center);
            }
            if (grandchild.GetComponent<BoxCollider>() != null)
            {
                Vector3 center = grandchild.GetComponent<BoxCollider>().center;
                return grandchild.TransformPoint(center);
            }

        }
        return Vector3.zero;
    }



    // ================================ UTIL ====================================
    // Get 6 points on capsule object
    // first we start off at the center, and the 6 points are given by
    // center plus-minus radius on y dimension
    // center plus-minus radius on z dimension
    // and since capsule is oriented on x dimension, the tippy tops are,
    // as we previously calculated,
    // center plus minus height/2 on x dimension 
    public static void getSixPointsOnCapsule(GameObject obj, ref Vector3[] outputs)
    {
        CapsuleCollider cap = obj.GetComponent<CapsuleCollider>();
        if (cap == null)
            throw new Exception($"Object you're trying to get six points of capsule on does not have capsule: {obj.name}");
        if (outputs.Length != 6)
            throw new Exception($"outputs should have length 6, actual length: {outputs.Length}");

        Vector3 center = cap.center;
        float rad = cap.radius;
        float half_height = cap.height / 2;
        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = Vector3.zero;
            if (i == 0)
                axis = Vector3.right;
            else if (i == 1)
                axis = Vector3.up;
            else if (i == 2)
                axis = Vector3.forward;

            // Cap direction corresponds to which dimension it has its height along and is important 
            // Use half height with Vector3.right because capsules are aligned on X axis
            float variable = i == cap.direction ? half_height : rad;
            outputs[i * 2] = obj.transform.TransformPoint(center + variable * axis);
            outputs[i * 2 + 1] = obj.transform.TransformPoint(center - variable * axis);
        }

    }

    public static void getSixPointsOnCollider(GameObject obj, ref Vector3[] outputs, mm_v2.Bones bone)
    {
        if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
            getSixPointsOnBox(obj, ref outputs);
        else
            getSixPointsOnCapsule(obj, ref outputs);

    }

    public static void getSixPointsOnBox(GameObject obj, ref Vector3[] outputs)
    {
        BoxCollider box = obj.GetComponent<BoxCollider>();
        if (box == null)
            throw new Exception($"Object you're trying to get six points of box on does not have box: {obj.name}");
        if (outputs.Length != 6)
            throw new Exception($"outputs should have length 6, actual length: {outputs.Length}");

        Vector3 center = box.center;
        Vector3 size = box.size;

        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = Vector3.zero;
            float move_amount = 0f;
            if (i == 0)
            {
                move_amount = size.x;
                axis = Vector3.right;
            }
            else if (i == 1)
            {
                move_amount = size.y;
                axis = Vector3.up;
            }
            else if (i == 2)
            {
                move_amount = size.z;
                axis = Vector3.forward;
            }
            // Cap direction corresponds to which dimension it has its height along and is important 
            // Use half height with Vector3.right because capsules are aligned on X axis
            outputs[i * 2] = obj.transform.TransformPoint(center + move_amount / 2 * axis);
            outputs[i * 2 + 1] = obj.transform.TransformPoint(center - move_amount / 2 * axis);
        }

    }

}
