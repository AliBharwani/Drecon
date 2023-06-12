using UnityEngine;
using System;
using static MotionMatchingAnimator.Bones;

public static class UnityObjUtils
{
    public static void setAllChildrenRenderersMaterial(Transform obj, Material mat)
    {
        foreach (var renderer in obj.GetComponentsInChildren<Renderer>())
            if (renderer.enabled)
                renderer.sharedMaterial = mat;
    }
    public static GameObject getChildCapsuleCollider(GameObject child)
    {
        foreach (Transform grandchild in child.transform)
        {
            if (grandchild.GetComponent<CapsuleCollider>() != null)
                return grandchild.gameObject;
        }
        return null;
    }

    public static GameObject getChildBoxCollider(GameObject child)
    {
        foreach (Transform grandchild in child.transform)
        {
            if (grandchild.GetComponent<BoxCollider>() != null)
                return grandchild.gameObject;
        }
        return null;
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

    /* Get 6 points on capsule object
    first we start off at the center, and the 6 points are given by
    center plus-minus radius on y dimension
    center plus-minus radius on z dimension
    and since capsule is oriented on x dimension, the tippy tops are,
    as we previously calculated,
    center plus minus height/2 on x dimension */
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

    public static void getSixPointsOnCollider(GameObject obj, ref Vector3[] outputs, MotionMatchingAnimator.Bones bone)
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
