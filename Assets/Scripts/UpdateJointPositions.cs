using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpdateJointPositions : MonoBehaviour
{
    public string child_str;
    public bool draw_gizmos;
    public float gizmoSphereRad = .01f;
    public List<Vector3> gizmoSpheres;


    public GameObject[] bones;
    // bones_joint_add[i] = true means anchor position goes to capsule.center + halfheight, false means minus
    public bool[] bones_joints_add;
    public bool debug;

    [ContextMenu("Update all articulation bodies centers")]
    private void setAllArticulationBodies()
    {
        for (int i = 0; i < bones.Length; i++)
        {
            GameObject bone = bones[i];
            bool add = bones_joints_add[i];
            GameObject child = getChildCapsuleCollider(bone);
            CapsuleCollider cap = child.GetComponent<CapsuleCollider>();
            float halfHeight = cap.height / 2;
            var world_pos = child.transform.TransformPoint(cap.center + halfHeight * Vector3.right * (add ? 1 : -1));
            var local_pos = bone.transform.InverseTransformPoint(world_pos);
            var artic_body = bone.GetComponent<ArticulationBody>();
            if (artic_body == null)
            {
                Debug.Log($"Bone {bone.name} does not have an articulation body");
                continue;
            }
            artic_body.anchorPosition = local_pos;
        }
    }


    [ContextMenu("Print tippy top of Capsule")]
    private void printCapsuleTippyTop()
    {
        GameObject child = getChildCapsuleCollider(FindDeepChild());
        CapsuleCollider cap = child.GetComponent<CapsuleCollider>();
        Vector3 center = cap.center;
        float halfHeight = cap.height / 2;
        var top = child.transform.TransformPoint(center + halfHeight * Vector3.right);
        var bottom = child.transform.TransformPoint(center - halfHeight * Vector3.right);
        Debug.Log($"Top: {top.ToString("f6")} | Bottom: {bottom.ToString("f6")}");
        resetGizmos();
        addGizmoSphere(top);
        addGizmoSphere(bottom);
    }

    [ContextMenu("Reset gizmos")]
    private void resetGizmos()
    {
        gizmoSpheres = null;
    }
    private void addGizmoSphere(Vector3 center)
    {
        if (gizmoSpheres == null)
            gizmoSpheres = new List<Vector3>();
        gizmoSpheres.Add(center);
    }
    private void OnDrawGizmos()
    {
        if (!draw_gizmos || gizmoSpheres == null)
            return;
        Gizmos.color = Color.cyan;
        foreach(Vector3 sphere in gizmoSpheres)
        {
            Gizmos.DrawSphere(sphere, gizmoSphereRad);
        }

    }

    public GameObject FindDeepChild()
    {
        Queue<Transform> queue = new Queue<Transform>();
        queue.Enqueue(transform);
        while (queue.Count > 0)
        {
            var c = queue.Dequeue();
            if (c.name == child_str)
                return c.gameObject;
            foreach (Transform t in c)
                queue.Enqueue(t);
        }
        Debug.Log($"Could not find {child_str}");
        return null;
    }

    private GameObject getChildCapsuleCollider(GameObject child)
    {
        foreach (Transform grandchild in child.transform)
        {
            if (grandchild.GetComponent<CapsuleCollider>() != null)
                return grandchild.gameObject;
        }
        return null;
    }

}

/*
 0 hip
1 spine
2 spine1
3 spine2
4 neck (contains all head verts basically)
5 head
6 right shoulder
7 right arm
8 right forearm
9 right hand
[10, 32] right fingers
33 left shoulder
34 left arm 
35 left forearm
36 left hand
[37, 59] left fingers
60 right up leg
61 right leg 
62 right foot
63 right toe
64 right toe end
65 left up leg
66 left leg 
67 left foot
68 left toe 
69 left toe end

 
 */