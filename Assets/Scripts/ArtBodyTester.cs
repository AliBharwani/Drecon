using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class ArtBodyTester : MonoBehaviour
{
    public int target_fr = 5;
    public bool set_target_fr = false;
    public bool scene_view_on_start = true;
    private void Start()
    {
        //if (set_target_fr)
        //    Application.targetFrameRate = target_fr;
#if UNITY_EDITOR
        if (Application.isEditor && scene_view_on_start)
        {
            UnityEditor.EditorWindow.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
#endif
        //if (debug == 0)
        //    return; 
        if (debug == 2)
        {
            disable_all_art_bodies_except_debug();
            return;
        }
    }
    public GameObject test_obj;
    [ContextMenu("Disable all articulation bodies except debug object")]
    public void disable_all_art_bodies_except_debug()
    {
        List<Transform> all = getAllChildren();
        foreach (Transform t in all)
        {
            var obj = t.gameObject;
            ArticulationBody ab = obj.GetComponent<ArticulationBody>();
            if (ab != null && ab.enabled)
            {
                bool is_debug_obj = test_obj != null && t.gameObject.name == test_obj.name;
                if (!is_debug_obj)
                {
                    if (ab.isRoot)
                        ab.immovable = true;
                    ab.useGravity = false;
                    ab.jointType = ArticulationJointType.FixedJoint;
                }
            }
        }
    }
    [ContextMenu("Disable all articulation bodies")]
    public void disable_all_art_bodies()
    {
        foreach (Transform t in getAllChildren())
        {
            ArticulationBody ab = t.gameObject.GetComponent<ArticulationBody>();
            if (ab != null && ab.enabled)
                ab.enabled = false;
        }
    }
    public Material selected_material;
    [ContextMenu("Set All Material Colors To Selected Material")]
    public void set_all_material_colors_to_selected_mat()
    {
        set_all_material(selected_material);
    } 
    public void set_all_material(Material mat)
    {
        foreach (var renderer in GetComponentsInChildren<Renderer>())
        {
            if (renderer.enabled)
# if UNITY_EDITOR
                renderer.sharedMaterial = mat;
#else
                renderer.sharedMaterial = mat;
#endif
        }
    }

    private Keyboard kboard = Keyboard.current;
    public GameObject hips;
    public float torque = .1f;
    private void Update()
    {
        if (debug == 2)
        {
            ArticulationBody ab = test_obj.GetComponent<ArticulationBody>();
            Vector3 force = Vector3.zero;
            if (kboard.qKey.isPressed)
                force = test_obj.transform.up;
            else if (kboard.aKey.isPressed)
                force = -test_obj.transform.up;
            else if (kboard.wKey.isPressed)
                force = test_obj.transform.right;
            else if (kboard.sKey.isPressed)
                force = -test_obj.transform.right;
            else if (kboard.eKey.isPressed)
                force = test_obj.transform.forward;
            else if (kboard.dKey.isPressed)
                force = -test_obj.transform.forward;
            if (ab != null && ab.enabled)
                ab.AddTorque(force * torque);
            return;
        }
        if (kboard.spaceKey.wasPressedThisFrame)
        {
            ArticulationBody ab = hips.GetComponent<ArticulationBody>();
            if (ab != null && ab.enabled)
                ab.AddForce(Vector3.up * 90f);
        }
    }
    public string child_str;
    public bool draw_gizmos;
    public float gizmoSphereRad = .01f;
    public List<Vector3> gizmoSpheres;


    public GameObject[] bones;
    public int debug;
    public int AVG_HUMAN_DENSITY = 985; // kg / m^3
    [ContextMenu("Update all articulation bodies anchors")]
    private void set_articulation_body_anchors()
    {
        resetGizmos();
        for (int i = 0; i < bones.Length; i++)
        {
            GameObject bone = bones[i];
            GameObject child = getChildCapsuleCollider(bone);
            CapsuleCollider cap = child.GetComponent<CapsuleCollider>();
            float halfHeight = cap.height / 2;
            var world_pos_add = child.transform.TransformPoint(cap.center + halfHeight * Vector3.right );
            var world_pos_sub = child.transform.TransformPoint(cap.center + halfHeight * Vector3.right * -1);
            // We want to use the position that is closest to the parent 
            Transform parent = bone.transform.parent;
            var world_pos = Vector3.Distance(world_pos_add, parent.position) < Vector3.Distance(world_pos_sub, parent.position) ? world_pos_add : world_pos_sub;
            addGizmoSphere(world_pos);
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

    public Transform debug_to_local_pos;

    [ContextMenu("Disable all visualizers")]
    private void disable_all_capsule_renderers()
    {
        List<Transform> all = getAllChildren();
        foreach (Transform t in all)
        {
            var mesh_renderer = t.gameObject.GetComponent<MeshRenderer>();
            if (mesh_renderer != null)
                mesh_renderer.enabled = false;
        }
    }
    [ContextMenu("Deactivate all colliders")]
    private void deactivate_all_colliders()
    {
        List<Transform> all = getAllChildren();
        foreach (Transform t in all)
        {
            var colliider = t.gameObject.GetComponent<Collider>();
            if (colliider != null)
                t.gameObject.SetActive(false);
        }
    }

    [ContextMenu("Print tippy top of Capsule")]
    private void debug_capsule_surface_points()
    {
        GameObject child = getChildCapsuleCollider(FindDeepChild());
        CapsuleCollider cap = child.GetComponent<CapsuleCollider>();
        Vector3 center = cap.center;
        float halfHeight = cap.height / 2;
        var top = child.transform.TransformPoint(center + halfHeight * Vector3.right);
        var bottom = child.transform.TransformPoint(center - halfHeight * Vector3.right);
        Debug.Log($"Top: {top.ToString("f6")} | Bottom: {bottom.ToString("f6")}");
        var local_top = debug_to_local_pos.InverseTransformPoint(top);
        var local_bot = debug_to_local_pos.InverseTransformPoint(bottom);
        Debug.Log($"In local coords: top: {local_top.ToString("f6")} bottom: {local_bot.ToString("f6")}");
        addGizmoSphere(top);
        //addGizmoSphere(bottom);
    }

    public bool useGravity = true;
    [ContextMenu("Set all art bodies to \'use gravity\' param")]
    private void set_art_bodies_use_gravity()
    {
        List<Transform> all = getAllChildren();
        foreach (Transform t in all)
        {
            var ab = t.gameObject.GetComponent<ArticulationBody>();
            if (ab != null)
                ab.useGravity = useGravity;
        }
    }

    public float forceLimit = float.MaxValue;
    [ContextMenu("Set all art bodies force limit to force limit param")]
    private void setAllArtBodyForceLimits()
    {
        foreach (var body in GetComponentsInChildren<ArticulationBody>())
            body.SetAllForceLimit(forceLimit);
    }
    [ContextMenu("Remove all anchor positions and rotations")]
    private void removeAllAnchorPositionsAndRotations()
    {
        foreach(var ab in GetComponentsInChildren<ArticulationBody>())
        {
            ab.anchorPosition = Vector3.zero;
            ab.anchorRotation = Quaternion.identity;
        }
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

    public List<Transform> getAllChildren()
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