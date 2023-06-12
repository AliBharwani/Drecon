using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreateAllBoundingCapsules : MonoBehaviour
{
    public enum Bones
    {
        Bone_Hips = 0,
        Bone_LeftUpLeg = 65,
        Bone_LeftLeg = 66,
        Bone_LeftFoot = 67,
        Bone_LeftToe = 68,
        Bone_LeftToeEnd = 69,
        Bone_RightUpLeg = 60,
        Bone_RightLeg = 61,
        Bone_RightFoot = 62,
        Bone_RightToe = 63,
        Bone_RightToeEnd = 64,
        Bone_Spine = 1,
        Bone_Spine1 = 2,
        Bone_Spine2 = 3,
        Bone_Neck = 4,
        Bone_Head = 5,
        Bone_LeftShoulder = 33,
        Bone_LeftArm = 34,
        Bone_LeftForeArm = 35,
        Bone_LeftHand = 36,
        Bone_RightShoulder = 6,
        Bone_RightArm = 7,
        Bone_RightForeArm = 8,
        Bone_RightHand = 9,
    };
    public GameObject[] boneObjects;
    // maps idx -> corresponding bone ID 
    public int[] bone_ids;
    public int left_hand_bone_id = 9;
    public int right_hand_bone_id = 36;
    public int num_bones = 70;
    private HashSet<int> left_hand_ids;
    private HashSet<int> right_hand_ids;
    private int left_hand_ids_start = 10;
    private int left_hand_ids_end = 32;
    private int right_hand_ids_start = 37;
    private int right_hand_ids_end = 59;

    private int right_foot_ids_start = 62;
    private int right_foot_ids_end = 64;
    private int left_foot_ids_start = 67;
    private int left_foot_ids_end = 69;
    // bone_verts_lists[i] = a list of vertices associated with bone i
    private List<Vector3>[] bone_verts_lists;
    public bool debug;
    public Bones[] debug_bone_ids;
    public Bones[] bones_that_use_next_joints_verts = new Bones[] { Bones.Bone_LeftUpLeg, Bones.Bone_RightUpLeg };
    public Bones[] bones_that_use_prev_joints_verts = new Bones[] { Bones.Bone_LeftForeArm, Bones.Bone_RightForeArm};

    public Bones[] bone_ids_use_small_rad;
    public Bones[] boxed_bones;
    public float min_rad = .005f;
    public bool draw_gizmos;
    Vector3[] m_vertices;

    [ContextMenu("Build all bone capsulse")]
    private void buildBoneCapsules()
    {
        debug_lines = new List<Vector3[]>();
        debug_points = new List<Vector3>();
        initializeBoneVerts();

        for (int i = 0; i < bone_ids.Length; i++)
        {
            Bones cur_bone = (Bones) bone_ids[i];
            if (debug && !debug_bone_ids.Contains(cur_bone))
                continue;
            GameObject boneObject = boneObjects[i];
            Vector3[] verts = bone_verts_lists[bone_ids[i]].ToArray();
            debug_points.AddRange(verts);
            GameObject colliderObject = boxed_bones.Contains(cur_bone) ? calc_bounding_box(verts) : calc_bounding_capsule(verts, bone_ids[i]);
            colliderObject.transform.parent = boneObject.transform;
        }
    }
    private Vector3[] getVerts(int bone_id, ref bool[] kept)
    {
        return filter_verts(bone_verts_lists[bone_id].ToArray(), min_rad, ref kept);
    }
    private Vector3[] getVerts(int bone_id)
    {
        bool[] kept = new bool[0];
        return filter_verts(bone_verts_lists[bone_id].ToArray(), min_rad, ref kept);
    }
    private List<Vector3[]> debug_lines;
    private List<Vector3> debug_points;
    private GameObject calc_bounding_box(Vector3[] verts)
    {
        Vector3 mins = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        Vector3 maxes = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
        foreach(Vector3 vert in verts)
        {
            mins = new Vector3(Math.Min(mins.x, vert.x), Math.Min(mins.y, vert.y), Math.Min(mins.z, vert.z));
            maxes = new Vector3(Math.Max(maxes.x, vert.x), Math.Max(maxes.y, vert.y), Math.Max(maxes.z, vert.z));
        }

        GameObject boxObject = new GameObject();
        boxObject.transform.position = (mins + maxes) / 2;
        boxObject.transform.rotation = Quaternion.identity;
        BoxCollider box = boxObject.AddComponent<BoxCollider>();
        box.size = maxes - mins;
        return boxObject;
    }
    private GameObject calc_bounding_capsule(Vector3[] verts, int bone_id)
    {

        int n = verts.Length;
        Vector3[] full_verts = verts;

        if (bones_that_use_next_joints_verts.Contains((Bones) bone_id))
            full_verts = verts.Concat(getVerts(bone_id + 1)).ToArray(); 
        else if (bones_that_use_prev_joints_verts.Contains((Bones) bone_id))
            full_verts = verts.Concat(getVerts(bone_id - 1)).ToArray();

        Vector3 mean = GeoUtils.calc_mean(full_verts);

        SkinnedMeshRenderer rend = GetComponent<SkinnedMeshRenderer>();
        Mesh mesh = rend.sharedMesh;
        double[,] covar = GeoUtils.calculate_continous_covar(mesh);
        double[] eigenvalues = GeoUtils.get_eigenvalues(covar);
        Vector3 largest_eigen = GeoUtils.get_eigenvector_from_value(covar, eigenvalues[0]).normalized;

        Vector3[] proj_verts = GeoUtils.proj_verts_onto_axis(verts, mean, mean + largest_eigen);
        Vector3 center = Vector3.zero;
        float height = GeoUtils.get_max_dist_apart(proj_verts, ref center);
        float radius;
        if (bone_ids_use_small_rad.Contains((Bones)bone_id))
        {
            double dist_from_main_axis_sum = 0;
            foreach (Vector3 v in verts)
                dist_from_main_axis_sum += (v - GeoUtils.closest_point_on_line(mean, mean + largest_eigen, v)).magnitude;
            radius = (float)dist_from_main_axis_sum / n;
        }
        else
        {
            double max_dist_from_main_axis = 0;
            foreach (Vector3 v in verts)
                max_dist_from_main_axis = Math.Max(max_dist_from_main_axis, (v - GeoUtils.closest_point_on_line(mean, mean + largest_eigen, v)).magnitude);
            radius = (float)max_dist_from_main_axis;
        }

        debug_lines.Add(new Vector3[] { mean - largest_eigen, mean + largest_eigen });
        GameObject capsuleObject = new GameObject();
        capsuleObject.transform.position = center;
        capsuleObject.transform.rotation = GeoUtils.get_rot_between(Vector3.right, largest_eigen);
        CapsuleCollider capsule = capsuleObject.AddComponent<CapsuleCollider>();
        capsule.height = height;
        capsule.direction = 0;
        capsule.radius = radius;
        return capsuleObject;
    }
    private void OnDrawGizmos()
    {
        if (!draw_gizmos || debug_lines == null || debug_points == null) 
            return;
        Gizmos.color = Color.blue;
        foreach(Vector3[] line in debug_lines)
        {
            Gizmos.DrawLine(line[0], line[1]);
        }
        Gizmos.color = Color.green;
        foreach (Vector3 v in debug_points)
            Gizmos.DrawSphere(v, .005f);
    }
    public bool do_not_filter;
    // Rudimentary algorithm to filter out a bunch of verts that are close to each other
    public  Vector3[] filter_verts(Vector3[] verts, float min_rad, ref bool[] kept)
    {
        int n = verts.Length;
        bool[] keep = Enumerable.Repeat(true, n).ToArray();
        kept = keep;
        if (do_not_filter)
            return verts;
        int it = 0;
        int num_kept = verts.Length;
        while (true)
        {
            bool should_break = true;
            for(int i = 0; i < n; i++)
            {
                if (!keep[i])
                    continue;
                for (int j = 0; j < n; j++)
                {
                    if (!keep[j] || i == j)
                        continue;
                    if (Vector3.Distance(verts[i], verts[j]) < min_rad)
                    {
                        keep[i] = false;
                        num_kept--;
                        should_break = false;
                        break;
                    }
                }
            }
            if (should_break)
                break;
            it++;
        }
        Vector3[] new_verts = new Vector3[num_kept];
        int new_verts_idx = 0;
        for (int i = 0; i < n; i++)
            if (keep[i])
                new_verts[new_verts_idx++] = verts[i];
        return new_verts;
    }
    private void initializeBoneVerts()
    {
        bone_verts_lists = new List<Vector3>[num_bones];
        for (int i = 0; i < 70; i++)
            bone_verts_lists[i] = new List<Vector3>();
        left_hand_ids = new HashSet<int>();
        right_hand_ids = new HashSet<int>();

        for (int i = left_hand_ids_start; i < left_hand_ids_end + 1; i++)
            left_hand_ids.Add(i);
        for (int i = right_hand_ids_start; i < right_hand_ids_end + 1; i++)
            right_hand_ids.Add(i);
        SkinnedMeshRenderer rend = GetComponent<SkinnedMeshRenderer>();

        Mesh mesh = rend.sharedMesh;
        BoneWeight[] bws = mesh.boneWeights;
        bool[] kept = new bool[mesh.vertexCount];
        m_vertices = mesh.vertices;
        filter_verts(mesh.vertices, min_rad, ref kept);
        int it = 0;
        for (int i = 0; i < mesh.vertexCount; i++)
        {
            if (!kept[i])
                continue;
            it++;
            BoneWeight bw = bws[i];
            int bone_id = -1;
            if (bw.weight0 > Mathf.Max(bw.weight1, bw.weight2, bw.weight3))
                bone_id = bw.boneIndex0;
            else if (bw.weight1 > Mathf.Max(bw.weight0, bw.weight2, bw.weight3))
                bone_id = bw.boneIndex1;
            else if (bw.weight2 > Mathf.Max(bw.weight1, bw.weight0, bw.weight3))
                bone_id = bw.boneIndex2;
            else if (bw.weight3 > Mathf.Max(bw.weight1, bw.weight2, bw.weight0))
                bone_id = bw.boneIndex3;
            if (bone_id == -1)
                continue;

            if (left_hand_ids.Contains(bone_id))
                bone_id = left_hand_bone_id;
            else if (right_hand_ids.Contains(bone_id))
                bone_id = right_hand_bone_id;
            else if (bone_id >= right_foot_ids_start && bone_id <= right_foot_ids_end)
                bone_id = right_foot_ids_start;
            else if (bone_id >= left_foot_ids_start && bone_id <= left_foot_ids_end)
                bone_id = left_foot_ids_start;
            bone_verts_lists[bone_id].Add(m_vertices[i]);
        }
    }
}
