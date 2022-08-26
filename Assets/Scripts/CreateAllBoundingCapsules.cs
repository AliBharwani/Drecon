using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreateAllBoundingCapsules : MonoBehaviour
{
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
    Vector3[] m_vertices;

    [ContextMenu("Build all bone capsulse")]
    private void buildBoneCapsules()
    {
        getBoneVerts();
        for (int i = 0; i < bone_ids.Length; i++)
        {
            GameObject boneObject = boneObjects[i];
            Vector3[] verts = bone_verts_lists[bone_ids[i]].ToArray();
            GameObject capsuleObject = calculateBoundingCapsule(verts);
            capsuleObject.transform.parent = boneObject.transform;
        }
    }

    private GameObject calculateBoundingCapsule(Vector3[] verts)
    {
        int n = verts.Length;
        Vector3 mean = GeoUtils.calculateMean(verts);
        double[,] covar = GeoUtils.calculateCovarMat(verts);
        double[] eigenvalues = GeoUtils.getEigenvalues(covar);
        Vector3 largest_eigen = GeoUtils.getEigenvectorFromValue(covar, eigenvalues[0]).normalized;
        Vector3[] proj_verts = GeoUtils.projectVertsOntoAxis(verts, mean, mean + largest_eigen);
        float height = GeoUtils.getMaxDistApart(proj_verts);
        double dist_from_main_axis_sum = 0;
        foreach (Vector3 v in verts)
            dist_from_main_axis_sum += (v - GeoUtils.closestPointOnLine(mean, mean + largest_eigen, v)).magnitude;
        float radius = (float)dist_from_main_axis_sum / n;


        GameObject capsuleObject = new GameObject();
        capsuleObject.transform.position = mean;
        capsuleObject.transform.rotation = GeoUtils.getRotationBetween(Vector3.right, largest_eigen);
        CapsuleCollider capsule = capsuleObject.AddComponent<CapsuleCollider>();
        capsule.height = height;
        capsule.direction = 0;
        capsule.radius = radius;
        return capsuleObject;
    }

    private void getBoneVerts()
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
        m_vertices = mesh.vertices;

        for (int i = 0; i < mesh.vertexCount; i++)
        {
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
