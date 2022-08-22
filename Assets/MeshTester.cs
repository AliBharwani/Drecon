using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

using System;
using System.Text;

public class MeshTester : MonoBehaviour
{
    #if UNITY_EDITOR
    [ContextMenu("Run Test Function")]
    public void testMeshRun()
    {
        testMesh();
    }
    [ContextMenu("Delete gizmos")]
    public void deleteGizmos()
    {
        bone_verts_lists = null;
        //closest_verts = null;
    }
    [ContextMenu("Output to text files")]
    public void writeToTxtFiles()
    {
        outputBoneVertices();
    }
    [ContextMenu("testCalculateBoundingCapsule")]
    public void testCalculateBoundingCapsule()
    {
        testMesh();
        debugObj.calculateBoundingCapsule(bone_verts_lists[RIGHT_FOREARM_ID].ToArray());
    }
#endif
    public debugScript debugObj;
    //bool[] verticesToHighlight;
    //int[] vert_bones;
    Color[] boneToColor;
    bool[] boneToColorSet;
    int m_vertexCount;
    Vector3[] m_vertices;
    //Vector3[][] closest_verts;
    //List<Tuple<float, int>>[] closest_verts_dyn;
    //int[] num_verts_found; // maps bone_id to num verts to draw for it
    public float vert_threshold = .9f;
    public int num_closest = 100;
    public int num_bones = 70;
    public bool gizmos_on = true;
    public bool debug = true;
    public int[] bone_ids_to_use;

    public int left_hand_bone_id = 9;
    public int right_hand_bone_id = 36;
    private HashSet<int> left_hand_ids;
    private HashSet<int> right_hand_ids;

    // bone_verts_lists[i] = a list of vertices associated with bone i
    private List<Vector3>[] bone_verts_lists;

    // Update is called once per frame
    void testMesh()
    {
        bone_verts_lists = new List<Vector3>[70];
        for (int i = 0; i < 70; i++)
            bone_verts_lists[i] = new List<Vector3>();
        left_hand_ids = new HashSet<int>();
        right_hand_ids = new HashSet<int>();

        for (int i = 10; i < 33; i++)
            left_hand_ids.Add(i);
        for (int i = 37; i < 60; i++)
            right_hand_ids.Add(i);

        SkinnedMeshRenderer rend = GetComponent<SkinnedMeshRenderer>();

        Mesh mesh = rend.sharedMesh;
        BoneWeight[] bws = mesh.boneWeights;
        boneToColor = new Color[100];
        boneToColorSet = new bool[100];
        //vert_bones = new int[mesh.vertexCount];
        m_vertexCount = mesh.vertexCount;
        m_vertices = mesh.vertices;
        //verticesToHighlight = new bool[mesh.vertexCount];
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

            if (!boneToColorSet[bone_id])
            {
                boneToColorSet[bone_id] = true;
                boneToColor[bone_id] = new Color(UnityEngine.Random.Range(0, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), 1f);
            }
            bone_verts_lists[bone_id].Add(m_vertices[i]);
            //vert_bones[i] = bone_id;
            //verticesToHighlight[i] = true;
        }
    }
    private void outputBoneVertices()
    {
        foreach (int bone_id in bone_ids_to_use)
        {
            string newFilename = Application.dataPath + $"/outputs/vertex_data/bone{bone_id}_verts.bin";
            using (FileStream file = File.Create(newFilename))
            {
                using (BinaryWriter writer = new BinaryWriter(file))
                {
                    foreach(Vector3 vert in bone_verts_lists[bone_id])
                    {
                        //if (!verticesToHighlight[i])
                        //    continue;
                        //int cur_bone_id = vert_bones[i];
                        //if (cur_bone_id != bone_id)
                        //    continue;
                        //Vector3 vert = m_vertices[i];
                        writer.Write(vert.x);
                        writer.Write(vert.y);
                        writer.Write(vert.z);
                    }
                }
            }
        }
        
    }
    public float vert_size = .005f;
    public int RIGHT_FOREARM_ID = 8;
    private void OnDrawGizmos()
    {
        if (!gizmos_on || bone_verts_lists == null)
            return;
        //if (!abTest && (verticesToHighlight == null || verticesToHighlight.Length == 0))
        //    return;
        //if (abTest && closest_verts == null)
        //    return;

        //string test = "-1.00000000e+00, -4.37139506e-10, -2.27232949e-08";
        //Vector3 testVec = BVHUtils.convertToVec(test);
        //Gizmos.color = Color.blue;
        //Gizmos.DrawLine(Vector3.zero , testVec);
        foreach (int bone_id in bone_ids_to_use)
        {
            if (debug && bone_id != RIGHT_FOREARM_ID) // 8 == right forearm 
                continue;
            Gizmos.color = boneToColor[bone_id];
            foreach (Vector3 vert in bone_verts_lists[bone_id])
                Gizmos.DrawSphere(vert, vert_size);
        }
        //for (int i = 0; i < m_vertexCount; i++)
        //{
        //    if (!verticesToHighlight[i])
        //        continue;
        //    int bone_id = vert_bones[i];
        //    if (debug && bone_id != 8) // 8 == right forearm 
        //        continue;
        //    bool use = false;
        //    foreach (int id in bone_ids_to_use)
        //        use = use || bone_id == id;
        //    if (!use)
        //        continue;
        //    Gizmos.color = boneToColor[bone_id];
        //    Gizmos.DrawSphere(m_vertices[i], vert_size);
        //    string mean = "0.647868  ,  1.44229543, -0.03082059";
        //    Vector3 meanPoint = BVHUtils.convertToVec(mean);
        //    string test = "-0.99862385, -0.01221036,  0.05100307";
        //    Vector3 testVec = BVHUtils.convertToVec(test).normalized;
        //    Gizmos.color = Color.green;
        //    Gizmos.DrawSphere(BVHUtils.closestPoint(meanPoint, meanPoint + testVec, m_vertices[i]), vert_size);
        //}
    }
}
