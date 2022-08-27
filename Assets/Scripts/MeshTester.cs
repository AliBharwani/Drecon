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
    public int num_bones = 70;
    public bool gizmos_on = true;
    public bool debug = true;
    public int[] bone_ids_to_use;
    public int[] bone_ids_debug;
    public int left_hand_bone_id = 9;
    public int right_hand_bone_id = 36;
    private HashSet<int> left_hand_ids;
    private HashSet<int> right_hand_ids;

    // bone_verts_lists[i] = a list of vertices associated with bone i
    private List<Vector3>[] bone_verts_lists;

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
            bone_verts_lists[bone_id].Add(m_vertices[i] + transform.position);
        }
    }
    public GameObject hip;
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
        if (!gizmos_on)
            return;
        if (bone_verts_lists == null)
            testMesh();
        string mean = "0.647868  ,  1.44229543, -0.03082059";
        Vector3 meanPoint = BVHUtils.convertToVec(mean);
        string test = "-0.99862385, -0.01221036,  0.05100307";
        Vector3 testVec = BVHUtils.convertToVec(test).normalized;
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(meanPoint, meanPoint + testVec);
        Gizmos.DrawLine(meanPoint, meanPoint - testVec);

        foreach (int bone_id in bone_ids_to_use)
        {
            bool bone_id_in_debug = bone_ids_debug.Contains(bone_id);
            if (debug && !bone_id_in_debug) // 8 == right forearm 
                continue;
            Gizmos.color = boneToColor[bone_id];
            foreach (Vector3 vert in bone_verts_lists[bone_id])
                Gizmos.DrawSphere(vert, vert_size);
            Vector3[] verts = bone_verts_lists[bone_id].ToArray();
            Vector3[] proj_verts = GeoUtils.projectVertsOntoAxis(verts, meanPoint, meanPoint + testVec);
            Gizmos.color = Color.green;
            foreach (Vector3 vert in proj_verts)
                Gizmos.DrawSphere(vert, vert_size);

        }
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