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
        verticesToHighlight = null;
        closest_verts = null;
    }
    [ContextMenu("Output to text files")]
    public void writeToTxtFiles()
    {
        outputBoneVertices();
    }
#endif

    bool[] verticesToHighlight;
    int[] vert_bones;
    Color[] boneToColor;
    bool[] boneToColorSet;
    int m_vertexCount;
    Vector3[] m_vertices;
    Vector3[][] closest_verts;
    List<Tuple<float, int>>[] closest_verts_dyn;
    int[] num_verts_found; // maps bone_id to num verts to draw for it
    public bool abTest = false;
    public float vert_threshold = .9f;
    public int num_closest = 100;
    public int num_bones = 70;
    public bool gizmos_on = true;
    public int[] bone_ids_to_use;

    public int left_hand_bone_id = 9;
    public int right_hand_bone_id = 36;
    private HashSet<int> left_hand_ids;
    private HashSet<int> right_hand_ids;
    void Start()
    {
        testMesh();

    }

    // Update is called once per frame
    void testMesh()
    {
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
        vert_bones = new int[mesh.vertexCount];
        m_vertexCount = mesh.vertexCount;
        m_vertices = mesh.vertices;
        verticesToHighlight = new bool[mesh.vertexCount];
        //foreach (BoneWeight b in bw)
        //{
        //    Debug.Log($"{b.boneIndex0} : {b.weight0} | {b.boneIndex1} : {b.weight1} | {b.boneIndex2} : {b.weight2} | {b.boneIndex3} : {b.weight3}");
        //}
        if (abTest)
        {
            num_verts_found = new int[num_bones];
            closest_verts = new Vector3[num_bones][];
            closest_verts_dyn = new List<Tuple<float, int>>[num_bones];
            for (int i = 0; i < num_bones; i++)
            {
                boneToColor[i] = new Color(UnityEngine.Random.Range(0, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), 1f);
                closest_verts[i] = new Vector3[num_closest];
                closest_verts_dyn[i] = new List<Tuple<float, int>>();
                // find the n closest vertices
                // Use min heap of maxes 
                // since I don't have a min heap I'll just negate all the values instead
                MaxHeap<int> minHeap = new MaxHeap<int>(num_closest);
                for (int j = 0; j < mesh.vertexCount; j++)
                {
                    BoneWeight bw = bws[j];
                    if (i != bw.boneIndex0 && i != bw.boneIndex1 && i != bw.boneIndex2 && i != bw.boneIndex3)
                        continue;
                    //Debug.Log($"Processing bone {i} and vertex {j}");
                    float bone_w = -1f;
                    if (i == bw.boneIndex0)
                        bone_w = bw.weight0;
                    else if (i == bw.boneIndex1)
                        bone_w = bw.weight1;
                    else if (i == bw.boneIndex2)
                        bone_w = bw.weight2;
                    else if (i == bw.boneIndex3)
                        bone_w = bw.weight3;

                    if (!minHeap.isFull() || -minHeap.peek() < bone_w)
                    {
                        int[] v = new int []{ j };
                        minHeap.add(-bone_w, v);
                    }
                    closest_verts_dyn[i].Add(new Tuple<float, int>(bone_w, j));
                }
                closest_verts_dyn[i].Sort((x, y) => y.Item1.CompareTo(x.Item1));

                for (int j = 0; j < minHeap.size; j++)
                {
                    var val = minHeap.arr[j + 1];
                    //Debug.Log(val.Item2);
                    int vert_idx = val.Item2[0];
                    closest_verts[i][j] = m_vertices[vert_idx];
                }
                num_verts_found[i] = minHeap.size;
            }
            return;
        }
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
            vert_bones[i] = bone_id;
            verticesToHighlight[i] = true;


        }
    }
    private void outputBoneVertices()
    {
        foreach (int bone_id in bone_ids_to_use)
        {
            string newFilename = Application.dataPath + $"/outputs/vertex_data/bone{bone_id}_verts.txt";
            TextWriter tw = new StreamWriter(newFilename, true);
            tw.WriteLine("[" + System.DateTime.Now + "]");
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < m_vertexCount; i++)
            {
                if (!verticesToHighlight[i])
                    continue;
                int cur_bone_id = vert_bones[i];
                if (cur_bone_id != bone_id)
                    continue;
                Vector3 vert = m_vertices[i];
                sb.Append($"{vert.x} , {vert.y} , {vert.z}\n");
            }
            tw.Write(sb.ToString());
            tw.Close();
        }
        
    }
    public float vert_size = .005f;
    public int percent_to_use = 3;
    private void OnDrawGizmos()
    {
        if (!gizmos_on)
            return;
        //if (!abTest && (verticesToHighlight == null || verticesToHighlight.Length == 0))
        //    return;
        //if (abTest && closest_verts == null)
        //    return;
        if (abTest)
        {
            for (int i = 0; i < num_bones; i++)
            {
                bool use = false;
                foreach (int id in bone_ids_to_use)
                    use = use || i == id;
                if (!use)
                    continue;
                Gizmos.color = boneToColor[i];
                //for (int j = 0; j < num_verts_found[i]; j++)
                //{
                //    Gizmos.DrawSphere(closest_verts[i][j], .008f);
                //}
                List<Tuple<float, int>> foo = closest_verts_dyn[i];
                for (int j = 0; j < foo.Count / percent_to_use; j++)
                {
                    int v_idx = foo[j].Item2;
                    Gizmos.DrawSphere(m_vertices[v_idx], vert_size);
                }

            }
            return;
        }
        for (int i = 0; i < m_vertexCount; i++)
        {
            if (!verticesToHighlight[i])
                continue;
            int bone_id = vert_bones[i];
            bool use = false;
            foreach (int id in bone_ids_to_use)
                use = use || bone_id == id;
            if (!use)
                continue;
            Gizmos.color = boneToColor[bone_id];
            Gizmos.DrawSphere(m_vertices[i], vert_size);
        }
    }
}
