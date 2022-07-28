using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Text;
public class database
{

    Vector3[][] bone_positions;
    Vector3[][] bone_velocities;
    Quaternion[][] bone_rotations;
    Vector3[][] bone_angular_velocities;
    public int[] bone_parents;

    int[] range_starts;
    int[] range_stops;

    float[][] features;
    float[]  features_offset;
    float[] features_scale;

    public database(string filename)
    {
        load_db(filename);
    }

    public void setDataToFrame(ref Vector3[] local_bone_positions, ref Quaternion[] local_bone_rotations, int frameIdx)
    {
        for(int i = 0; i < nbones(); i++)
        {
            local_bone_positions[i] = bone_positions[i][frameIdx];
            local_bone_rotations[i] = bone_rotations[i][frameIdx];
        }

    }
    public int nframes() { return bone_positions[0].Length; }
    public int nbones() { return bone_positions.Length; }
    public int nranges() { return range_starts.Length; }
    public int nfeatures() { return features.Length; }

    private void load2Darray(BinaryReader reader, ref Vector3[][] arr, bool invertY = false)
    {
        // rows = num_frames, cols = num_bones
        uint rows, cols;
        rows = reader.ReadUInt32();
        cols = reader.ReadUInt32();
        //Debug.Log($"Rows: {rows} , Cols: {cols}");
        arr = new Vector3[cols][];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (i == 0)
                {
                    arr[j] = new Vector3[rows];
                }
                float x, y, z;
                x = reader.ReadSingle();
                y = reader.ReadSingle();
                y *= invertY ? -1 : 1;
                z = reader.ReadSingle();
                arr[j][i] = new Vector3(x, y, z);
                //if (i < 3)
                //{
                //    Debug.Log($"[ {x}, {y}, {z}");
                //}
            }
        }
    }


    private void load2Darray(BinaryReader reader, ref Quaternion[][] arr)
    {
        // rows = num_frames, cols = num_bones
        uint rows, cols;
        rows = reader.ReadUInt32();
        cols = reader.ReadUInt32();
        //Debug.Log($"Rows: {rows} , Cols: {cols}");
        arr = new Quaternion[cols][];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (i == 0)
                {
                    arr[j] = new Quaternion[rows];
                }
                float w, x, y, z;
                w = reader.ReadSingle();
                x = reader.ReadSingle();
                y = reader.ReadSingle();
                z = reader.ReadSingle();
                //w = reader.ReadSingle();
                arr[j][i] = new Quaternion(x, y, z, w);
                //arr[j][i] = Utils.to_unity_rot(new Quaternion(x, y, z, w));
                //if (i < 3)
                //{
                //    Debug.Log($"[ {x}, {y}, {z}, {w}]");
                //}
            }
        }
    }

    private void loadArray(BinaryReader reader, ref int[] arr)
    {
        // rows = num_frames, cols = num_bones
        uint size;
        size = reader.ReadUInt32();
        //Debug.Log(size);
        //Debug.Log($"Rows: {rows} , Cols: {cols}");
        arr = new int[size];
        for (int i = 0; i < size; i++)
        {
            uint val = reader.ReadUInt32();
            arr[i] = (int) val;
            //Debug.Log(val);
        }
    }

    private void load_db(string filename)
    {
        using (var stream = File.Open(filename, FileMode.Open))
        {
            using (var reader = new BinaryReader(stream, Encoding.UTF8))
            {
                load2Darray(reader, ref bone_positions, false);
                load2Darray(reader, ref bone_velocities);
                load2Darray(reader, ref bone_rotations);
                load2Darray(reader, ref bone_angular_velocities);
                loadArray(reader, ref bone_parents);
                // in the original generate database code, bone_parents is written in binary
                // as an unsigned int, need to convert it
                loadArray(reader, ref  range_starts);
                loadArray(reader, ref range_stops);
            }
        }
        bone_parents[0] = -1;

    }



}
