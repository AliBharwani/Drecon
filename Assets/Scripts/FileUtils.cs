using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;

public static class FileUtils 
{
    public static void WriteVector(BinaryWriter bw, Vector3 vec)
    {
        bw.Write(vec.x);
        bw.Write(vec.y);
        bw.Write(vec.z);
    }

    public static void ReadVector(BinaryReader reader, ref Vector3 vec)
    {
        vec.x = reader.ReadSingle();
        vec.y = reader.ReadSingle();
        vec.z = reader.ReadSingle();
    }
}
