using System.Linq;
using UnityEngine;

public static class ArrayUtils
{
    public static void debugArray<T>(T[] data, string name)
    {
        Debug.Log(name + string.Join(" , ", data));
    }

    public static void debugVector3Array(Vector3[] data, string name, string argument = "f6")
    {
        Debug.Log(name + string.Join(" , ", data.Select(vec => vec.ToString(argument))));
    }

    public static Quaternion[] identityQuatArr(int size)
    {
        Quaternion[] ret = new Quaternion[size];
        for (int i = 0; i < size; i++)
            ret[i] = Quaternion.identity;
        return ret;
    }
    public static void copyVecIntoArray(ref float[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    public static void copyVecIntoArray(ref float[] state, ref int start_idx, Vector2 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        start_idx += 2;
    }
}
