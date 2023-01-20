using System.Collections;
using System.Collections.Generic;
using UnityEngine;




// Accepts a running value and returns it normalized
// Useful for rewards that should range from 0-1 but do not do so exactly
public class Normalizer 
{
    float min = float.MaxValue;
    float max = float.MinValue;
    //int numSeen = 0;
    public Normalizer() { }

    public float getNormalized(float val)
    {
        min = Mathf.Min(min, val);
        max = Mathf.Max(max, val);
        //numSeen++;
        return (val - min) / (max - min + float.Epsilon);
    }
}
