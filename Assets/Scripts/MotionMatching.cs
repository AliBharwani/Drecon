using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

public class MotionMatching : MonoBehaviour
{


    private void ingestMotionMatchingDB()
    {
        // @ escapes the backslashes
        string pathToData = @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/";
        // Lol I have to build a KD tree to load the data into
        string[] prefixes = {
            "walk1_subject1",
            "walk1_subject2",
            "walk1_subject5",
            "walk3_subject1",
            "walk3_subject2",
            "run1_subject2",
            "run1_subject5",
            "run2_subject1",
            "sprint1_subject2",
        };
        KDTree motionDB = new KDTree();
        int counter = 0;
        foreach (string prefix in prefixes)
        {
            foreach (string line in File.ReadLines(pathToData + prefix + "_normalized_outputs.txt"))
            {
                string[] stringValues = line.Split(',').ToArray();
                motionDB.Add(Array.ConvertAll(stringValues, float.Parse));
                counter++;
            }
        }
        Debug.Log("Counter: " + counter.ToString());
        motionDB.Build();

    }
    void Start()
    {
        ingestMotionMatchingDB();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
