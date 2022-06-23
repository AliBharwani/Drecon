using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

public class MotionMatching : MonoBehaviour
{
    double[] means = new double[] {
        -0.0458957508171,
        0.148889939955,
        -1.98212023408,
        -0.0303706152439,
        0.147394518158,
        -2.00615341094,
        -0.00105541224217,
        0.000328890938174,
        0.0011302420994,
        -0.00187898262142,
        3.8678468464e-05,
        0.00122720085238,
        -0.00165934029416,
        -0.000206693790398,
        0.00128557976763,
        0.898474555729,
        16.7989438667,
        182.212187285,
        188.200992116,
        267.64185031,
        0.896308134437,
        16.8000879095,
        182.354338485,
        188.366045526,
        267.646277596,
        0.893661161189,
        16.8010779027,
        182.299978261,
        188.581469405,
        267.647131713
    };
    double[] std_devs = new double[] { 2.75672803451, 0.0851475658313, 5.39294074082, 2.76605976978, 0.0867144467634, 5.404615269, 1.50215804108, 0.671604269949, 1.99188466153, 1.52159098328, 0.687993202854, 2.0142738406, 1.02034196113, 0.29502827464, 1.43598251437, 2.73879165649, 5.38729029753, 175.25471024, 107.414065461, 8.47444261664, 2.74219568686, 5.40109608084, 175.25317976, 107.38595772, 8.47702118087, 2.74476959155, 5.41456517415, 175.261603812, 107.328262296, 8.47962818468 };

    private void ingestMotionMatchingDB()
    {
        // @ escapes the backslashes
        string pathToData = @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/";

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
        DateTime startTime = DateTime.Now;
        KDTree motionDB = new KDTree();
        int counter = 0;
        foreach (string prefix in prefixes)
        {
            foreach (string line in File.ReadLines(pathToData + prefix + "_normalized_outputs.txt"))
            {
                string[] stringValues = line.Split(',').ToArray();
                motionDB.Add(Array.ConvertAll(stringValues, double.Parse));
                counter++;
            }
        }
        DateTime ingestTime = DateTime.Now;
        Debug.Log("Counter: " + counter.ToString());
        motionDB.Build();
        DateTime buildTime = DateTime.Now;
        Debug.Log("Time to ingest: " + (ingestTime - startTime).Milliseconds);
        Debug.Log("Time to Build: " + (buildTime - ingestTime).Milliseconds);

    }

    private void normalizeVector(double[] vec)
    {
        for(int i = 0; i < 30; i ++)
        {
            vec[i] = (vec[i] - means[i]) / std_devs[i];
        }
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
