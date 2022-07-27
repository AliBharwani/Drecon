using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class mm_v2 : MonoBehaviour
{

    database motionDB;

    void Start()
    {
        motionDB = new database(Application.dataPath + @"/outputs/database.bin");
    }

    void Update()
    {
        
    }
}
