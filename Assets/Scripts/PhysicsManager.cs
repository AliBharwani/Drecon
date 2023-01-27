using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsManager : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Physics.autoSimulation = false;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Physics.Simulate(Time.fixedDeltaTime);
    }
}
