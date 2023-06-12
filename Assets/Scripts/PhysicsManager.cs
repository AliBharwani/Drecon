using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsManager : MonoBehaviour
{
    void Start()
    {
        Physics.autoSimulation = false;
    }

    void FixedUpdate()
    {
        Physics.Simulate(Time.fixedDeltaTime);
    }
}
