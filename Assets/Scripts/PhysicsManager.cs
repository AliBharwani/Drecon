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
    public ArticulationBody debugBody;
    public mm_v2 MMScript;
    // Update is called once per frame
    void FixedUpdate()
    {
        //Debug.Log($"{Time.frameCount} : PhysicsManager updated");
        bool debug = MMScript != null && debugBody != null && MMScript.teleportedThisFixedUpdate;
        if (debug)
        {
            List<float> jointVelocities = new List<float>(debugBody.dofCount);
            debugBody.GetJointVelocities(jointVelocities);
            Utils.debugArray(jointVelocities.ToArray(), $"{Time.frameCount} jointForces before physics simulate ");
        }
        //Debug.Log($"Executing physics");
        Physics.Simulate(Time.fixedDeltaTime);
        if (debug)
        {
            List<float> jointVelocities = new List<float>(debugBody.dofCount);
            debugBody.GetJointVelocities(jointVelocities);
            Utils.debugArray(jointVelocities.ToArray(), $"{Time.frameCount} jointForces after physics simulate ");
        }
    }
}
