using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class AcademyManager : MonoBehaviour
{
    void Awake()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
    }
    
    /* https://forum.unity.com/threads/lifecycle.989227/
     *  During each fixed update, the Academy (the orchestrator) calls CollectObservations on all 
     *  Agents that requested a decision since the last fixed update 
     *  (Note that RequestDecision can be called anytime from anywhere on the Agent).
     *  Once all relevant observations are collected, the data is communicated to Python.
     *  The data includes all observations and all rewards that were collected since last fixed update
     *  (note that AddReward can be called from anywhere anytime on the Agent). 
     *  It also includes some other useful information such as wether or not the Agent terminated 
     *  since the last fixed update.
     *  Python selects an action and sends it to the UnityEnvironment.
     *  The Academy calls OnActionReceived on all of the Agents that requested a decision since the last fixed update.
     *  The rest of the fixed update and other game loop unfolds.
     */
    void FixedUpdate()
    {
        Academy.Instance.EnvironmentStep();
    }
}
