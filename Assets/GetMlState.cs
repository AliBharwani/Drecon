using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetMlState : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject kinematic_char;
    public GameObject sim_char;
    private Transform[] kin_bone_to_transform;
    private Transform[] sim_bone_to_transform;
    void Start()
    {

        kin_bone_to_transform = sim_char.GetComponent<mm_v2>().boneToTransform;
        sim_bone_to_transform = sim_char.GetComponent<BVHJointTester>().boneToTransform;
    }

    /*
     At each control step the policy is provided with a state s in R^110
    The state contains:
    Center of Mass velocity in R^3 for kinematic and simulated character

    The desired horizontal CM velocity from user-input is also considered v(des) - R^2

    The diff between current simulated character horizontal
    CM velocity and v(des) = v(diff) R^2


    For a subset of bodies:
    Left Toe, RightToe, Spine, Head, LeftForeArm, RightForeArm
    We compute positions and velocities then concatenate these giving s(sim)
    and s(kin) in R^36

    The smootehd actions produced in the previous step of the policy are collected
    in y(t-1) in R^25

     
     
     
     */
    void get_state()
    {

    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
