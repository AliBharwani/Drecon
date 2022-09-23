using System;
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


    private Vector3 prev_kin_cm;
    private Vector3 prev_sim_cm;

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

    Animated character body positions, velocities, CM velocity, and
    desired CM velocity are resolved in reference frame F(kin), which
    is formed from the horizontal heading of the kinematic character,
    the gravity direction, and positioned at the character CM
        -> Means multiply those by inverse of kin root bone rotation

    The exact same procedure is used for the simulated character positions and
    velocities, except with a frame F(sim), which is identical to F(kin) in
    orientation but positioned at the simulated character’s CM position.
        -> Means multiply by inverse of kin root bone rotation but keep sim position 
     
     
     */
    void get_state()
    {
        // kinematic character center of mass

        // simulated character center of mass
        
    }

    Vector3 get_kinematic_cm()
    {

        // We start at 1 because 0 is the root bone with no colliders
        // to calculate CM, we get the masses and centers of each capsule and
        // sum them together and divide by the total mass
        float total_mass = 0f;
        Vector3 CoM = Vector3.zero;
        for (int i = 1; i < kin_bone_to_transform.Length; i++)
        {
            Transform t = kin_bone_to_transform[i];
            float mass = t.GetComponent<ArticulationBody>().mass;
            Vector3 child_center = getChildColliderCenter(t.gameObject);
            CoM += mass * child_center;
            total_mass += mass;

        }
        return CoM / total_mass;
    }

    private Vector3 getChildColliderCenter(GameObject child)
    {
        foreach (Transform grandchild in child.transform)
        {
            if (grandchild.GetComponent<CapsuleCollider>() != null) { 
                Vector3 center = grandchild.GetComponent<CapsuleCollider>().center;
                return grandchild.TransformPoint(center);
            }
            if (grandchild.GetComponent<BoxCollider>() != null) {
                Vector3 center = grandchild.GetComponent<BoxCollider>().center;
                return grandchild.TransformPoint(center);
            }

        }
        return Vector3.zero;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
