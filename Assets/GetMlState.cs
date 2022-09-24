using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;

public class GetMlState : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject kinematic_char;
    private Transform kinematic_char_trans;
    public GameObject sim_char;
    private Transform sim_char_trans;

    private Transform[] kin_bone_to_transform;
    private Transform[] sim_bone_to_transform;
    private GameObject[] kin_bone_to_collider;
    private GameObject[] sim_bone_to_collider;
    private mm_v2 MMScript;
    private SimCharController SimCharController; 

    private ArticulationBody sim_hip_bone; // root of ArticulationBody

    private Vector3 prev_kin_cm;
    private Vector3 prev_sim_cm;
    private Vector3 prev_sim_pos;
    private mm_v2.Bones[] state_bones = new mm_v2.Bones[] {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    private Vector3[] prev_kin_bone_local_pos;
    private Vector3[] prev_sim_bone_local_pos;

    void initalize()
    {
        MMScript = kinematic_char.GetComponent<mm_v2>();
        kinematic_char_trans = kinematic_char.transform;
        kin_bone_to_transform = MMScript.boneToTransform;

        SimCharController = sim_char.GetComponent<SimCharController>();
        sim_char_trans = sim_char.transform;
        sim_bone_to_transform = SimCharController.boneToTransform;
        sim_hip_bone = SimCharController.bone_to_art_body[(int)Bone_Hips];

        prev_kin_bone_local_pos = new Vector3[state_bones.Length];
        prev_sim_bone_local_pos = new Vector3[state_bones.Length];

        for (int i = 0; i < kin_bone_to_transform.Length; i++)
        {
            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)
            {

                kin_bone_to_collider[i] = UpdateJointPositions.getChildBoxCollider(kin_bone_to_transform[i].gameObject);
                sim_bone_to_collider[i] = UpdateJointPositions.getChildBoxCollider(sim_bone_to_transform[i].gameObject);
            }
            kin_bone_to_collider[i] = UpdateJointPositions.getChildCapsuleCollider(kin_bone_to_transform[i].gameObject);
            sim_bone_to_collider[i] = UpdateJointPositions.getChildCapsuleCollider(sim_bone_to_transform[i].gameObject);

        }
    }

    void Start()
    {
        initalize();
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

    Note that when velocities are decomposed into F(kin) or F(sim), the
    reference frames are considered to have no angular or linear velocity
    and acceleration so that global velocity features are measurable in
    the state.
     
     */
    double[] get_state()
    {
        double[] state = new double[110];
        int state_idx = 0;

        // kinematic character center of mass
        Vector3 kin_cm = get_kinematic_cm();
        Vector3 kin_cm_vel = (kin_cm - prev_kin_cm) / Time.deltaTime;
        kin_cm_vel = resolve_vel_in_kim_ref_frame(kin_cm_vel);
        prev_kin_cm = kin_cm;
        copy_vector_into_state(ref state, ref state_idx, kin_cm_vel);

        // simulated character center of mass
        Vector3 sim_cm = sim_hip_bone.worldCenterOfMass;
        Vector3 sim_cm_vel = (sim_cm - prev_sim_cm) / Time.deltaTime;
        sim_cm_vel = resolve_vel_in_kim_ref_frame(sim_cm_vel);
        prev_sim_cm = sim_cm;
        copy_vector_into_state(ref state, ref state_idx, sim_cm_vel);

        // Copy v(sim) - v(kin)
        copy_vector_into_state(ref state, ref state_idx, sim_cm_vel - kin_cm_vel);


        // The desired horizontal CM velocity from user-input is also considered v(des) - R^2
        Vector2 desired_vel = new Vector2(MMScript.desired_velocity.x, MMScript.desired_velocity.z);
        desired_vel = resolve_vel_in_kim_ref_frame(desired_vel);

        copy_vector_into_state(ref state, ref state_idx, desired_vel);


        //The diff between current simulated character horizontal
        //CM velocity and v(des) = v(diff) R ^ 2
        Vector3 cur_sim_vel = (sim_char_trans.position - prev_sim_pos) / Time.deltaTime;
        cur_sim_vel = resolve_vel_in_kim_ref_frame(cur_sim_vel);
        Vector2 v_diff = new Vector2(desired_vel.x - cur_sim_vel.x, desired_vel.y - cur_sim_vel.z);
        prev_sim_pos = sim_char_trans.position;
        copy_vector_into_state(ref state, ref state_idx, v_diff);

        // we do it once for kin char and once for sim char
        Transform[] bone_to_transform = kin_bone_to_transform;
        Vector3 relative_cm = kin_cm;
        Quaternion relative_rot = kinematic_char_trans.rotation;
        Vector3[] prev_bone_local_pos = prev_kin_bone_local_pos;
        double[] s_sim, s_kin;
        s_sim = new double[36];
        s_kin = new double[36];
        int copy_idx = 0;
        double[] copy_into = s_kin;
        for (int i = 0; i < 2; i++) {
            for(int j = 0; j < state_bones.Length; j++)
            {
                mm_v2.Bones bone = state_bones[j];
                // Compute position of bone
                Vector3 bone_world_pos = bone_to_transform[(int)bone].position;
                Vector3 bone_local_pos = bone_world_pos - relative_cm;
                Vector3 bone_relative_pos = Utils.quat_inv_mul_vec3(relative_rot, bone_local_pos);
                Vector3 prev_bone_pos = prev_bone_local_pos[j];
                Vector3 bone_vel = (bone_relative_pos - prev_bone_pos) / Time.deltaTime;
                copy_vector_into_state(ref copy_into, ref copy_idx, bone_relative_pos);
                copy_vector_into_state(ref copy_into, ref copy_idx, bone_vel);
                prev_bone_local_pos[j] = bone_relative_pos;

            }
            // Reset for second loop run with sim car
            bone_to_transform = sim_bone_to_transform;
            relative_cm = sim_cm;
            //relative_rot = sim_char_trans.rotation;
            prev_bone_local_pos = prev_sim_bone_local_pos;
            copy_into = s_sim;
            copy_idx = 0;
        }

        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then s(kin)
        for (int i = 0; i < 36; i++)
            state[state_idx++] = s_sim[i];
        for (int i = 0; i < 36; i++)
            state[state_idx++] = s_sim[i] - s_kin[i];

        if (state_idx != 90)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything but smootehd actions");

        // TODO: Add actions here once actions are being output 
        return state;

    }

    // Gets CoM in world position
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

    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    Vector3 resolve_vel_in_kim_ref_frame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinematic_char_trans.rotation, vel);
    }

    void copy_vector_into_state(ref double[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    void copy_vector_into_state(ref double[] state, ref int start_idx, Vector2 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        start_idx += 2;
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

    void calculate_position_reward()
    {
        // Position reward
        for (int i = 1; i < 23; i++)
        {
            // Get bone, transform, child capsule object
            mm_v2.Bones bone = (mm_v2.Bones)i;
            Transform kin_bone = kin_bone_to_transform[i];
            Transform sim_bone = sim_bone_to_transform[i];
            if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
            {
                // do special stuff
            }
            GameObject kin_capsule_obj = kin_bone_to_collider[i];
            GameObject sim_capsule_obj = sim_bone_to_collider[i];

            // Get 6 points on capsule object
            // first we start off at the center, and the 6 points are given by
            // center plus-minus radius on y dimension
            // center plus-minus radius on z dimension
            // and since capsule is oriented on x dimension, the tippy tops are,
            // as we previously calculated,
            // center plus minus height/2 on x dimension 
        }
    }
    public static void get_six_points_on_capsule(GameObject obj, ref Vector3[] outputs)
    {
        CapsuleCollider cap = obj.GetComponent<CapsuleCollider>();
        if (cap == null)
            throw new Exception($"Object you're trying to get six points of capsule on does not have capsule: {obj.name}");
        if (outputs.Length != 6)
            throw new Exception($"outputs should have length 6, actual length: {outputs.Length}");

        Vector3 center = cap.center;
        float rad = cap.radius;
        float half_height = cap.height / 2;
        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = Vector3.zero;
            if (i == 0)
                axis = Vector3.right;
            else if (i == 1)
                axis = Vector3.up;
            else if (i == 2)
                axis = Vector3.forward;

            // Cap direction corresponds to which dimension it has its height along and is important 
            // Use half height with Vector3.right because capsules are aligned on X axis
            float variable = i == cap.direction ? half_height : rad;
            outputs[i * 2] = obj.transform.TransformPoint(center + variable * axis);
            outputs[i * 2 + 1] = obj.transform.TransformPoint(center - variable * axis);
        }

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Testing and internal stuff
    [ContextMenu("Create gizmos for capsule position points")]
    private void debug_capsule_surface_points()
    {
        initalize();
        for (int i = 1; i < 23; i++)
        {
            // Get bone, transform, child capsule object
            mm_v2.Bones bone = (mm_v2.Bones)i;
            Transform kin_bone = kin_bone_to_transform[i];
            if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
            {
                continue;
                // do special stuff
            }
            GameObject kin_capsule_obj = kin_bone_to_collider[i];
            CapsuleCollider caps = kin_capsule_obj.GetComponent<CapsuleCollider>();
            Vector3[] gizmos = new Vector3[6];
            get_six_points_on_capsule(kin_capsule_obj, ref gizmos);
            foreach (Vector3 v in gizmos)
                add_gizmo_sphere(v);
            // Get 6 points on capsule object
            // first we start off at the center, and the 6 points are given by
            // center plus-minus radius on y dimension
            // center plus-minus radius on z dimension
            // and since capsule is oriented on x dimension, the tippy tops are,
            // as we previously calculated,
            // center plus minus height/2 on x dimension 
        }
    }

    public float gizmoSphereRad = .01f;
    private List<Vector3> gizmo_spheres;

    [ContextMenu("Reset gizmos")]
    private void reset_gizmos()
    {
        gizmo_spheres = null;
    }
    private void add_gizmo_sphere(Vector3 center)
    {
        if (gizmo_spheres == null)
            gizmo_spheres = new List<Vector3>();
        gizmo_spheres.Add(center);
    }
    private void OnDrawGizmos()
    {
        if (gizmo_spheres == null)
            return;
        Gizmos.color = Color.cyan;
        foreach (Vector3 sphere in gizmo_spheres)
        {
            Gizmos.DrawSphere(sphere, gizmoSphereRad);
        }

    }
}
