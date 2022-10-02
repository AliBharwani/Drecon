using System;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
using System.Collections;

struct char_info
{
    public Transform[]  bone_to_transform;
    public GameObject[] bone_to_collider;
    public GameObject char_obj;
    public Transform  char_trans;
    public ArticulationBody hip_bone;
    public Vector3 cm; // prev center of mass
    public Vector3 cm_vel;
    public Vector3 pos;
    public Vector3[] bone_local_pos;
    public Vector3[][] bone_surface_pts;
    public ArticulationBody[] bone_to_art_body;
}
public class MLAgentsDirector : Agent
{
    public float ACTION_STIFFNESS_HYPERPARAM = .2f;
    public bool use_deltatime = false;
    float frametime = 1f / 30f;
    private bool is_initalized;
    public bool normalize_observations = false;
    char_info kin_char, sim_char;
    public GameObject kinematic_char;
    public GameObject simulated_char;
    public GameObject simulated_char_prefab;
    public GameObject kinematic_char_prefab;

    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies;
    float[] prev_action_output = new float[25];
    database motionDB;

    //private ArticulationBody sim_hip_bone; // root of ArticulationBody


    [HideInInspector]
    public static mm_v2.Bones[] state_bones = new mm_v2.Bones[] 
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] full_dof_bones = new mm_v2.Bones[]
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine};
    [HideInInspector]
    public static mm_v2.Bones[] limited_dof_bones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftToe, Bone_RightToe };
    [HideInInspector]
    public static mm_v2.Bones[] openloop_bones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand};

    Vector3[] bone_pos_mins, bone_pos_maxes, bone_vel_mins, bone_vel_maxes;
    public override void CollectObservations(VectorSensor sensor)
    {
        if (!is_initalized)
        {
            my_initalize();
            return;
        }
        double[] cur_state = get_state();
        foreach (double d in cur_state)
            sensor.AddObservation((float) d);
    }

    // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!is_initalized)
        {
            my_initalize();
            return;
        }
        float[] cur_actions = actionBuffers.ContinuousActions.Array;
       float[] final_actions = new float[25];
       for (int i = 0; i < 25; i++)
           final_actions[i] = ACTION_STIFFNESS_HYPERPARAM * cur_actions[i] + (1 - ACTION_STIFFNESS_HYPERPARAM) * prev_action_output[i];
       prev_action_output = final_actions;
       Quaternion[] cur_rotations = MMScript.bone_rotations;
       for (int i = 0; i < full_dof_bones.Length; i ++)
       {
            int bone_idx = (int)full_dof_bones[i];
            Vector3 scaled_angleaxis = new Vector3(final_actions[i*3], final_actions[i*3 + 1], final_actions[i*3 + 2]);
            float angle = scaled_angleaxis.magnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            angle = (angle - 1.5f) * 120;
            scaled_angleaxis.Normalize();
            Quaternion offset = Quaternion.AngleAxis(angle, scaled_angleaxis);
            Quaternion final = cur_rotations[bone_idx] * offset;
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            ab.SetDriveRotation(final);
       }
        for (int i = 0; i < limited_dof_bones.Length; i++)
        {
            int bone_idx = (int)limited_dof_bones[i];
            // Angle is in range (-1, 1) => map to (-180, 180)
            float angle = final_actions[i] * 180;
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[bone_idx]);
            bool use_xdrive = limited_dof_bones[i] ==  Bone_LeftLeg || limited_dof_bones[i] == Bone_RightLeg;
            if (use_xdrive)
            {
                ArticulationDrive drive = ab.xDrive;
                drive.target = target.x + angle;
                ab.xDrive = drive;
            } else
            {
                ArticulationDrive drive = ab.zDrive;
                drive.target = target.z + angle;
                ab.zDrive = drive;
            }
        }
        for (int i = 0; i < openloop_bones.Length; i++)
        {
            int bone_idx = (int)openloop_bones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            ab.SetDriveRotation(final);
        }
    }
    public override void Heuristic(in ActionBuffers actionsout)
    {
        if (!is_initalized)
        {
            my_initalize();
            return;
        }
        float[] cur_actions = actionsout.ContinuousActions.Array;
        float[] final_actions = new float[25];
        for (int i = 0; i < 25; i++)
            final_actions[i] = ACTION_STIFFNESS_HYPERPARAM * cur_actions[i] + (1 - ACTION_STIFFNESS_HYPERPARAM) * prev_action_output[i];
        prev_action_output = final_actions;
        Quaternion[] cur_rotations = MMScript.bone_rotations;
        for (int i = 0; i < full_dof_bones.Length; i++)
        {
            int bone_idx = (int)full_dof_bones[i];
            Vector3 scaled_angleaxis = new Vector3(final_actions[i * 3], final_actions[i * 3 + 1], final_actions[i * 3 + 2]);
            float angle = scaled_angleaxis.magnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            angle = (angle - 1.5f) * 120;
            scaled_angleaxis.Normalize();
            Quaternion offset = Quaternion.AngleAxis(angle, scaled_angleaxis);
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            ab.SetDriveRotation(final);
        }
        for (int i = 21; i < limited_dof_bones.Length; i++)
        {
            int bone_idx = (int)limited_dof_bones[i];
            // Angle is in range (-1, 1) => map to (-180, 180)
            float angle = final_actions[i] * 180;
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[bone_idx]);
            bool use_xdrive = limited_dof_bones[i] == Bone_LeftLeg || limited_dof_bones[i] == Bone_RightLeg;
            if (use_xdrive)
            {
                ArticulationDrive drive = ab.xDrive;
                drive.target = target.x;
                ab.xDrive = drive;
            }
            else
            {
                ArticulationDrive drive = ab.zDrive;
                drive.target = target.z;
                ab.zDrive = drive;
            }
        }
    }
    void my_initalize()
    {
        MMScript = kinematic_char.GetComponent<mm_v2>();
        if (!MMScript.is_initalized)
            return;
        kin_char = new char_info();
        kin_char.char_trans = kinematic_char.transform;
        kin_char.bone_to_transform = MMScript.boneToTransform;
        kin_char.char_obj = kinematic_char;
        nbodies = kin_char.bone_to_transform.Length;
        kin_char.bone_surface_pts = new Vector3[nbodies][];

        SimCharController = simulated_char.GetComponent<SimCharController>();
        sim_char = new char_info();
        sim_char.char_trans = simulated_char.transform;
        sim_char.bone_to_transform = SimCharController.boneToTransform;
        sim_char.hip_bone = SimCharController.bone_to_art_body[(int)Bone_Hips];
        sim_char.char_obj = simulated_char;
        sim_char.bone_surface_pts = new Vector3[nbodies][];
        sim_char.bone_to_art_body = SimCharController.bone_to_art_body;

        kin_char.bone_local_pos = new Vector3[state_bones.Length];
        sim_char.bone_local_pos = new Vector3[state_bones.Length];

        kin_char.bone_to_collider = new GameObject[nbodies];
        sim_char.bone_to_collider = new GameObject[nbodies];
        for (int i = 0; i < nbodies; i++)
        {
            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)
            {
                kin_char.bone_to_collider[i] = UpdateJointPositions.getChildBoxCollider(kin_char.bone_to_transform[i].gameObject);
                sim_char.bone_to_collider[i] = UpdateJointPositions.getChildBoxCollider(sim_char.bone_to_transform[i].gameObject);
            } else { 
                kin_char.bone_to_collider[i] = UpdateJointPositions.getChildCapsuleCollider(kin_char.bone_to_transform[i].gameObject);
                sim_char.bone_to_collider[i] = UpdateJointPositions.getChildCapsuleCollider(sim_char.bone_to_transform[i].gameObject);
            }
            kin_char.bone_surface_pts[i] = new Vector3[6];
            sim_char.bone_surface_pts[i] = new Vector3[6];
        }

        origin = kin_char.char_trans.position;
        origin_hip_rot = sim_char.bone_to_transform[(int)Bone_Hips].rotation;
        is_initalized = true;
    }
    public void Awake()
    {
        Debug.Log("MLAgents Director Awake called");
        // motionDB = new database(Application.dataPath + @"/outputs/database.bin");
        if (normalize_observations)
        {
            SimCharController.find_mins_and_maxes(kin_char.bone_to_transform, ref MIN_VELOCITY, ref MAX_VELOCITY, ref bone_pos_mins, ref bone_pos_maxes, ref bone_vel_mins, ref bone_vel_maxes);
        }
        kinematic_char = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("OnEpisodeBegin() called");
        if (motionDB == null) {
            Debug.Log("OnEpisodeBegin motionDB is null");
            motionDB = new database(Application.dataPath + @"/outputs/database.bin");
        }
        is_initalized = false;
        //if (kinematic_char != null)
        //    Destroy(kinematic_char);
        if (simulated_char != null)
            Destroy(simulated_char);
        // Setup the kinematic character
        // kinematic_char = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
        // kinematic_char.GetComponent<mm_v2>().motionDB = motionDB;
        // Setup the sim character
        if (kinematic_char != null)
        {
            simulated_char = Instantiate(simulated_char_prefab, kinematic_char.transform.position, kinematic_char.transform.rotation);
            ArticulationBody sim_hip = simulated_char.GetComponent<SimCharController>().bone_to_art_body[(int)Bone_Hips];
            Transform[] kin_bone_to_transform = kinematic_char.GetComponent<mm_v2>().boneToTransform;
            sim_hip.TeleportRoot(sim_hip.gameObject.transform.position, kin_bone_to_transform[(int)Bone_Hips].localRotation);
        } else
        {
            simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        }
        // simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        my_initalize();
    }
    Vector3 origin;
    Quaternion origin_hip_rot;

    private void FixedUpdate()
    {
        if (!is_initalized) { 
            my_initalize();
            return;
        }
        // Make sure to teleport sim character if kin character teleported
        bool teleport_sim = MMScript.teleported_last_frame;
        if (teleport_sim)
        {
            sim_char.char_trans.rotation = kin_char.char_trans.rotation;
            sim_char.hip_bone.TeleportRoot(origin, kin_char.bone_to_transform[(int)Bone_Hips].rotation);
        }
        // request Decision
        RequestDecision();
        set_rewards();
    }

    private void set_rewards()
    {
        bool heads_1m_apart;
        double pos_reward, vel_reward, local_pose_reward, cm_vel_reward, fall_factor;
        calc_fall_factor(out fall_factor, out heads_1m_apart);
        if (heads_1m_apart)
        {
            SetReward(0f);
            //Debug.Log($"Reward is {0}");
            EndEpisode();
        }
        calculate_pos_and_vel_reward(out pos_reward, out vel_reward);
        calc_local_pose_reward(out local_pose_reward);
        calc_cm_vel_reward(out cm_vel_reward);
        // generated reward
        float final_reward = (float)(fall_factor * (pos_reward + vel_reward + local_pose_reward + cm_vel_reward));
        //Debug.Log($"fall_factor: {fall_factor}, pos_reward: {pos_reward}, vel_reward: {vel_reward}, local_pose_reward: {local_pose_reward}, cm_vel_reward: {cm_vel_reward}");
        //Debug.Log($"final_reward: {final_reward}");

        SetReward(final_reward);
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
    // Roughly 6.81 m/s
    public Vector3 MAX_VELOCITY = new Vector3(4.351938f, 1.454015f, 5.032811f);
    public Vector3 MIN_VELOCITY = new Vector3(-4.351710f, -1.688771f, -4.900798f);
    Vector3 normalize_vel_vector(Vector3 vel)
    {
        float new_x = normalize_float(vel.x, MIN_VELOCITY.x, MAX_VELOCITY.x);
        float new_y = normalize_float(vel.y, MIN_VELOCITY.y, MAX_VELOCITY.y);
        float new_z = normalize_float(vel.z, MIN_VELOCITY.z, MAX_VELOCITY.z);
        return new Vector3(new_x, new_y, new_z);
    }
    Vector2 MIN_DESIRED_SPEED = new Vector2(-2.5f, -2f);
    Vector2 MAX_DESIRED_SPEED = new Vector2(2.5f, 3f);

    Vector2 normalize_desired_vel_vector(Vector2 vel)
    {
        float new_x = normalize_float(vel.x, MIN_DESIRED_SPEED.x, MAX_DESIRED_SPEED.x);
        float new_y = normalize_float(vel.y, MIN_DESIRED_SPEED.y, MAX_DESIRED_SPEED.y);
        return new Vector2(new_x, new_y);
    }
    Vector3 normalize_bone_pos(Vector3 pos, int idx)
    {
        float new_x = normalize_float(pos.x, bone_pos_mins[idx].x, bone_pos_maxes[idx].x);
        float new_y = normalize_float(pos.y, bone_pos_mins[idx].y, bone_pos_maxes[idx].y);
        float new_z = normalize_float(pos.z, bone_pos_mins[idx].z, bone_pos_maxes[idx].z);
        return new Vector3(new_x, new_y, new_z);
    }
    Vector3 normalize_bone_vel(Vector3 vel, int idx)
    {
        float new_x = normalize_float(vel.x, bone_vel_mins[idx].x, bone_vel_maxes[idx].x);
        float new_y = normalize_float(vel.y, bone_vel_mins[idx].y, bone_vel_maxes[idx].y);
        float new_z = normalize_float(vel.z, bone_vel_mins[idx].z, bone_vel_maxes[idx].z);
        return new Vector3(new_x, new_y, new_z);
    }
    float normalize_float(float f, float min, float max)
    {
        return (f - min) / (max - min + float.Epsilon);
    }
    double[] get_state()
    {
        double[] state = new double[110];
        int state_idx = 0;

        // kinematic character center of mass
        Vector3 new_kin_cm = get_kinematic_cm(kin_char.bone_to_transform);
        Vector3 kin_cm_vel = (new_kin_cm - kin_char.cm) / deltatime();
        kin_cm_vel = resolve_vel_in_kin_ref_frame(kin_cm_vel);
        if (normalize_observations)
            kin_cm_vel = normalize_vel_vector(kin_cm_vel);
        kin_char.cm_vel = kin_cm_vel;
        kin_char.cm = new_kin_cm;
        copy_vector_into_arr(ref state, ref state_idx, kin_cm_vel);

        // simulated character center of mass
        Vector3 new_sim_cm = sim_char.hip_bone.worldCenterOfMass;
        Vector3 sim_cm_vel = (new_sim_cm - sim_char.cm) / deltatime();
        if (normalize_observations)
            sim_cm_vel = normalize_vel_vector(sim_cm_vel);
        sim_cm_vel = resolve_vel_in_kin_ref_frame(sim_cm_vel);
        sim_char.cm_vel = sim_cm_vel;

        sim_char.cm = new_sim_cm;
        copy_vector_into_arr(ref state, ref state_idx, sim_cm_vel);

        // Copy v(sim) - v(kin)
        copy_vector_into_arr(ref state, ref state_idx, sim_cm_vel - kin_cm_vel);


        // The desired horizontal CM velocity from user-input is also considered v(des) - R^2
        Vector2 desired_vel = new Vector2(MMScript.desired_velocity.x, MMScript.desired_velocity.z);
        desired_vel = resolve_vel_in_kin_ref_frame(desired_vel);
        if (normalize_observations)
            desired_vel = normalize_desired_vel_vector(desired_vel);
        copy_vector_into_arr(ref state, ref state_idx, desired_vel);


        //The diff between current simulated character horizontal
        //CM velocity and v(des) = v(diff) R ^ 2
        Vector3 cur_sim_vel = (sim_char.char_trans.position - sim_char.pos) / deltatime();
        cur_sim_vel = resolve_vel_in_kin_ref_frame(cur_sim_vel);
        if (normalize_observations)
            cur_sim_vel = normalize_vel_vector(cur_sim_vel);
        Vector2 v_diff = new Vector2(desired_vel.x - cur_sim_vel.x, desired_vel.y - cur_sim_vel.z);
        sim_char.pos = sim_char.char_trans.position;
        copy_vector_into_arr(ref state, ref state_idx, v_diff);

        // we do it once for kin char and once for sim char
        char_info cur_char = kin_char;
        //Transform[] bone_to_transform = kin_char.bone_to_transform;
        //Vector3 relative_cm = kin_cm;
        //Quaternion relative_rot = kin_char.char_trans.rotation;
        //Vector3[] prev_bone_local_pos = prev_kin_bone_local_pos;
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
                Vector3 bone_world_pos = cur_char.bone_to_transform[(int)bone].position;
                Vector3 bone_local_pos = i == 0 ? resolve_pos_in_kin_ref_frame(bone_world_pos) : resolve_pos_in_sim_ref_frame(bone_world_pos);
                //Vector3 bone_relative_pos = Utils.quat_inv_mul_vec3(relative_rot, bone_local_pos);
                Vector3 prev_bone_pos = cur_char.bone_local_pos[j];
                Vector3 bone_vel = (bone_local_pos - prev_bone_pos) / deltatime();
                if (normalize_observations)
                {
                    bone_local_pos = normalize_bone_pos(bone_local_pos, j);
                    bone_vel = normalize_bone_vel(bone_vel, j);
                }
                copy_vector_into_arr(ref copy_into, ref copy_idx, bone_local_pos);
                copy_vector_into_arr(ref copy_into, ref copy_idx, bone_vel);
                cur_char.bone_local_pos[j] = bone_local_pos;

            }
            // Reset for second loop run with sim car
            cur_char = sim_char;
            copy_into = s_sim;
            copy_idx = 0;
        }

        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then s(kin)
        for (int i = 0; i < 36; i++)
            state[state_idx++] = s_sim[i];
        for (int i = 0; i < 36; i++)
            state[state_idx++] = s_sim[i] - s_kin[i];
        for (int i = 0; i < 25; i++)
            state[state_idx++] = prev_action_output[i];

        if (state_idx != 110)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything but smootehd actions");

        return state;

    }

    // Gets CoM in world position
    Vector3 get_kinematic_cm(Transform[] bone_to_transform)
    {
        // We start at 1 because 0 is the root bone with no colliders
        // to calculate CM, we get the masses and centers of each capsule and
        // sum them together and divide by the total mass
        float total_mass = 0f;
        Vector3 CoM = Vector3.zero;
        for (int i = 1; i < bone_to_transform.Length; i++)
        {
            Transform t = bone_to_transform[i];
            float mass = t.GetComponent<ArticulationBody>().mass;
            Vector3 child_center = get_child_collider_center(t.gameObject);
            CoM += mass * child_center;
            total_mass += mass;

        }
        return CoM / total_mass;
    }
    float deltatime()
    {
        return use_deltatime ? Time.deltaTime : frametime;
    }
    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    Vector3 resolve_vel_in_kin_ref_frame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kin_char.char_trans.rotation, vel);
    }
    Vector3 resolve_pos_in_kin_ref_frame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kin_char.char_trans.rotation, pos - kin_char.cm);
    }
    Vector3 resolve_pos_in_sim_ref_frame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kin_char.char_trans.rotation, pos - sim_char.cm);
    }

    void copy_vector_into_arr(ref double[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    void copy_vector_into_arr(ref double[] state, ref int start_idx, Vector2 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        start_idx += 2;
    }

    public static Vector3 get_child_collider_center(GameObject child)
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

    void calculate_pos_and_vel_reward(out double pos_reward, out double vel_reward)
    {
        // Position reward
        double pos_diffs_sum = 0f;
        double vel_diffs_sum = 0f;
        for (int i = 1; i < 23; i++)
        {
            GameObject kin_collider_obj = kin_char.bone_to_collider[i];
            GameObject sim_collider_obj = sim_char.bone_to_collider[i];

            Vector3[] new_kin_bone_surface_pts = new Vector3[6];
            Vector3[] new_sim_bone_surface_pts = new Vector3[6];


            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)
            {
                // do special stuff
                get_six_points_on_box(kin_collider_obj, ref new_kin_bone_surface_pts);
                get_six_points_on_box(sim_collider_obj, ref new_sim_bone_surface_pts);
            } else
            {
                get_six_points_on_capsule(kin_collider_obj, ref new_kin_bone_surface_pts);
                get_six_points_on_capsule(sim_collider_obj, ref new_sim_bone_surface_pts);
            }

            Vector3[] prev_kin_surface_pts = kin_char.bone_surface_pts[i];
            Vector3[] prev_sim_surface_pts = sim_char.bone_surface_pts[i];

            for (int j = 0; j < 6; j++)
            {
                new_kin_bone_surface_pts[j] = resolve_pos_in_kin_ref_frame(new_kin_bone_surface_pts[j]);
                new_sim_bone_surface_pts[j] = resolve_pos_in_sim_ref_frame(new_sim_bone_surface_pts[j]);

                pos_diffs_sum += (new_kin_bone_surface_pts[j] - new_sim_bone_surface_pts[j]).magnitude;
                Vector3 kin_surface_pt_vel = (new_kin_bone_surface_pts[j] - prev_kin_surface_pts[j]) / deltatime();
                Vector3 sim_surface_pt_vel = (new_sim_bone_surface_pts[j] - prev_sim_surface_pts[j]) / deltatime();
                vel_diffs_sum += (kin_surface_pt_vel - sim_surface_pt_vel).magnitude;
            }
            kin_char.bone_surface_pts[i] = new_kin_bone_surface_pts;
            sim_char.bone_surface_pts[i] = new_sim_bone_surface_pts;
        }
        pos_reward = Math.Exp((-10f / nbodies) *  pos_diffs_sum );
        vel_reward = Math.Exp((-1f / nbodies) *  vel_diffs_sum );
    }

    void calc_local_pose_reward(out double pose_reward)
    {
        double pose_reward_sum = 0;
        for (int i = 1; i < 23; i++)
        {
            Transform kin_bone = kin_char.bone_to_transform[i];
            Transform sim_bone = sim_char.bone_to_transform[i];
            // From Stack Overflow:
            //If you want to find a quaternion diff such that diff * q1 == q2, then you need to use the multiplicative inverse:
            // diff * q1 = q2  --->  diff = q2 * inverse(q1)
            Quaternion diff = sim_bone.localRotation * Quaternion.Inverse(kin_bone.localRotation);
            //Vector3 axis;
            // https://stackoverflow.com/questions/21513637/dot-product-of-two-quaternion-rotations
            // angle = 2*atan2(q.vec.length(), q.w)
            //double sqrd_dot =  Math.Pow(Quaternion.Dot(kin_bone.localRotation, sim_bone.localRotation), 2);
            //double angle = Math.Acos(2 * sqrd_dot - 1);
            //diff.ToAngleAxis(out angle, out axis);

            Vector3 diff_vec = new Vector3(diff.x, diff.y, diff.z);
            double angle = 2 * Math.Atan2(diff_vec.magnitude, diff.w) * Mathf.Rad2Deg;
            // We want the magnitude of the diff so we take abs value
            angle = Math.Abs(GeoUtils.wrap_angle(angle));
            //double unity_angle = Quaternion.Angle(sim_bone.localRotation, kin_bone.localRotation);
            //Debug.Log($"Bone: {(mm_v2.Bones)i} Unity Angle: {unity_angle}, My Angle {angle}");
            pose_reward_sum += GeoUtils.wrap_angle(angle);
        }
        pose_reward = Math.Exp((-10f / nbodies) * pose_reward_sum);

    }

    void calc_cm_vel_reward(out double cm_vel_reward)
    {
        cm_vel_reward = Math.Exp(-1d * (kin_char.cm_vel - sim_char.cm_vel).magnitude);
    }

    void calc_fall_factor(out double fall_factor, out bool heads_1m_apart)
    {
        Vector3 kin_head_pos = kin_char.bone_to_transform[(int)Bone_Head].position;
        Vector3 sim_head_pos = sim_char.bone_to_transform[(int)Bone_Head].position;
        float head_distance = (kin_head_pos - sim_head_pos).magnitude;
        heads_1m_apart = head_distance > 1f;
        fall_factor = Math.Clamp(1.3 - 1.4 * head_distance, 0d, 1d);
    }

    // Get 6 points on capsule object
    // first we start off at the center, and the 6 points are given by
    // center plus-minus radius on y dimension
    // center plus-minus radius on z dimension
    // and since capsule is oriented on x dimension, the tippy tops are,
    // as we previously calculated,
    // center plus minus height/2 on x dimension 
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
    public static void get_six_points_on_box(GameObject obj, ref Vector3[] outputs)
    {
        BoxCollider box = obj.GetComponent<BoxCollider>();
        if (box == null)
            throw new Exception($"Object you're trying to get six points of box on does not have box: {obj.name}");
        if (outputs.Length != 6)
            throw new Exception($"outputs should have length 6, actual length: {outputs.Length}");

        Vector3 center = box.center;
        Vector3 size = box.size;

        for (int i = 0; i < 3; i++)
        {
            Vector3 axis = Vector3.zero;
            float move_amount = 0f;
            if (i == 0)
            {
                move_amount = size.x;
                axis = Vector3.right;
            }
            else if (i == 1)
            {
                move_amount = size.y;
                axis = Vector3.up;
            }
            else if (i == 2)
            {
                move_amount = size.z;
                axis = Vector3.forward;
            }
            // Cap direction corresponds to which dimension it has its height along and is important 
            // Use half height with Vector3.right because capsules are aligned on X axis
            outputs[i * 2] = obj.transform.TransformPoint(center + move_amount/2 * axis);
            outputs[i * 2 + 1] = obj.transform.TransformPoint(center - move_amount/2 * axis);
        }

    }

    // Testing and internal stuff
    [ContextMenu("Create gizmos for capsule position points")]
    private void debug_capsule_surface_points()
    {
        my_initalize();
        for (int i = 1; i < 23; i++)
        {
            // Get bone, transform, child capsule object
            mm_v2.Bones bone = (mm_v2.Bones)i;
            GameObject kin_collider_obj = kin_char.bone_to_collider[i];
            Vector3[] gizmos = new Vector3[6];
            if (bone == Bone_LeftFoot || bone == Bone_RightFoot)            
                get_six_points_on_box(kin_collider_obj, ref gizmos);            
            else
                get_six_points_on_capsule(kin_collider_obj, ref gizmos);            
            foreach (Vector3 v in gizmos)
                add_gizmo_sphere(v);
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

    [ContextMenu("Testing")]
    private void test_func()
    {
        Quaternion q1 = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion q2 = Quaternion.AngleAxis(0f, Vector3.right);
        Debug.Log($"{Quaternion.Dot(q1, q2)}");
        Debug.Log($"{Quaternion.Dot(q1, q2)}");

    }
}
