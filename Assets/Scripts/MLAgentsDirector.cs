using System;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
using System.Collections;
using UnityEditor;

public struct CharInfo
{
    public Transform[]  boneToTransform;
    public GameObject[] boneToCollider;
    public GameObject charObj;
    public Transform  trans;
    public ArticulationBody root;
    public Vector3 cm; // prev center of mass
    public Vector3 cmVel;
    public Vector3[] boneWorldPos;
    public Vector3[][] boneSurfacePts;
    public Vector3[][] boneSurfaceVels;
    public ArticulationBody[] boneToArtBody;
    public float[] boneState;

    public CharInfo(int nbodies, int numStateBones) : this()
    {
        boneSurfacePts = new Vector3[nbodies][];
        boneSurfaceVels = new Vector3[nbodies][];
        boneWorldPos = new Vector3[numStateBones];
        boneToCollider = new GameObject[nbodies];
        // From DreCon paper, contains: 
        // { LeftToe, RightToe, Spine, Head, LeftForeArm, RightForeArm },
        // we compute positions and velocities then concatenate these
        boneState = new float[36];
    }
}
public class MLAgentsDirector : Agent
{
    public float ACTION_STIFFNESS_HYPERPARAM = .2f;
    public bool use_deltatime = false;
    float frametime = 1f / 30f;
    private bool is_initalized;
    public bool normalize_action_ouputs = true;
    public bool normalize_observations = false;
    CharInfo kinChar, simChar;
    public GameObject kinematic_char;
    public GameObject simulated_char;
    public GameObject simulated_char_prefab;
    public GameObject kinematic_char_prefab;
    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies; 
    private int evaluate_k_steps = 1;
    private int cur_step = 0;

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
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand, Bone_LeftShoulder, Bone_RightShoulder};

    Vector3[] bone_pos_mins, bone_pos_maxes, bone_vel_mins, bone_vel_maxes;
    public override void CollectObservations(VectorSensor sensor)
    {
        if (!is_initalized)
        {
            CustomInit();
            return;
        }
        sensor.AddObservation(get_state());
    }
    // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!is_initalized)
        {
            CustomInit();
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
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            if (normalize_action_ouputs)
            {
                Vector3 output = new Vector3(final_actions[i * 3], final_actions[i * 3 + 1], final_actions[i * 3 + 2]);
                Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpaceV2(cur_rotations[bone_idx], true);
                //Vector3 targetRotationInJointSpace = -(Quaternion.Inverse(ab.anchorRotation) * Quaternion.Inverse(cur_rotations[bone_idx]) * ab.parentAnchorRotation).eulerAngles;
                //targetRotationInJointSpace = new Vector3(
                //        Mathf.DeltaAngle(0, targetRotationInJointSpace.x),
                //        Mathf.DeltaAngle(0, targetRotationInJointSpace.y),
                //        Mathf.DeltaAngle(0, targetRotationInJointSpace.z));

                var xdrive = ab.xDrive;
                var scale = (xdrive.upperLimit - xdrive.lowerLimit) / 2f;
                var midpoint = xdrive.lowerLimit + scale;
                //var normalizedTargetX = (targetRotationInJointSpace.x - midpoint) / scale;
                //normalizedTargetX += output.x;
                var outputX = (output.x * scale) + midpoint;
                xdrive.target = targetRotationInJointSpace.x + outputX;
                ab.xDrive = xdrive;

                var ydrive = ab.yDrive;
                 scale = (ydrive.upperLimit - ydrive.lowerLimit) / 2f;
                 midpoint = ydrive.lowerLimit + scale;
                //var normalizedTargetY = (targetRotationInJointSpace.y - midpoint) / scale;
                //normalizedTargetY += output.y;
                var outputY = (output.y * scale) + midpoint;
                ydrive.target = targetRotationInJointSpace.y + outputY ;
                ab.yDrive = ydrive;

                var zdrive = ab.zDrive;
                scale = (zdrive.upperLimit - zdrive.lowerLimit) / 2f;
                midpoint = zdrive.lowerLimit + scale;
                //var normalizedTargetZ = (targetRotationInJointSpace.z - midpoint) / scale;
                //normalizedTargetZ += output.z;
                var outputZ = (output.z  * scale) + midpoint;
                zdrive.target = targetRotationInJointSpace.z + outputZ;
                ab.zDrive = zdrive;
                continue;
            }
            Vector3 scaled_angleaxis = new Vector3(final_actions[i*3], final_actions[i*3 + 1], final_actions[i*3 + 2]);
            float angle = scaled_angleaxis.sqrMagnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            angle = (angle * 120) - 180;
            scaled_angleaxis.Normalize();
            Quaternion offset = Quaternion.AngleAxis(angle, scaled_angleaxis);
            Quaternion final = cur_rotations[bone_idx] * offset;
            ab.SetDriveRotation(final);
       }
        for (int i = 0; i < limited_dof_bones.Length; i++)
        {
            int bone_idx = (int)limited_dof_bones[i];
            bool use_xdrive = limited_dof_bones[i] == Bone_LeftLeg || limited_dof_bones[i] == Bone_RightLeg;
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            int final_actions_idx = i + 21;

            if (normalize_action_ouputs)
            {
                float output = final_actions[final_actions_idx];
                // parent anchor rotation = q(ab) 
                // anchor rotation = q(a) 
                // target local = q(c) 
                // need q(x) s.t. a * x = a * c
                // 
                Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpaceV2(cur_rotations[bone_idx], true);
                if (use_xdrive)
                {
                    //Debug.Log($"{limited_dof_bones[i]} target euler: {targetEuler} | targetRot: {targetRotationInJointSpace} | test: {test} | Current x drive target: {ab.xDrive.target}");
                    var xdrive = ab.xDrive;
                    var scale = (xdrive.upperLimit - xdrive.lowerLimit) / 2f;
                    var midpoint = xdrive.lowerLimit + scale;
                    //var normalizedTargetX = (targetRotationInJointSpace.x - midpoint) / scale;
                    //normalizedTargetX += output.x;
                    var outputX = (output * scale) + midpoint;
                    xdrive.target = targetRotationInJointSpace.x + outputX;
                    ab.xDrive = xdrive;

                } else
                {
                    var zdrive = ab.zDrive;
                    var scale = (zdrive.upperLimit - zdrive.lowerLimit) / 2f;
                    var midpoint = zdrive.lowerLimit + scale;
                    //var normalizedTargetZ = (targetRotationInJointSpace.z - midpoint) / scale;
                    //normalizedTargetZ += output.z;
                    var outputZ = (output * scale) + midpoint;
                    zdrive.target = targetRotationInJointSpace.z + outputZ;
                    ab.zDrive = zdrive;
                }
                continue;
            }
            // Angle is in range (-1, 1) => map to (-180, 180)
            float angle = final_actions[final_actions_idx] * 180;
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[bone_idx]);
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
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            ab.SetDriveRotation(final);
        }
        set_rewards();
    }
    public override void Heuristic(in ActionBuffers actionsout)
    {
        if (!is_initalized)
        {
            CustomInit();
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
            //Vector3 scaled_angleaxis = new Vector3(final_actions[i * 3], final_actions[i * 3 + 1], final_actions[i * 3 + 2]);
            //float angle = scaled_angleaxis.sqrMagnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            //angle = (angle - 1.5f) * 120;
            //scaled_angleaxis.Normalize();
            //Quaternion offset = Quaternion.AngleAxis(angle, scaled_angleaxis);
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            ab.SetDriveRotation(final);
        }
        for (int i = 0; i < limited_dof_bones.Length; i++)
        {
            mm_v2.Bones bone =  limited_dof_bones[i];
            // Angle is in range (-1, 1) => map to (-180, 180)
            //float angle = final_actions[i] * 180;
            ArticulationBody ab = simChar.boneToArtBody[(int)bone];
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[(int) bone]);
            bool use_xdrive = bone == Bone_LeftLeg || bone == Bone_RightLeg;
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
        for (int i = 0; i < openloop_bones.Length; i++)
        {
            int bone_idx = (int)openloop_bones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            ab.SetDriveRotation(final);
        }
    }
    void CustomInit()
    {
        if (use_debug_mats)
        {
            Material RedMatTransparent, WhiteMatTransparent;
#if UNITY_EDITOR
            RedMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/RedMatTransparent.mat", typeof(Material));
            WhiteMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/WhiteMatTransparent.mat", typeof(Material));
#else
            RedMatTransparent = Resources.Load<Material>("RedMatTransparent.mat");
            WhiteMatTransparent = Resources.Load<Material>("WhiteMatTransparent.mat");
#endif
            kinematic_char.GetComponent<ArtBodyTester>().set_all_material(WhiteMatTransparent);
            simulated_char.GetComponent<ArtBodyTester>().set_all_material(RedMatTransparent);
        }
        MMScript = kinematic_char.GetComponent<mm_v2>();
        if (!MMScript.is_initalized)
            return;
        MMScript.use_deltatime = use_deltatime;
        kinChar = new CharInfo(nbodies, state_bones.Length);
        kinChar.trans = kinematic_char.transform;
        kinChar.boneToTransform = MMScript.boneToTransform;
        kinChar.charObj = kinematic_char;
        //nbodies = kinChar.boneToTransform.Length;
        //kinChar.boneSurfacePts = new Vector3[nbodies][];

        SimCharController = simulated_char.GetComponent<SimCharController>();
        SimCharController.is_active = false;
        SimCharController.enabled = true;
        simChar = new CharInfo(nbodies, state_bones.Length);
        simChar.trans = simulated_char.transform;
        simChar.boneToTransform = SimCharController.boneToTransform;
        simChar.root = SimCharController.bone_to_art_body[(int)Bone_Entity];
        simChar.charObj = simulated_char;
        //simChar.boneSurfacePts = new Vector3[nbodies][];
        simChar.boneToArtBody = SimCharController.bone_to_art_body;

        //kinChar.boneWorldPos = new Vector3[state_bones.Length];
        //simChar.boneWorldPos = new Vector3[state_bones.Length];

        //kinChar.boneToCollider = new GameObject[nbodies];
        //simChar.boneToCollider = new GameObject[nbodies];
        for (int i = 0; i < nbodies; i++)
        {
            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)
            {
                kinChar.boneToCollider[i] = ArtBodyTester.getChildBoxCollider(kinChar.boneToTransform[i].gameObject);
                simChar.boneToCollider[i] = ArtBodyTester.getChildBoxCollider(simChar.boneToTransform[i].gameObject);
            } else { 
                kinChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(kinChar.boneToTransform[i].gameObject);
                simChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(simChar.boneToTransform[i].gameObject);
            }
            kinChar.boneSurfacePts[i] = new Vector3[6];
            kinChar.boneSurfaceVels[i] = new Vector3[6];
            simChar.boneSurfacePts[i] = new Vector3[6];
            simChar.boneSurfaceVels[i] = new Vector3[6];
        }

        if (normalize_observations)
            SimCharController.find_mins_and_maxes(kinChar.boneToTransform, ref MIN_VELOCITY, ref MAX_VELOCITY, ref bone_pos_mins, ref bone_pos_maxes, ref bone_vel_mins, ref bone_vel_maxes);
        is_initalized = true;
    }

    public bool use_debug_mats = false;
    public void Awake()
    {
        //Debug.Log("MLAgents Director Awake called");
        //Application.targetFrameRate = 30;
        // motionDB = new database(Application.dataPath + @"/outputs/database.bin");
        //Unity.MLAgents.Policies.BehaviorParameters behavior_params = gameObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (motionDB == null)
            motionDB = database.Instance;
        nbodies = motionDB.nbones();
        kinematic_char = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
        simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        CustomInit();
        //SimCharController.set_art_body_rot_limits();
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log("OnEpisodeBegin() called");
        //if (motionDB == null) 
        //    motionDB = database.Instance;
        //is_initalized = false;
        MMScript.Reset();
        MMScript.FixedUpdate();
        SimCharController.teleport_sim_char(simChar, kinChar);
        return;
    }


    private void FixedUpdate()
    {
        if (!is_initalized) { 
            CustomInit();
            return;
        }
        // Make sure to teleport sim character if kin character teleported
        bool updateVelocity = true;
        if (MMScript.teleported_last_frame)
        {
            //sim_char.char_trans.rotation = kin_char.char_trans.rotation;
            simChar.root.TeleportRoot(Vector3.zero, kinChar.trans.rotation);
            updateVelocity = false;
        }
        float deltaTime = Time.fixedDeltaTime + float.Epsilon;
        // Update CMs 
        UpdateCMData(updateVelocity, deltaTime);
        UpdateBoneObsState(updateVelocity, deltaTime);
        UpdateBoneSurfacePts(updateVelocity, deltaTime);

        //SimCharController.teleport_sim_char(sim_char, kin_char);
        // request Decision
        if (cur_step % evaluate_k_steps == 0)
            RequestDecision();
        cur_step++;
    }

    private void UpdateCMData(bool updateVelocity, float deltaTime)
    {
        Vector3 newSimCM = get_cm(simChar.boneToTransform);
        simChar.cmVel = updateVelocity ? (simChar.cm - newSimCM) / deltaTime : simChar.cmVel;
        simChar.cm = newSimCM;
        Vector3 newKinCM = get_cm(kinChar.boneToTransform);
        kinChar.cmVel = updateVelocity ? (kinChar.cm - newKinCM) / deltaTime : kinChar.cmVel;
        kinChar.cm = newKinCM;
    }

    private void UpdateBoneSurfacePts(bool updateVelocity, float deltaTime)
    {
        for (int i = 1; i < 23; i++)
        {

            Vector3[] new_kin_bone_surface_pts = new Vector3[6];
            Vector3[] new_sim_bone_surface_pts = new Vector3[6];

            getSixPointsOnCollider(kinChar.boneToCollider[i], ref new_kin_bone_surface_pts, (mm_v2.Bones) i);
            getSixPointsOnCollider(simChar.boneToCollider[i], ref new_sim_bone_surface_pts, (mm_v2.Bones) i);

            Vector3[] prev_kin_surface_pts = kinChar.boneSurfacePts[i];
            Vector3[] prev_sim_surface_pts = simChar.boneSurfacePts[i];

            for (int j = 0; j < 6; j++)
            {
                new_kin_bone_surface_pts[j] = resolve_pos_in_kin_ref_frame(new_kin_bone_surface_pts[j]);
                new_sim_bone_surface_pts[j] = resolve_pos_in_sim_ref_frame(new_sim_bone_surface_pts[j]);

                if (updateVelocity) {
                    kinChar.boneSurfaceVels[i][j] = (new_kin_bone_surface_pts[j] - prev_kin_surface_pts[j]) / deltaTime;
                    simChar.boneSurfaceVels[i][j] = (new_sim_bone_surface_pts[j] - prev_sim_surface_pts[j]) / deltaTime;
                }
            }
            kinChar.boneSurfacePts[i] = new_kin_bone_surface_pts;
            simChar.boneSurfacePts[i] = new_sim_bone_surface_pts;
        }
    }

    private void UpdateBoneObsState(bool updateVelocity, float deltaTime)
    {
        CharInfo curInfo = kinChar;
        for (int i = 0; i < 2; i++)
        {
            float[] copyInto = curInfo.boneState;
            int copyIdx = 0;
            for (int j = 0; j < state_bones.Length; j++)
            {
                mm_v2.Bones bone = state_bones[j];
                // Compute position of bone
                Vector3 bone_world_pos = curInfo.boneToTransform[(int)bone].position;
                Vector3 bone_local_pos = i == 0 ? resolve_pos_in_kin_ref_frame(bone_world_pos) : resolve_pos_in_sim_ref_frame(bone_world_pos);
                //Vector3 bone_relative_pos = Utils.quat_inv_mul_vec3(relative_rot, bone_local_pos);
                Vector3 prev_bone_pos = curInfo.boneWorldPos[j];
                Vector3 bone_vel = (bone_world_pos - prev_bone_pos) / deltaTime;
                bone_vel = resolve_vel_in_kin_ref_frame(bone_vel);
                if (normalize_observations)
                {
                    bone_local_pos = normalize_bone_pos(bone_local_pos, j);
                    bone_vel = normalize_bone_vel(bone_vel, j);
                }
                copy_vector_into_arr(ref copyInto, ref copyIdx, bone_local_pos);
                if (updateVelocity)
                    copy_vector_into_arr(ref copyInto, ref copyIdx, bone_vel);
                else
                    copyIdx += 3;
                curInfo.boneWorldPos[j] = bone_world_pos;
            }
            curInfo = simChar;
        }
    }
    private void set_rewards()
    {
        bool heads_1m_apart;
        double pos_reward, vel_reward, local_pose_reward, cm_vel_reward, fall_factor;
        calc_fall_factor(out fall_factor, out heads_1m_apart);
        if (heads_1m_apart)
        {
            SetReward(-.5f);
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

    Vector2 normalize_desired_vel_vector(Vector3 vel)
    {
        float new_x = normalize_float(vel.x, MIN_DESIRED_SPEED.x, MAX_DESIRED_SPEED.x);
        float new_z = normalize_float(vel.z, MIN_DESIRED_SPEED.y, MAX_DESIRED_SPEED.y);
        return new Vector3(new_x, 0f, new_z);
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

    List<(Vector3 pos, Color color)> gizmoSpheres;
    List<(Vector3 a, Vector3 b, Color color)> gizmoLines;

    private void add_gizmo_sphere(Vector3 v, Color c)
    {
        if (gizmoSpheres == null)
            gizmoSpheres = new List<(Vector3, Color)>();
        gizmoSpheres.Add((v,c));
    }
    private void add_gizmo_line(Vector3 a, Vector3 b, Color color)
    {
        if (gizmoLines == null)
            gizmoLines = new List<(Vector3, Vector3, Color)>();
        gizmoLines.Add((a, b, color));
    }
    private void clear_gizmos()
    {
        if (gizmoSpheres != null)
            gizmoSpheres.Clear();
        if (gizmoLines != null)
            gizmoLines.Clear();
    }
    public bool draw_gizmos = false;
    private void OnDrawGizmosSelected()
    {
        if (!draw_gizmos)
            return;
        if (gizmoSpheres != null) { 
            foreach((Vector3 v, Color c) in gizmoSpheres) {
                Gizmos.color = c;
                Gizmos.DrawSphere(v, .1f);
            }
        }

        if (gizmoLines != null)
        {
            foreach ((Vector3 a, Vector3 b, Color color) in gizmoLines)
            {
                Gizmos.color = color;
                Gizmos.DrawLine(a, b);
            }
        }
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
    float[] get_state()
    {
        // Debug.Log("get_state called");
        float[] state = new float[110];
        int state_idx = 0;

        Vector3 kin_cm_vel_normalized = resolve_vel_in_kin_ref_frame(kinChar.cmVel);
        if (normalize_observations)
            kin_cm_vel_normalized = normalize_vel_vector(kin_cm_vel_normalized);
        clear_gizmos();
        copy_vector_into_arr(ref state, ref state_idx, kin_cm_vel_normalized);


        Vector3 sim_cm_vel_normalized = resolve_vel_in_kin_ref_frame(simChar.cmVel);
        if (normalize_observations)
            sim_cm_vel_normalized = normalize_vel_vector(sim_cm_vel_normalized);
        //add_gizmo_sphere(sim_char.cm, Color.blue);
        //add_gizmo_sphere(new_sim_cm, Color.green);
        copy_vector_into_arr(ref state, ref state_idx, sim_cm_vel_normalized);

        // Copy v(sim) - v(kin)
        Vector3 vel_diff = simChar.cmVel - kinChar.cmVel;
        copy_vector_into_arr(ref state, ref state_idx, normalize_observations ? normalize_vel_vector(vel_diff) : vel_diff);


        // The desired horizontal CM velocity from user-input is also considered v(des) - R^2
        Vector3 desired_vel = MMScript.desired_velocity;
        //add_gizmo_line(new_kin_cm, new_kin_cm + desired_vel, Color.blue);
        desired_vel = resolve_vel_in_kin_ref_frame(desired_vel);
        //Vector2 desired_vel = new Vector2(MMScript.desired_velocity.x, MMScript.desired_velocity.z);

        // Since we resolved in kin ref frame, it should point the "wrong way", not in the direction kin moves
        // Red and green lines should overlap when char should be moving straight forward
        //add_gizmo_line(new_kin_cm, new_kin_cm + desired_vel, Color.red);
        //add_gizmo_line(new_kin_cm, new_kin_cm + Vector3.forward, Color.green);
        Vector3 desired_vel_normalized = desired_vel;
        if (normalize_observations)
            desired_vel_normalized = normalize_desired_vel_vector(desired_vel);
        copy_vector_into_arr(ref state, ref state_idx, new Vector2(desired_vel_normalized.x, desired_vel_normalized.z));


        //The diff between current simulated character horizontal
        //CM velocity and v(des) = v(diff) R ^ 2
        Vector3 v_diff = desired_vel_normalized - sim_cm_vel_normalized;
        copy_vector_into_arr(ref state, ref state_idx, new Vector2(v_diff.x, v_diff.z));

        // we do it once for kin char and once for sim char
        CharInfo cur_char = kinChar;
        //Vector3 relative_cm = kin_cm;
        //Quaternion relative_rot = kin_char.char_trans.rotation;
        //Vector3[] prev_bone_local_pos = prev_kin_bone_local_pos;
        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then (s(sim) - s(kin))
        for (int i = 0; i < 36; i++)
            state[state_idx++] = kinChar.boneState[i];
        for (int i = 0; i < 36; i++)
            state[state_idx++] = simChar.boneState[i] - kinChar.boneState[i];
        for (int i = 0; i < 25; i++)
            state[state_idx++] = prev_action_output[i];

        if (state_idx != 110)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything but smootehd actions");
        for (int i = 0; i < 110; i++)
        {
            if (float.IsInfinity(state[i]))
                Debug.Log($"State idx {i} is infinity");
            if (float.IsNaN(state[i]))
                Debug.Log($"State idx {i} is NaN");
        }
        return state;

    }

    // Gets CoM in world position
    public static Vector3 get_cm(Transform[] bone_to_transform, Vector3[] global_bone_positions = null)
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
            Vector3 child_center = global_bone_positions == null ? get_child_collider_center(t.gameObject) : global_bone_positions[i];
            CoM += mass * child_center;
            total_mass += mass;

        }
        return CoM / total_mass;
    }
    float deltatime()
    {
        return use_deltatime ? Time.fixedDeltaTime : frametime;
    }
    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    Vector3 resolve_vel_in_kin_ref_frame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, vel);
    }
    Vector3 resolve_pos_in_kin_ref_frame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - kinChar.cm);
    }
    Vector3 resolve_pos_in_sim_ref_frame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - simChar.cm);
    }

    void copy_vector_into_arr(ref float[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    void copy_vector_into_arr(ref float[] state, ref int start_idx, Vector2 v)
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
            for (int j = 0; j < 6; j++)
            {
                pos_diffs_sum += (kinChar.boneSurfacePts[i][j] - simChar.boneSurfacePts[i][j]).magnitude;
                vel_diffs_sum += (kinChar.boneSurfaceVels[i][j] - simChar.boneSurfaceVels[i][j]).magnitude;
            }
        }
        pos_reward = Math.Exp((-10f / nbodies) *  pos_diffs_sum );
        vel_reward = Math.Exp((-1f / nbodies) *  vel_diffs_sum );
    }

    void calc_local_pose_reward(out double pose_reward)
    {
        double pose_reward_sum = 0;
        for (int i = 1; i < 23; i++)
        {
            Transform kin_bone = kinChar.boneToTransform[i];
            Transform sim_bone = simChar.boneToTransform[i];
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
        cm_vel_reward = Math.Exp(-1d * (kinChar.cmVel - simChar.cmVel).magnitude);
    }

    void calc_fall_factor(out double fall_factor, out bool heads_1m_apart)
    {
        Vector3 kin_head_pos = kinChar.boneToTransform[(int)Bone_Head].position;
        Vector3 sim_head_pos = simChar.boneToTransform[(int)Bone_Head].position;
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
    public static void getSixPointsOnCollider(GameObject obj, ref Vector3[] outputs, mm_v2.Bones bone)
    {
        if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
            get_six_points_on_box(obj, ref outputs);
        else
            get_six_points_on_capsule(obj, ref outputs);

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
        CustomInit();
        for (int i = 1; i < 23; i++)
        {
            // Get bone, transform, child capsule object
            mm_v2.Bones bone = (mm_v2.Bones)i;
            GameObject kin_collider_obj = kinChar.boneToCollider[i];
            Vector3[] gizmos = new Vector3[6];
            getSixPointsOnCollider(kin_collider_obj, ref gizmos, bone);            
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
