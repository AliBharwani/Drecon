using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.InputSystem;

public class MotionMatchingAnimator : MonoBehaviour
{
    public bool gen_inputs = true;
    public bool walk_only = false;
    private ConfigManager _config;

    public enum Bones
    {
        Bone_Entity = 0,
        Bone_Hips = 1,
        Bone_LeftUpLeg = 2,
        Bone_LeftLeg = 3,
        Bone_LeftFoot = 4,
        Bone_LeftToe = 5,
        Bone_RightUpLeg = 6,
        Bone_RightLeg = 7,
        Bone_RightFoot = 8,
        Bone_RightToe = 9,
        Bone_Spine = 10,
        Bone_Spine1 = 11,
        Bone_Spine2 = 12,
        Bone_Neck = 13,
        Bone_Head = 14,
        Bone_LeftShoulder = 15,
        Bone_LeftArm = 16,
        Bone_LeftForeArm = 17,
        Bone_LeftHand = 18,
        Bone_RightShoulder = 19,
        Bone_RightArm = 20,
        Bone_RightForeArm = 21,
        Bone_RightHand = 22,
    };
    public Transform[] toy_pointers = new Transform[3];
    public GameObject traj_marker;
    GameObject[] traj_markers;
    public float feature_weight_foot_position = 0.75f;
    public float feature_weight_foot_velocity = 1.0f;
    public float feature_weight_hip_velocity = 1.0f;
    public float feature_weight_trajectory_positions = 1.0f;
    public float feature_weight_trajectory_directions = 1.5f;
    public float inertialize_blending_halflife = .25f; //0.1f;
    public float simulation_velocity_halflife = .27f;
    public float simulation_rotation_halflife = .27f;
    public int frame_increments = 10;
    public int ignore_surrounding = 10;

    public float search_time = 5f / 60f;
    float search_timer;
    float force_search_timer;

    public int frameCounter = 1;

    public Transform[] boneToTransform = new Transform[23];
    [HideInInspector]
    public MocapDB motionDB;
    int frameIdx = 0;

    Vector3[] curr_bone_positions;
    internal Vector3[] curr_bone_velocities;
    Quaternion[] curr_bone_rotations;
    internal Vector3[] curr_bone_angular_velocities;
    bool[] curr_bone_contacts;

    Vector3[] trns_bone_positions;
    Vector3[] trns_bone_velocities;
    Quaternion[] trns_bone_rotations;
    Vector3[] trns_bone_angular_velocities;

    Vector3[] bone_positions;
    Vector3[] bone_velocities;
    internal Quaternion[] bone_rotations;
    internal Vector3[] bone_angular_velocities;

    Vector3[] bone_offset_positions;
    Vector3[] bone_offset_velocities;
    Quaternion[] bone_offset_rotations;
    Vector3[] bone_offset_angular_velocities;

    Vector3[] global_bone_positions;
    Quaternion[] global_bone_rotations;

    Vector3 transition_dst_position;
    Quaternion transition_dst_rotation;
    Vector3 transition_src_position;
    Quaternion transition_src_rotation;

    Vector3[] trajectory_desired_velocities;
    Quaternion[] trajectory_desired_rotations;
    Vector3[] trajectory_positions;
    Vector3[] trajectory_velocities;
    Vector3[] trajectory_accelerations;
    Quaternion[] trajectory_rotations;
    Vector3[] trajectory_angular_velocities;

    float desired_gait = 0.0f;
    float desired_gait_velocity = 0.0f;
    bool is_runbutton_pressed = false;

    float simulation_fwrd_speed = 0f;
    float simulation_side_speed = 0f;
    float simulation_back_speed = 0f;

    float simulation_run_fwrd_speed = 4.5f;
    float simulation_run_side_speed = 3.0f;
    float simulation_run_back_speed = 2.5f;

    float simulation_walk_fwrd_speed = 1.75f;
    float simulation_walk_side_speed = 1.5f;
    float simulation_walk_back_speed = 1.25f;

    Vector3 simulation_position;
    Vector3 simulation_velocity;
    Vector3 simulation_acceleration;
    Quaternion simulation_rotation;
    Vector3 simulation_angular_velocity;
    [HideInInspector]
    public Vector3 desired_velocity;
    Vector3 desired_velocity_change_curr;
    Vector3 desired_velocity_change_prev;
    float desired_velocity_change_threshold = 50.0f;

    public Quaternion desired_rotation = Quaternion.identity;
    Vector3 desired_rotation_change_curr = Vector3.zero;
    Vector3 desired_rotation_change_prev = Vector3.zero;
    float desired_rotation_change_threshold = 50.0f;

    int[] bone_parents;
    int best_idx;
    int numBones;
    Gamepad gamepad;

    // IK 
    public bool ik_enabled = true;
    float ik_max_length_buffer = 0.015f;
    float ik_foot_height = 0.02f;
    float ik_toe_length = 0.15f;
    float ik_unlock_radius = 0.2f;
    float ik_blending_halflife = 0.1f;
    static Bones[] contact_bones =  { Bones.Bone_LeftToe, Bones.Bone_RightToe};
    bool[] contact_states = new bool[contact_bones.Length];
    bool[] contact_locks = new bool[contact_bones.Length];
    Vector3[] contact_positions = new Vector3[contact_bones.Length];
    Vector3[] contact_velocities = new Vector3[contact_bones.Length];
    Vector3[] contact_points = new Vector3[contact_bones.Length];
    Vector3[] contact_targets = new Vector3[contact_bones.Length];
    Vector3[] contact_offset_positions = new Vector3[contact_bones.Length];
    Vector3[] contact_offset_velocities = new Vector3[contact_bones.Length];
    bool[] global_bone_computed = new bool[23];

    // ====================== Stuff added for ML
    Vector2 random_lstick_input;
    internal Vector3 origin;
    Quaternion origin_rot;
    bool is_strafing;
    [HideInInspector]
    public bool teleportedThisFixedUpdate = false;
    [HideInInspector]
    public bool is_initalized = false;
    public bool synchronization_enabled = false;
    private InputGenerator input_generator;
    internal PlayerCamTarget playerCamTarget;
    public bool drawGizmos;
    public bool show_traj_markers;
    void Awake()
    {
        Application.targetFrameRate = 60;
        Time.fixedDeltaTime = 1f / 60f;
        gamepad = Gamepad.current;
        origin = transform.position;
        origin_rot = transform.rotation;

        motionDB = MocapDB.Instance;
        _config = ConfigManager.Instance;

        simulation_velocity_halflife = _config.simulationVelocityHalflife;
        simulation_rotation_halflife = _config.simulation_rotation_halflife;

        walk_only = _config.walkOnly;
        motionDB.database_build_matching_features(
            feature_weight_foot_position,
            feature_weight_foot_velocity,
            feature_weight_hip_velocity,
            feature_weight_trajectory_positions,
            feature_weight_trajectory_directions,
            frame_increments,
            ignore_surrounding);
        numBones = motionDB.nbones();
        init(origin, Quaternion.identity);
        input_generator = gameObject.AddComponent<InputGenerator>();
        input_generator.inputChangeHalflife = _config.inputGeneratorHalflife;
        is_initalized = true;
        if (show_traj_markers)
        {
            traj_markers = new GameObject[4];
            for (int i = 0; i < 4; i++)
                traj_markers[i] = Instantiate(traj_marker);
        }
    }

    void init(Vector3 pos, Quaternion rot)
    {
        curr_bone_positions = motionDB.bone_positions[frameIdx];
        curr_bone_velocities = motionDB.bone_velocities[frameIdx];
        curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        curr_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];
        curr_bone_contacts = motionDB.contact_states[frameIdx];

        trns_bone_positions = motionDB.bone_positions[frameIdx];
        trns_bone_velocities = motionDB.bone_velocities[frameIdx];
        trns_bone_rotations = motionDB.bone_rotations[frameIdx];
        trns_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];

        bone_positions = new Vector3[numBones];
        bone_velocities = new Vector3[numBones];
        bone_rotations = new Quaternion[numBones];
        bone_angular_velocities = new Vector3[numBones];
        System.Array.Copy(motionDB.bone_positions[frameIdx], bone_positions, numBones);
        System.Array.Copy(motionDB.bone_velocities[frameIdx], bone_velocities, numBones);
        System.Array.Copy(motionDB.bone_rotations[frameIdx], bone_rotations, numBones);
        System.Array.Copy(motionDB.bone_angular_velocities[frameIdx], bone_angular_velocities, numBones);

        bone_offset_positions = new Vector3[numBones];
        bone_offset_velocities = new Vector3[numBones];
        bone_offset_rotations = identity_quat_arr(numBones);
        bone_offset_angular_velocities = new Vector3[numBones];

        global_bone_positions = new Vector3[numBones];
        global_bone_rotations = identity_quat_arr(numBones);

        bone_parents = motionDB.bone_parents;

        trajectory_desired_velocities = new Vector3[4];
        trajectory_desired_rotations = identity_quat_arr(4);
        trajectory_positions = new Vector3[4];
        trajectory_velocities = new Vector3[4];
        trajectory_accelerations = new Vector3[4];
        trajectory_rotations = identity_quat_arr(4);
        trajectory_angular_velocities = new Vector3[4];
        random_lstick_input =  Random.insideUnitCircle;

        simulation_position = pos;
        simulation_rotation = rot;
        inertialize_pose_reset(bone_positions[0], bone_rotations[0]);
        inertialize_pose_update(
            motionDB.bone_positions[frameIdx],
            motionDB.bone_velocities[frameIdx],
            motionDB.bone_rotations[frameIdx],
            motionDB.bone_angular_velocities[frameIdx],
            inertialize_blending_halflife,
            0f
        );
        reset_contact_state();

        search_timer = search_time;
        force_search_timer = search_time;
    }
    public void Reset()
    {
        frameIdx = 0;
        frameCounter = 1;
        simulation_velocity = Vector3.zero;
        simulation_acceleration = Vector3.zero;
        simulation_rotation = Quaternion.identity;
        simulation_angular_velocity = Vector3.zero;
        desired_velocity = Vector3.zero;
        desired_rotation = Quaternion.identity;
        init(origin, Quaternion.identity);
    }

    internal void reset_contact_state()
    {
        for (int i = 0; i < contact_bones.Length; i++)
        {
            Vector3 bone_position = Vector3.zero;
            Vector3 bone_velocity = Vector3.zero;
            Quaternion bone_rotation = Quaternion.identity;
            Vector3 bone_angular_velocity = Vector3.zero;
            motionDB.forward_kinematics_velocity(
                ref bone_position,
                ref bone_velocity,
                ref bone_rotation,
                ref bone_angular_velocity,
                bone_positions,
                bone_velocities,
                bone_rotations,
                bone_angular_velocities,
                (int)contact_bones[i]);

            contact_reset(
                ref contact_states[i],
                ref contact_locks[i],
                ref contact_positions[i],
                ref contact_velocities[i],
                ref contact_points[i],
                ref contact_targets[i],
                ref contact_offset_positions[i],
                ref contact_offset_velocities[i],
                bone_position,
                bone_velocity);
        }
    }
    internal void set_random_pose()
    {
        if (motionDB == null)
        {
            motionDB = new MocapDB();
            bone_parents = motionDB.bone_parents;
            numBones = motionDB.nbones();

            global_bone_positions = new Vector3[numBones];
            global_bone_rotations = identity_quat_arr(numBones);
        }
        // Set bone positions and bone rotations
        int random_frame = (int)Random.Range(0f, motionDB.numframes - 1);
        bone_positions = motionDB.bone_positions[random_frame];
        bone_rotations = motionDB.bone_rotations[random_frame];
        forward_kinematics_full();
        apply_global_pos_and_rot();
    }

    bool should_change_generated_inputs()
    {
        if (!gen_inputs)
            return false;
        return Random.value <= _config.prob_to_change_inputs;
    }

    internal void FixedUpdate()
    {
        teleportedThisFixedUpdate = false;
        if (should_change_generated_inputs())
        {
            if (_config.useCustomInputGenerator)
                input_generator.changeDirection();
            random_lstick_input = _config.useCustomInputGenerator ? input_generator.currentPosition : Random.insideUnitCircle;
            is_strafing = !_config.noStrafing && Random.value <= .5f;
            is_runbutton_pressed = !walk_only && Random.value <= .5f;
            Vector2 rotation_vec = is_strafing ? Random.insideUnitCircle : random_lstick_input;
            desired_rotation = MathUtils.quat_from_stick_dir(rotation_vec.x, rotation_vec.y);
        }
        else if (!gen_inputs)
        {
            is_strafing = gamepad != null && gamepad.leftTrigger.isPressed;
            is_runbutton_pressed = gamepad != null ? gamepad.aButton.isPressed : Keyboard.current.shiftKey.isPressed;
        }
        else if (gen_inputs && _config.useCustomInputGenerator)
            random_lstick_input = input_generator.currentPosition;

        desired_gait_update(Time.fixedDeltaTime);
        simulation_fwrd_speed = Mathf.Lerp(simulation_walk_fwrd_speed, simulation_run_fwrd_speed, desired_gait);
        simulation_side_speed = Mathf.Lerp(simulation_walk_side_speed, simulation_run_side_speed, desired_gait);
        simulation_back_speed = Mathf.Lerp( simulation_walk_back_speed, simulation_run_back_speed, desired_gait);

        Vector3 desired_velocity_curr = desired_velocity_update(simulation_rotation);
        Quaternion desired_rotation_curr = desired_rotation_update(desired_rotation, desired_velocity);
        Vector3 world_space_position = bone_positions[0];
        bool end_of_anim = motionDB.database_trajectory_index_clamp(frameIdx, 1) == frameIdx;

        // Check if we should force a search because input changed quickly
        desired_velocity_change_prev = desired_velocity_change_curr;
        desired_velocity_change_curr = (desired_velocity_curr - desired_velocity) / Time.fixedDeltaTime;
        desired_velocity = desired_velocity_curr;

        desired_rotation_change_prev = desired_rotation_change_curr;
        desired_rotation_change_curr = MathUtils.quat_to_scaled_angle_axis(MathUtils.quat_abs(MathUtils.quat_mul_inv(desired_rotation_curr, desired_rotation))) / Time.fixedDeltaTime;
        desired_rotation = desired_rotation_curr;

        bool force_search = false;

        if (force_search_timer <= 0.0f && (
            ((desired_velocity_change_prev).magnitude >= desired_velocity_change_threshold &&
             (desired_velocity_change_curr).magnitude < desired_velocity_change_threshold)
        || ((desired_rotation_change_prev).magnitude >= desired_rotation_change_threshold &&
             (desired_rotation_change_curr).magnitude < desired_rotation_change_threshold)))
        {
            force_search = true;
            force_search_timer = search_time;
        }
        else if (force_search_timer > 0)
        {
            force_search_timer -= Time.fixedDeltaTime;
        }
        //bool search = end_of_anim || (frameCounter % searchEveryNFrames) == 0;
        if (is_out_of_bounds(world_space_position))
        {
            bone_positions[0] = origin;
            simulation_position =  origin;
            force_search = true;
            reset_contact_state();
            teleportedThisFixedUpdate = true;
        }
        // Get the desired velocity

        trajectory_desired_rotations_predict();
        trajectory_rotations_predict(frame_increments * (Time.fixedDeltaTime));
        trajectory_desired_velocities_predict();
        trajectory_positions_predict(frame_increments * (Time.fixedDeltaTime));
        if (force_search || search_timer <= 0.0f || end_of_anim)
        {
            // Search database and update frame idx 
            motionMatch();
            search_timer = search_time;
        }
        else
            frameIdx++;

        playFrameIdx();
        search_timer -= Time.fixedDeltaTime;
        frameCounter++;
    }


    private void OnDrawGizmos()
    {
        if (!drawGizmos || !Application.isPlaying || trajectory_positions == null)
            return;
        Gizmos.color = Color.blue;
        for (int i = 0; i < 4; i++)
            Gizmos.DrawSphere(trajectory_positions[i], .1f);
    }

    private void playFrameIdx()
    {
        curr_bone_positions = motionDB.bone_positions[frameIdx];
        curr_bone_velocities = motionDB.bone_velocities[frameIdx];
        curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        curr_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];
        curr_bone_contacts = motionDB.contact_states[frameIdx];

        inertialize_pose_update(
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            inertialize_blending_halflife,
            Time.fixedDeltaTime
            );
        simulation_positions_update(
            ref simulation_position,
            ref simulation_velocity,
            ref simulation_acceleration,
            desired_velocity,
            simulation_velocity_halflife,
            Time.fixedDeltaTime);
        simulation_rotations_update(
            ref simulation_rotation,
            ref simulation_angular_velocity,
            desired_rotation,
            simulation_rotation_halflife,
            Time.fixedDeltaTime);

        if (synchronization_enabled)
        {
            simulation_position = bone_positions[0];
            simulation_rotation = bone_rotations[0];
        }

        Vector3[] adjusted_bone_positions = bone_positions;
        Quaternion[] adjusted_bone_rotations = bone_rotations;

        if (ik_enabled)
        {
            for (int i = 0; i < contact_bones.Length; i++)
            {
                // Find all the relevant bone indices
                int toe_bone = (int) contact_bones[i];
                int heel_bone = motionDB.bone_parents[toe_bone];
                int knee_bone = motionDB.bone_parents[heel_bone];
                int hip_bone = motionDB.bone_parents[knee_bone];
                int root_bone = motionDB.bone_parents[hip_bone];

                // Compute the world space position for the toe
                global_bone_computed = new bool[23];

                motionDB.forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    bone_positions,
                    bone_rotations,
                    toe_bone);

                // Update the contact state
                contact_update(
                    ref contact_states[i],
                    ref contact_locks[i],
                    ref contact_positions[i],
                    ref contact_velocities[i],
                    ref contact_points[i],
                    ref contact_targets[i],
                    ref contact_offset_positions[i],
                    ref contact_offset_velocities[i],
                    global_bone_positions[toe_bone],
                    curr_bone_contacts[i],
                    ik_unlock_radius,
                    ik_foot_height,
                    ik_blending_halflife,
                    Time.fixedDeltaTime);

                // Ensure contact position never goes through floor
                Vector3 contact_position_clamp = contact_positions[i];
                contact_position_clamp.y = Mathf.Max(contact_position_clamp.y, ik_foot_height);

                // Re-compute toe, heel, knee, hip, and root bone positions
                int[] feet_bones = { heel_bone, knee_bone, hip_bone, root_bone };
                foreach (int bone in feet_bones)
                {
                    motionDB.forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        bone_positions,
                        bone_rotations,
                        bone);
                }

                // Perform simple two-joint IK to place heel
                ik_two_bone(
                    ref adjusted_bone_rotations[hip_bone],
                    ref adjusted_bone_rotations[knee_bone],
                    global_bone_positions[hip_bone],
                    global_bone_positions[knee_bone],
                    global_bone_positions[heel_bone],
                    contact_position_clamp + (global_bone_positions[heel_bone] - global_bone_positions[toe_bone]),
                    MathUtils.quat_mul_vec3(global_bone_rotations[knee_bone], new Vector3(0.0f, 1.0f, 0.0f)),
                    global_bone_rotations[hip_bone],
                    global_bone_rotations[knee_bone],
                    global_bone_rotations[root_bone],
                    ik_max_length_buffer);

                // Re-compute toe, heel, and knee positions 
                global_bone_computed = new bool[23];
                int[] toe_heel_and_knee_bones = { toe_bone, heel_bone, knee_bone };
                foreach (int bone in toe_heel_and_knee_bones) {
                motionDB.forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    adjusted_bone_positions,
                    adjusted_bone_rotations,
                    bone);
                }

                // Rotate heel so toe is facing toward contact point
                ik_look_at(
                    ref adjusted_bone_rotations[heel_bone],
                    global_bone_rotations[knee_bone],
                    global_bone_rotations[heel_bone],
                    global_bone_positions[heel_bone],
                    global_bone_positions[toe_bone],
                    contact_position_clamp);

                // Re-compute toe and heel positions
                global_bone_computed = new bool[23];
                int[] toe_and_heel_bones = { toe_bone, heel_bone };
                foreach (int bone in toe_and_heel_bones)
                {
                    motionDB.forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        adjusted_bone_positions,
                        adjusted_bone_rotations,
                        bone);
                }

                // Rotate toe bone so that the end of the toe 
                // does not intersect with the ground
                Vector3 toe_end_curr = MathUtils.quat_mul_vec3(
                    global_bone_rotations[toe_bone], new Vector3(ik_toe_length, 0.0f, 0.0f)) +
                    global_bone_positions[toe_bone];

                Vector3 toe_end_targ = toe_end_curr;
                toe_end_targ.y = Mathf.Max(toe_end_targ.y, ik_foot_height);

                ik_look_at(
                    ref adjusted_bone_rotations[toe_bone],
                    global_bone_rotations[heel_bone],
                    global_bone_rotations[toe_bone],
                    global_bone_positions[toe_bone],
                    toe_end_curr,
                    toe_end_targ);
            }
            bone_positions = adjusted_bone_positions;
            bone_rotations = adjusted_bone_rotations;
        }

        forward_kinematics_full();
        apply_global_pos_and_rot();
    }

    // Moving the root is a little bit difficult when we have the
    // inertializer set up in the way we do. Essentially we need
    // to also make sure to adjust all of the locations where 
    // we are transforming the data to and from as well as the 
    // offsets being blended out
    private void inertialize_root_adjust(ref Vector3 offset_position, ref Vector3 position, ref Quaternion rotation, Vector3 input_position, Quaternion input_rotation)
    {
        // Find the position difference and add it to the state and transition location
        Vector3 position_difference = input_position - position;
        position = position_difference + position;
        transition_dst_position = position_difference + transition_dst_position;

        // Find the point at which we want to now transition from in the src data
        transition_src_position = transition_src_position + MathUtils.quat_mul_vec3(transition_src_rotation,
             MathUtils.quat_inv_mul_vec3(transition_dst_rotation, position - offset_position - transition_dst_position));
        transition_dst_position = position;
        offset_position = new Vector3();

        // Find the rotation difference. We need to normalize here or some error can accumulate 
        // over time during adjustment.
        Quaternion rotation_difference = Quaternion.Normalize(MathUtils.quat_mul_inv(input_rotation, rotation));

        // Apply the rotation difference to the current rotation and transition location
        rotation = rotation_difference * rotation;
        transition_dst_rotation = rotation_difference * transition_dst_rotation;
    }
    private void motionMatch()
    {
        float[] query = new float[motionDB.nfeatures()];
        float[] query_features = motionDB.features[frameIdx];
        int num_features_to_copy =
            3 // Left Foot position
            + 3 // Right Foot Position
            + 3 // Left Foot velocity
            + 3 // Right Foot velocity
            + 3; //Hip Velocity

        for (int i = 0; i < num_features_to_copy; i++)
            query[i] = query_features[i];
        query_compute_trajectory_position_feature(num_features_to_copy, query);
        query_compute_trajectory_direction_feature(num_features_to_copy + 6, query);
        best_idx = motionDB.motionMatch(query);

        trns_bone_positions = motionDB.bone_positions[best_idx];
        trns_bone_velocities = motionDB.bone_velocities[best_idx];
        trns_bone_rotations = motionDB.bone_rotations[best_idx];
        trns_bone_angular_velocities = motionDB.bone_angular_velocities[best_idx];

        inertialize_pose_transition(
            bone_positions[0],
            bone_velocities[0],
            bone_rotations[0],
            bone_angular_velocities[0],
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            trns_bone_positions,
            trns_bone_velocities,
            trns_bone_rotations,
            trns_bone_angular_velocities);

        frameIdx = best_idx;
    }

    private void query_compute_trajectory_position_feature(int offset, float[] query)
    {
        Vector3 root_position = bone_positions[0];
        Quaternion root_rotation = bone_rotations[0];

        Vector3 traj0 = MathUtils.quat_inv_mul_vec3(root_rotation, trajectory_positions[1] - root_position);
        Vector3 traj1 = MathUtils.quat_inv_mul_vec3(root_rotation,  trajectory_positions[2] - root_position);
        Vector3 traj2 = MathUtils.quat_inv_mul_vec3(root_rotation, trajectory_positions[3] - root_position);

        query[offset + 0] = traj0.x;
        query[offset + 1] = traj0.z;
        query[offset + 2] = traj1.x;
        query[offset + 3] = traj1.z;
        query[offset + 4] = traj2.x;
        query[offset + 5] = traj2.z;

        normalize(query, offset, 6);
    }

    private void query_compute_trajectory_direction_feature(int offset, float[] query)
    {
        Quaternion root_rotation = bone_rotations[0];

        Vector3 traj0 = MathUtils.quat_inv_mul_vec3(root_rotation, MathUtils.quat_mul_vec3(trajectory_rotations[1], Vector3.forward));
        Vector3 traj1 = MathUtils.quat_inv_mul_vec3(root_rotation, MathUtils.quat_mul_vec3(trajectory_rotations[2], Vector3.forward));
        Vector3 traj2 = MathUtils.quat_inv_mul_vec3(root_rotation, MathUtils.quat_mul_vec3(trajectory_rotations[3], Vector3.forward));

        query[offset + 0] = traj0.x;
        query[offset + 1] = traj0.z;
        query[offset + 2] = traj1.x;
        query[offset + 3] = traj1.z;
        query[offset + 4] = traj2.x;
        query[offset + 5] = traj2.z;

        normalize(query, offset, 6);
    }

    private void normalize(float[] query, int offset, int size)
    {
        for (int i = offset; i < offset + size; i++)
        {
            query[i] = (query[i] - motionDB.features_offset[i]) / motionDB.features_scale[i];
        }
    }

    private Vector2 get_user_input()
    {
        if (gamepad != null)
            return gamepad.leftStick.ReadValue();
        float x_val = (Keyboard.current.aKey.isPressed ? -1f : 0f) + (Keyboard.current.dKey.isPressed ? 1f : 0f);
        float y_val = (Keyboard.current.wKey.isPressed ? 1f : 0f) + (Keyboard.current.sKey.isPressed ? -1f : 0f);
        return new Vector2(x_val, y_val);
    }

    // Get desired velocity
    private Vector3 desired_velocity_update(Quaternion sim_rotation)
    {
        Vector2 lstick2 = gen_inputs ? random_lstick_input : get_user_input();
        Vector3 lstick = new Vector3(lstick2.x, 0f, lstick2.y);
        if (!gen_inputs && playerCamTarget != null) {
            lstick = Quaternion.Euler(0f, playerCamTarget.yRot, 0f) * lstick;
        }

        // Find stick position local to current facing direction
        Vector3 local_stick_dir = MathUtils.quat_inv_mul_vec3(sim_rotation, lstick);

        local_stick_dir.x *= simulation_side_speed;
        local_stick_dir.z *= local_stick_dir.z > 0.0 ? simulation_fwrd_speed : simulation_back_speed;
        return MathUtils.quat_mul_vec3(sim_rotation, local_stick_dir);
    }

    void desired_gait_update(
    float dt,
    float gait_change_halflife = 0.1f)
    {
        SpringUtils.simple_spring_damper_exact(
            ref desired_gait,
            ref desired_gait_velocity,
            is_runbutton_pressed ? 1.0f : 0.0f,
            gait_change_halflife,
            dt);
    }

    private void trajectory_positions_predict(float dt)
    {
        trajectory_positions[0] = simulation_position;
        trajectory_velocities[0] = simulation_velocity;
        trajectory_accelerations[0] = simulation_acceleration;
        if (show_traj_markers)
            traj_markers[0].transform.position = trajectory_positions[0];

        for (int i = 1; i < trajectory_positions.Length; i++)
        {
            trajectory_positions[i] = trajectory_positions[i - 1];
            trajectory_velocities[i] = trajectory_velocities[i - 1];
            trajectory_accelerations[i] = trajectory_accelerations[i - 1];

            simulation_positions_update(
                ref trajectory_positions[i],
                ref trajectory_velocities[i],
                ref trajectory_accelerations[i],
                trajectory_desired_velocities[i],
                simulation_velocity_halflife,
                dt);
            if (show_traj_markers)
                traj_markers[i].transform.position = trajectory_positions[i];
        }
    }
    private void trajectory_desired_velocities_predict()
    {
        trajectory_desired_velocities[0] = desired_velocity;
        for (int i = 1; i < trajectory_desired_velocities.Length; i++)
        {
            trajectory_desired_velocities[i] = desired_velocity_update(
                trajectory_rotations[i]);
        }
    }

    private void trajectory_rotations_predict(float dt)
    {
        for (int i = 0; i < trajectory_rotations.Length; i++)
        {
            trajectory_rotations[i] = simulation_rotation;
            trajectory_angular_velocities[i] = simulation_angular_velocity;
        }
        for (int i = 1; i < trajectory_rotations.Length; i++)
        {
            simulation_rotations_update(
                ref trajectory_rotations[i],
                ref trajectory_angular_velocities[i],
                trajectory_desired_rotations[i],
                simulation_rotation_halflife,
                i * dt
                );
        }


    }

    private void contact_reset(
        ref bool contact_state,
        ref bool contact_lock,
        ref Vector3 contact_position,
        ref Vector3 contact_velocity,
        ref Vector3 contact_point,
        ref Vector3 contact_target,
        ref Vector3 contact_offset_position,
        ref Vector3 contact_offset_velocity,
        Vector3 input_contact_position, 
        Vector3 input_contact_velocity)
    {
        contact_state = false;
        contact_lock = false;
        contact_position = input_contact_position;
        contact_velocity = input_contact_velocity;
        contact_point = input_contact_position;
        contact_target = input_contact_position;
        contact_offset_position = Vector3.zero;
        contact_offset_velocity = Vector3.zero;
    }

    private void contact_update(
        ref bool contact_state,
        ref bool contact_lock,
        ref Vector3 contact_position,
        ref Vector3 contact_velocity,
        ref Vector3 contact_point,
        ref Vector3 contact_target,
        ref Vector3 contact_offset_position,
        ref Vector3 contact_offset_velocity,
        Vector3 input_contact_position,
        bool input_contact_state,
        float unlock_radius,
        float foot_height,
        float halflife,
        float dt,
        float eps = 1e-8f)
    {
        // First compute the input contact position velocity via finite difference
        Vector3 input_contact_velocity =
        (input_contact_position - contact_target) / (dt + eps);
        contact_target = input_contact_position;

        // Update the inertializer to tick forward in time
        SpringUtils.inertialize_update(
        ref contact_position,
        ref contact_velocity,
        ref contact_offset_position,
        ref contact_offset_velocity,
        // If locked we feed the contact point and zero velocity, 
        // otherwise we feed the input from the animation
        contact_lock? contact_point : input_contact_position,
        contact_lock? Vector3.zero : input_contact_velocity,
        halflife,
        dt);

    // If the contact point is too far from the current input position 
    // then we need to unlock the contact
    bool unlock_contact = contact_lock && (
        (contact_point - input_contact_position).magnitude > unlock_radius);
    
    // If the contact was previously inactive but is now active we 
    // need to transition to the locked contact state
    if (!contact_state && input_contact_state)
    {
        // Contact point is given by the current position of 
        // the foot projected onto the ground plus foot height
        contact_lock = true;
        contact_point = contact_position;
        contact_point.y = foot_height;

            SpringUtils.inertialize_transition(
            ref contact_offset_position,
            ref contact_offset_velocity,
            input_contact_position,
            input_contact_velocity,
            contact_point,
            Vector3.zero);
    }

        // Otherwise if we need to unlock or we were previously in 
        // contact but are no longer we transition to just taking 
        // the input position as-is
    else if ((contact_lock && contact_state && !input_contact_state)
             || unlock_contact)
    {
        contact_lock = false;

         SpringUtils.inertialize_transition(
            ref contact_offset_position,
            ref contact_offset_velocity,
            contact_point,
            Vector3.zero,
            input_contact_position,
            input_contact_velocity);
    }

    // Update contact state
    contact_state = input_contact_state;
    }

    // Rotate a joint to look toward some 
    // given target position
    private void ik_look_at(
        ref Quaternion bone_rotation,
        Quaternion global_parent_rotation,
        Quaternion global_rotation,
        Vector3 global_position,
        Vector3 child_position,
        Vector3 target_position,
         float eps = 1e-5f)
    {
        Vector3 curr_dir = (child_position - global_position).normalized;
        Vector3 targ_dir = (target_position - global_position).normalized;

        if (1f - Vector3.Dot(curr_dir, targ_dir) > eps)
        {
            bone_rotation = MathUtils.quat_inv_mul(global_parent_rotation,
                MathUtils.quat_between(curr_dir, targ_dir) * global_rotation );
        }
    }
    // Basic two-joint IK in the style of https://theorangeduck.com/page/simple-two-joint
    // Here I add a basic "forward vector" which acts like a kind of pole-vetor
    // to control the bending direction
    void ik_two_bone(
        ref Quaternion bone_root_lr,
        ref Quaternion bone_mid_lr,
        Vector3 bone_root,
        Vector3 bone_mid,
        Vector3 bone_end,
        Vector3 target,
        Vector3 fwd,
        Quaternion bone_root_gr,
        Quaternion bone_mid_gr,
        Quaternion bone_par_gr,
        float max_length_buffer)
    {
        float max_extension =
            (bone_root - bone_mid).magnitude +
            (bone_mid - bone_end).magnitude -
            max_length_buffer;

        Vector3 target_clamp = target;
        if ((target - bone_root).magnitude > max_extension)
        {
            target_clamp = bone_root + max_extension*  (target - bone_root).normalized;
        }

        Vector3 axis_dwn = (bone_end - bone_root).normalized;
        Vector3 axis_rot = (Vector3.Cross(axis_dwn, fwd)).normalized;

        Vector3 a = bone_root;
        Vector3 b = bone_mid;
        Vector3 c = bone_end;
        Vector3 t = target_clamp;

        float lab =  (b - a).magnitude;
        float lcb =  (b - c).magnitude;
        float lat =  (t - a).magnitude;

        float ac_ab_0 = Mathf.Acos(Mathf.Clamp(Vector3.Dot((c - a).normalized, (b - a).normalized), -1.0f, 1.0f));
        float ba_bc_0 = Mathf.Acos(Mathf.Clamp(Vector3.Dot((a - b).normalized, (c - b).normalized), -1.0f, 1.0f));

        float ac_ab_1 = Mathf.Acos(Mathf.Clamp((lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat), -1.0f, 1.0f));
        float ba_bc_1 = Mathf.Acos(Mathf.Clamp((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));

        Quaternion r0 = MathUtils.quat_from_angle_axis(ac_ab_1 - ac_ab_0, axis_rot);
        Quaternion r1 = MathUtils.quat_from_angle_axis(ba_bc_1 - ba_bc_0, axis_rot);

        Vector3 c_a =  (bone_end - bone_root).normalized;
        Vector3 t_a =  (target_clamp - bone_root).normalized;

        Quaternion r2 = MathUtils.quat_from_angle_axis(
            Mathf.Acos(Mathf.Clamp(Vector3.Dot(c_a, t_a), -1.0f, 1.0f)),
           Vector3.Cross(c_a, t_a).normalized);

        bone_root_lr = MathUtils.quat_inv_mul(bone_par_gr,  r2 * (r0 * bone_root_gr));
        bone_mid_lr = MathUtils.quat_inv_mul(bone_root_gr,  r1 * bone_mid_gr);
    }
    private void simulation_rotations_update(ref Quaternion rotation, ref Vector3 angular_velocity, in Quaternion desired_rot, float halflife, float dt)
    {
        SpringUtils.simple_spring_damper_implicit(
                ref rotation,
                ref angular_velocity,
                desired_rot,
                halflife,
                dt
            );
    }

    private void simulation_positions_update(ref Vector3 position, ref Vector3 velocity, ref Vector3 acceleration, in Vector3 desired_velocity, float halflife, float dt)
    {
        float y = SpringUtils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = velocity - desired_velocity;
        Vector3 j1 = acceleration + j0 * y;
        float eydt = SpringUtils.fast_negexp(y * dt);

        Vector3 position_prev = position;

        position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
        velocity = eydt * (j0 + j1 * dt) + desired_velocity;
        acceleration = eydt * (acceleration - j1 * y * dt);
    }
    // Predict desired rotations 
    private void trajectory_desired_rotations_predict()
    {
        trajectory_desired_rotations[0] = desired_rotation;
        for (int i = 1; i < trajectory_desired_rotations.Length; i++)
        {
            trajectory_desired_rotations[i] = desired_rotation_update(
                trajectory_desired_rotations[i - 1],
                trajectory_desired_velocities[i]
                );
        }
    }
    private Quaternion desired_rotation_update(Quaternion rotation, Vector3 velocity)
    {
        if ( is_strafing)
            return desired_rotation;
        if (gen_inputs || user_is_inputting())
        {
            Vector3 desired_dir = velocity.normalized;
            return MathUtils.quat_from_stick_dir(desired_dir.x, desired_dir.z);
        }
        else
        {
            return rotation;
        }
    }

    private bool user_is_inputting()
    {
        if (gamepad != null)
            return gamepad.leftStick.ReadValue().magnitude > .01f;
        return Keyboard.current.wKey.isPressed || Keyboard.current.sKey.isPressed || Keyboard.current.aKey.isPressed || Keyboard.current.dKey.isPressed;
    }

    private bool is_out_of_bounds(Vector3 pos)
    {
        return Vector3.Distance(origin, pos) > _config.MAX_WANDERING_RADIUS;
    }

    private Vector3 get_world_space_position()
    {
        return MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_positions[0] - transition_src_position))
                + transition_dst_position; 
    }
    private void inertialize_pose_update(
        in Vector3[] bone_input_positions,
        in Vector3[] bone_input_velocities,
        in Quaternion[] bone_input_rotations,
        in Vector3[] bone_input_angular_velocities,
        float halflife,
        float dt)
    {
        // First we find the next root position, velocity, rotation
        // and rotational velocity in the world space by transforming 
        // the input animation from it's animation space into the 
        // space of the currently playing animation.

        Vector3 world_space_position = MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_input_positions[0] - transition_src_position))
                + transition_dst_position;


        Vector3 world_space_velocity = MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities[0]));

        // Normalize here because quat inv mul can sometimes produce 
        // unstable returns when the two rotations are very close.
        Quaternion world_space_rotation = transition_dst_rotation *
             MathUtils.quat_inv_mul(transition_src_rotation, bone_input_rotations[0]);
        world_space_rotation.Normalize();

        Vector3 world_space_angular_velocity = MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities[0]));

        SpringUtils.inertialize_update(
            ref bone_positions[0],
            ref bone_velocities[0],
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            world_space_position,
            world_space_velocity,
            halflife,
            dt);

        SpringUtils.inertialize_update(
            ref bone_rotations[0],
            ref bone_angular_velocities[0],
            ref bone_offset_rotations[0],
            ref bone_offset_angular_velocities[0],
            world_space_rotation,
            world_space_angular_velocity,
            halflife,
            dt);
        // Then we update the inertializers for the rest of the bones
        for (int i = 1; i < bone_positions.Length; i++)
        {
            SpringUtils.inertialize_update(
                ref bone_positions[i],
                ref bone_velocities[i],
                ref bone_offset_positions[i],
                ref bone_offset_velocities[i],
                bone_input_positions[i],
                bone_input_velocities[i],
                halflife,
                dt);

            SpringUtils.inertialize_update(
                ref bone_rotations[i],
                ref bone_angular_velocities[i],
                ref bone_offset_rotations[i],
                ref bone_offset_angular_velocities[i],
                bone_input_rotations[i],
                bone_input_angular_velocities[i],
                halflife,
                dt);
        }
    }
    private void inertialize_pose_reset(Vector3 root_position, Quaternion root_rotation)
    {
        bone_offset_positions = new Vector3[numBones];
        bone_offset_velocities = new Vector3[numBones];
        bone_offset_rotations = identity_quat_arr(numBones);
        bone_offset_angular_velocities = new Vector3[numBones];

        transition_src_position = root_position;
        transition_src_rotation = root_rotation;
        transition_dst_position = new Vector3() + origin;
        transition_dst_rotation = Quaternion.identity;
    }
    // This function transitions the inertializer for 
    // the full character. It takes as input the current 
    // offsets, as well as the root transition locations,
    // current root state, and the full pose information 
    // for the pose being transitioned from (src) as well 
    // as the pose being transitioned to (dst) in their
    // own animation spaces.
    private void inertialize_pose_transition(
            Vector3 root_position,
            Vector3 root_velocity,
            Quaternion root_rotation,
            Vector3 root_angular_velocity,
            in Vector3[] bone_src_positions,
            in Vector3[] bone_src_velocities,
            in Quaternion[] bone_src_rotations,
            in Vector3[] bone_src_angular_velocities,
            in Vector3[] bone_dst_positions,
            in Vector3[] bone_dst_velocities,
            in Quaternion[] bone_dst_rotations,
            in Vector3[] bone_dst_angular_velocities
        )
    {
        // First we record the root position and rotation
        // in the animation data for the source and destination
        // animation
        transition_dst_position = root_position;
        transition_dst_rotation = root_rotation;
        transition_src_position = bone_dst_positions[0];
        transition_src_rotation = bone_dst_rotations[0];


        // We then find the velocities so we can transition the 
        // root inertiaizers
        Vector3 world_space_dst_velocity = MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_velocities[0]));

        Vector3 world_space_dst_angular_velocity = MathUtils.quat_mul_vec3(transition_dst_rotation,
            MathUtils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_angular_velocities[0]));


        SpringUtils.inertialize_transition(
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            root_position,
            root_velocity,
            root_position,
            world_space_dst_velocity);

        SpringUtils.inertialize_transition(
            ref bone_offset_rotations[0],
            ref bone_offset_angular_velocities[0],
            root_rotation,
            root_angular_velocity,
            root_rotation,
            world_space_dst_angular_velocity);

        // Then we update the inertializers for the rest of the bones
        for (int i = 1; i < bone_offset_positions.Length; i++)
        {

            SpringUtils.inertialize_transition(
                ref bone_offset_positions[i],
                ref bone_offset_velocities[i],
                bone_src_positions[i],
                bone_src_velocities[i],
                bone_dst_positions[i],
                bone_dst_velocities[i]);

            SpringUtils.inertialize_transition(
                    ref bone_offset_rotations[i],
                    ref bone_offset_angular_velocities[i],
                    bone_src_rotations[i],
                    bone_src_angular_velocities[i],
                    bone_dst_rotations[i],
                    bone_dst_angular_velocities[i]);
        }
    }


    // Compute forward kinematics for all joints
    private void forward_kinematics_full()
    {

        for (int i = 0; i < bone_parents.Length; i++)
        {
            // Assumes bones are always sorted from root onwards
            Assert.IsTrue(bone_parents[i] < i);

            if (bone_parents[i] == -1)
            {
                global_bone_positions[i] = bone_positions[i];
                global_bone_rotations[i] = bone_rotations[i];
            }
            else
            {
                Vector3 parent_position = global_bone_positions[bone_parents[i]];
                Quaternion parent_rotation = global_bone_rotations[bone_parents[i]];
                global_bone_positions[i] = MathUtils.quat_mul_vec3(parent_rotation, bone_positions[i]) + parent_position;
                global_bone_rotations[i] = parent_rotation * bone_rotations[i];
            }
        }
    }

    private void apply_global_pos_and_rot()
    {
        for (int i = 0; i < numBones; i++)
        {
            Transform t = boneToTransform[i];
            t.position = global_bone_positions[i];
            t.rotation = global_bone_rotations[i];
        }
    }
    // --- Inference Time options

    public  void clamp_pos_and_rot(Vector3 target_pos, Quaternion target_rot)
    {
        Vector3 adjusted_position = bone_positions[0];
        Quaternion adjusted_rotation = bone_rotations[0];

        adjusted_position = clamp_character_position(
            adjusted_position,
            target_pos,
           _config.clampingMaxDistance);

        inertialize_root_adjust(
            ref bone_offset_positions[0],
            ref bone_positions[0],
            ref bone_rotations[0],
            adjusted_position,
            adjusted_rotation);
    }
    Vector3 clamp_character_position(
     Vector3 character_position,
     Vector3 simulation_position,
     float max_distance)
    {
        // If the character deviates too far from the simulation 
        // position we need to clamp it to within the max distance
        if ((character_position - simulation_position).magnitude > max_distance)
        {
            return max_distance*
                (character_position - simulation_position).normalized + 
                simulation_position;
        }
        else
        {
            return character_position;
        }
    }
    Quaternion clamp_character_rotation(
        Quaternion character_rotation,
        Quaternion simulation_rotation,
        float max_angle)
    {
        // If the angle between the character rotation and simulation 
        // rotation exceeds the threshold we need to clamp it back
        if (Quaternion.Angle(character_rotation, simulation_rotation) > max_angle)
        {
            // First, find the rotational difference between the two
            Quaternion diff = MathUtils.quat_abs(MathUtils.quat_mul_inv(
                character_rotation, simulation_rotation));

            // We can then decompose it into angle and axis
            float diff_angle; Vector3 diff_axis;
            diff.ToAngleAxis(out diff_angle, out diff_axis);

            // We then clamp the angle to within our bounds
            diff_angle = Mathf.Clamp(diff_angle, -max_angle, max_angle);
        
            // And apply back the clamped rotation
            return MathUtils.quat_from_angle_axis(diff_angle, diff_axis) * simulation_rotation;
        }
           else
        {
            return character_rotation;
        }
    }
    public static Quaternion[] identity_quat_arr(int size)
    {
        Quaternion[] ret = new Quaternion[size];
        for (int i = 0; i < size; i++)
            ret[i] = Quaternion.identity;
        return ret;
    }

}
