using System.Text;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.InputSystem;


public class mm_v2 : MonoBehaviour
{
    public bool gen_inputs = true;
    public float MAX_WANDERING_RADIUS = 10f;
    public float prob_to_change_inputs = 60f;
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
    public float feature_weight_foot_position = 0.75f;
    public float feature_weight_foot_velocity = 1.0f;
    public float feature_weight_hip_velocity = 1.0f;
    public float feature_weight_trajectory_positions = 1.0f;
    public float feature_weight_trajectory_directions = 1.5f;
    public float inertialize_blending_halflife = 0.1f;
    public float simulation_rotation_halflife = .1f;
    public bool abTest = true;
    public float MoveSpeed = 3;
    public int frame_increments = 10;
    public int ignore_surrounding = 10;

    public string databaseFilepath = "database_v1";
    public int numNeigh = 1;
    public int searchEveryNFrames = 1;
    public int frameCounter = 1;

    public Transform[] boneToTransform = new Transform[23];
    database motionDB;
    int frameIdx = 0;

    Vector3[] curr_bone_positions;
    Vector3[] curr_bone_velocities;
    Quaternion[] curr_bone_rotations;
    Vector3[] curr_bone_angular_velocities;

    Vector3[] trns_bone_positions;
    Vector3[] trns_bone_velocities;
    Quaternion[] trns_bone_rotations;
    Vector3[] trns_bone_angular_velocities;

    Vector3[] bone_positions;
    Vector3[] bone_velocities;
    [HideInInspector]
    public Quaternion[] bone_rotations;
    Vector3[] bone_angular_velocities;

    Vector3[] bone_offset_positions;
    Vector3[] bone_offset_velocities;
    Quaternion[] bone_offset_rotations;
    Vector3[] bone_offset_angular_velocities;

    Vector3[] global_bone_positions;
    Vector3[] global_bone_velocities;
    Quaternion[] global_bone_rotations;
    Vector3[] global_bone_angular_velocities;

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

    Vector3 simulation_position;
    Vector3 simulation_velocity;
    Vector3 simulation_acceleration;
    Quaternion simulation_rotation;
    Vector3 simulation_angular_velocity;
    [HideInInspector]
    public Vector3 desired_velocity;
    Quaternion desired_rotation = Quaternion.identity;

    Vector3[] local_bone_positions;
    Quaternion[] local_bone_rotations;
    int[] bone_parents;
    int best_idx;
    int numBones;
    Gamepad gamepad;

    // ====================== Stuff added for ML
    Vector2 random_lstick_input;
    Vector3 origin;
    bool is_strafing;
    [HideInInspector]
    public bool teleported_last_frame = false;
    [HideInInspector]
    public bool is_initalized = false;

    void Awake()
    {
        if (Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        gamepad = Gamepad.current;
        if (gamepad == null)
        {
            Debug.LogWarning("Warning: Gamepad not found");
        }
        Application.targetFrameRate = 30;
        origin = transform.position;
        motionDB = new database(Application.dataPath + @"/outputs/" + databaseFilepath + ".bin", numNeigh, abTest, frame_increments, ignore_surrounding);
        motionDB.database_build_matching_features(
            feature_weight_foot_position,
            feature_weight_foot_velocity,
            feature_weight_hip_velocity,
            feature_weight_trajectory_positions,
            feature_weight_trajectory_directions);
        numBones = motionDB.nbones();

        curr_bone_positions = motionDB.bone_positions[frameIdx];
        curr_bone_velocities = motionDB.bone_velocities[frameIdx];
        curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        curr_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];

        trns_bone_positions = motionDB.bone_positions[frameIdx];
        trns_bone_velocities = motionDB.bone_velocities[frameIdx];
        trns_bone_rotations = motionDB.bone_rotations[frameIdx];
        trns_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];

        bone_positions = motionDB.bone_positions[frameIdx];
        bone_velocities = motionDB.bone_velocities[frameIdx];
        bone_rotations = motionDB.bone_rotations[frameIdx];
        bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];

        bone_offset_positions = new Vector3[numBones];
        bone_offset_velocities = new Vector3[numBones];
        bone_offset_rotations = identityQuatArray(numBones);
        bone_offset_angular_velocities = new Vector3[numBones];

        global_bone_positions = new Vector3[numBones];
        global_bone_velocities = new Vector3[numBones];
        global_bone_rotations = identityQuatArray(numBones);
        global_bone_angular_velocities = new Vector3[numBones];

        local_bone_positions = new Vector3[numBones];
        local_bone_rotations = identityQuatArray(numBones);
        bone_parents = motionDB.bone_parents;

        trajectory_desired_velocities = new Vector3[4];
        trajectory_desired_rotations = identityQuatArray(4);
        trajectory_positions = new Vector3[4];
        trajectory_velocities = new Vector3[4];
        trajectory_accelerations = new Vector3[4];
        trajectory_rotations = identityQuatArray(4);
        trajectory_angular_velocities = new Vector3[4];
        random_lstick_input = Random.insideUnitCircle;
        prob_to_change_inputs = 1f / prob_to_change_inputs;
        simulation_position = origin;
        inertialize_pose_reset(bone_positions[0], bone_rotations[0]);
        inertialize_pose_update(
            motionDB.bone_positions[frameIdx],
            motionDB.bone_velocities[frameIdx],
            motionDB.bone_rotations[frameIdx],
            motionDB.bone_angular_velocities[frameIdx],
            inertialize_blending_halflife,
            0f
        );

    }

    void FixedUpdate()
    {
        teleported_last_frame = false;
        // Update if we are reading from user input (ie not generating random rotations) or we are
        // generating random inputs and every frame the user changes desires with P(.001)
        if (gen_inputs && Random.value <= prob_to_change_inputs) {
            Debug.Log("Genning new inputs!");
            random_lstick_input = Random.insideUnitCircle;
            // Random chance of making desired rotation face direction of velocity  
            is_strafing = Random.value <= .5f;
            Vector2 rotation_vec = is_strafing ?  Random.insideUnitCircle : random_lstick_input;
            desired_rotation =  Utils.quat_from_stick_dir(rotation_vec.x, rotation_vec.y) ;
        }
        desired_velocity = desired_velocity_update(simulation_rotation);
        //Debug.Log($"Gamepad: {gamepad.leftStick.ReadValue()}");
        desired_rotation = !gen_inputs ? desired_rotation_update(desired_rotation, desired_velocity) : desired_rotation;


        Vector3 world_space_position = bone_positions[0];
        bool end_of_anim = motionDB.database_trajectory_index_clamp(frameIdx, 1) == frameIdx;
        bool search = end_of_anim || (frameCounter % searchEveryNFrames) == 0;
        if (is_out_of_bounds(world_space_position))
        {
            bone_positions[0] = origin;
            simulation_position =  origin;
            search = true;
            teleported_last_frame = true;
        }
        // Get the desired velocity

        trajectory_desired_rotations_predict();
        trajectory_rotations_predict(frame_increments * Time.fixedDeltaTime);
        trajectory_desired_velocities_predict();
        trajectory_positions_predict(frame_increments * Time.fixedDeltaTime);
        if (search)
        {
            // Search database and update frame idx 
            motionMatch();
        }
        //motionDB.setDataToFrame(ref local_bone_positions, ref local_bone_rotations, frameIdx);
        frameIdx++;
        playFrameIdx();
        frameCounter++;
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || trajectory_positions == null)
            return;
        Gizmos.color = Color.blue;
        for (int i = 0; i < 4; i++)
        {
            Gizmos.DrawSphere(trajectory_positions[i], .1f);
            //if (i > 0)
            //{
            //    toy_pointers[i - 1].position = trajectory_positions[i];
            //    toy_pointers[i - 1].rotation = trajectory_rotations[i];
            //}
        }
        
        //toy_pointers[1].rotation = simulation_rotation;
        //toy_pointers[2].rotation = desired_rotation;

        //Gizmos.color = Color.green;
        //Vector3 root_position = bone_positions[0];
        //Quaternion root_rotation = bone_rotations[0];

        //Vector3 traj0 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[1] - root_position);
        //Vector3 traj1 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[2] - root_position);
        //Vector3 traj2 = Utils.quat_inv_mul_vec3(root_rotation,  trajectory_positions[3] - root_position);

        //Vector3 traj0 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[1] - root_position);
        //traj0 = Utils.quat_mul_vec3(root_rotation, traj0) + root_position;
        ////Vector3 traj1 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[2] - root_position);
        //traj1 = Utils.quat_mul_vec3(root_rotation, traj1) + root_position;
        ////Vector3 traj2 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[3] - root_position);
        //traj2 = Utils.quat_mul_vec3(root_rotation, traj2) + root_position;

        //Gizmos.DrawSphere(traj0, .1f);
        //Gizmos.DrawSphere(traj1, .1f);
        //Gizmos.DrawSphere(traj2, .1f);


    }

    private void playFrameIdx()
    {
        //Debug.Log($"Playing frame {frameIdx}");
        curr_bone_positions = motionDB.bone_positions[frameIdx];
        curr_bone_velocities = motionDB.bone_velocities[frameIdx];
        curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        curr_bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];
        inertialize_pose_update(
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            inertialize_blending_halflife,
            //1f/30f
            Time.deltaTime
            );
        simulation_positions_update(
            ref simulation_position,
            ref simulation_velocity,
            ref simulation_acceleration,
            desired_velocity,
            simulation_velocity_halflife,
            Time.deltaTime);
        simulation_rotations_update(
            ref simulation_rotation,
            ref simulation_angular_velocity,
            desired_rotation,
            simulation_rotation_halflife,
            Time.deltaTime);
        //inertialize_root_adjust(ref bone_offset_positions[0], ref bone_positions[0], ref bone_rotations[0], simulation_position, simulation_rotation);
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
        transition_src_position = transition_src_position + Utils.quat_mul_vec3(transition_src_rotation,
             Utils.quat_inv_mul_vec3(transition_dst_rotation, position - offset_position - transition_dst_position));
        transition_dst_position = position;
        offset_position = new Vector3();

        // Find the rotation difference. We need to normalize here or some error can accumulate 
        // over time during adjustment.
        Quaternion rotation_difference = Quaternion.Normalize(Utils.quat_mul_inv(input_rotation, rotation));

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
        //int debug_ignore = num_features_to_copy;
        for (int i = 0; i < num_features_to_copy; i++)
            query[i] = query_features[i];
        query_compute_trajectory_position_feature(num_features_to_copy, query);
        query_compute_trajectory_direction_feature(num_features_to_copy + 6, query);
        best_idx = motionDB.motionMatch(query);
        //Debug.Log($"Best idx: {best_idx}");
        //StringBuilder sb = new StringBuilder("Query: ");
        //for (int i = num_features_to_copy; i < query.Length; i++)
        //{
        //    sb.Append(query[i].ToString() + " , " );
        //}
        //Debug.Log(sb.ToString());

        trns_bone_positions = motionDB.bone_positions[best_idx];
        trns_bone_velocities = motionDB.bone_velocities[best_idx];
        trns_bone_rotations = motionDB.bone_rotations[best_idx];
        trns_bone_angular_velocities = motionDB.bone_angular_velocities[best_idx];

        inertialize_pose_transition(
            //bone_offset_positions,
            //bone_offset_velocities,
            //bone_offset_rotations,
            //bone_offset_angular_velocities,
            //transition_src_position,
            //transition_src_rotation,
            //transition_dst_position,
            //transition_dst_rotation,
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

        Vector3 traj0 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[1] - root_position);
        Vector3 traj1 = Utils.quat_inv_mul_vec3(root_rotation,  trajectory_positions[2] - root_position);
        Vector3 traj2 = Utils.quat_inv_mul_vec3(root_rotation, trajectory_positions[3] - root_position);

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

        Vector3 traj0 = Utils.quat_inv_mul_vec3(root_rotation, Utils.quat_mul_vec3(trajectory_rotations[1], Vector3.forward));
        Vector3 traj1 = Utils.quat_inv_mul_vec3(root_rotation, Utils.quat_mul_vec3(trajectory_rotations[2], Vector3.forward));
        Vector3 traj2 = Utils.quat_inv_mul_vec3(root_rotation, Utils.quat_mul_vec3(trajectory_rotations[3], Vector3.forward));

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
    float fwrd_speed = 3f;// 1.75f;
    float side_speed = 2.5f;//1.5f;
    float back_speed = 2f;//1.25f;
    // Get desired velocity
    private Vector3 desired_velocity_update(Quaternion sim_rotation)
    {
        Vector2 lstick = gen_inputs ? random_lstick_input : gamepad.leftStick.ReadValue();

        // Find stick position local to current facing direction
        Vector3 local_stick_dir = Utils.quat_inv_mul_vec3(sim_rotation, new Vector3(lstick.x, 0, lstick.y));
        local_stick_dir.x *= side_speed;
        local_stick_dir.z *= local_stick_dir.z > 0.0 ? fwrd_speed : back_speed;
        //Vector3 local_desired_vel = local_stick_dir.z > 0.0 ?
        //        new Vector3(side_speed, 0.0f, fwrd_speed):
        //        new Vector3(side_speed, 0.0f, back_speed) ;
        //Vector3 local_desired_vel = local_stick_dir.Scale(scale_vec);
        //Vector3 local_desired_vel = (MoveSpeed * local_stick_dir.magnitude) * local_stick_dir.normalized;
        return Utils.quat_mul_vec3(sim_rotation, local_stick_dir);
    }

    public float simulation_velocity_halflife = .27f;
    private void trajectory_positions_predict(float dt)
    {
        trajectory_positions[0] = simulation_position;
        trajectory_velocities[0] = simulation_velocity;
        trajectory_accelerations[0] = simulation_acceleration;

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
        if (gen_inputs && is_strafing)
            return desired_rotation;
        if (gen_inputs || userIsInputting())
        {
            Vector3 desired_dir = velocity.normalized;
            //Debug.Log(Mathf.Atan2(desired_dir.x, desired_dir.z));
            return Utils.quat_from_stick_dir(desired_dir.x, desired_dir.z);// Quaternion.AngleAxis(Mathf.Atan2(desired_dir.x, desired_dir.z) * Mathf.Rad2Deg, Vector3.up);
        }
        else
        {
            return rotation;
        }
    }

    private bool userIsInputting()
    {
        return gamepad.leftStick.ReadValue().magnitude > .01f;
    }

    private bool is_out_of_bounds(Vector3 pos)
    {
        return Vector3.Distance(origin, pos) > MAX_WANDERING_RADIUS;
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
        Vector3 world_space_position = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_positions[0] - transition_src_position))
                + transition_dst_position;

        //Vector3 debugTerm = Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_positions[0] - transition_src_position);
        //Debug.Log($"bone_input_positions[0] - transition_src_position: {bone_input_positions[0] - transition_src_position} \n" +
        //    $"quat_inv_mul_vec3(transition_src_rotation, bone_input_positions[0] - transition_src_position): {debugTerm} \n" +
        //    $"world_space_position: {world_space_position}");

        Vector3 world_space_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities[0]));
        //Debug.Log($"world_space_velocity: {world_space_velocity}");

        // Normalize here because quat inv mul can sometimes produce 
        // unstable returns when the two rotations are very close.
        Quaternion world_space_rotation = transition_dst_rotation *
             Utils.quat_inv_mul(transition_src_rotation, bone_input_rotations[0]);
        world_space_rotation.Normalize();
        //Debug.Log($"world_space_rotation: {world_space_rotation}");

        Vector3 world_space_angular_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities[0]));
        //Debug.Log($"world_space_angular_velocity: {world_space_angular_velocity}");

        SpringUtils.inertialize_update(
            ref bone_positions[0],
            ref bone_velocities[0],
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            world_space_position,
            world_space_velocity,
            halflife,
            dt);
        //Debug.Log($"Bone position after update: { bone_positions[0]}");

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
        bone_offset_rotations = identityQuatArray(numBones);
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
        Vector3 world_space_dst_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_velocities[0]));

        Vector3 world_space_dst_angular_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_angular_velocities[0]));


        SpringUtils.inertialize_transition(
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            root_position,
            root_velocity,
            root_position,
            world_space_dst_velocity);
        //Debug.Log($"Transitioning - transition_dst_position: {transition_dst_position}\n" +
        //            $"transition_src_position: {transition_src_position}\n" +
        //            $"bone_offset_positions: {bone_offset_positions[0]}");
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
    private void forward_kinematics_full(
        //in Vector3[] local_bone_positions,
        //in Quaternion[] local_bone_rotations,
        //in int[] bone_parents
        )
    {

        //Vector3[] bone_pos = useBlending ? bone_positions : local_bone_positions;
        //Quaternion[] bone_rot = useBlending ? bone_rotations : local_bone_rotations;
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
                global_bone_positions[i] = Utils.quat_mul_vec3(parent_rotation, bone_positions[i]) + parent_position;
                global_bone_rotations[i] = parent_rotation * bone_rotations[i];
            }
        }
    }

    private void apply_global_pos_and_rot()
    {
        //Debug.Log(numBones); // 23
        for (int i = 0; i < numBones; i++)
        {
            Transform t = boneToTransform[i];
            //if (i == 1)
            //{
                //Quaternion rot = local_bone_rotations[i];
                //Debug.Log($"Frame {frameIdx}: {rot.w}, {rot.x} , {rot.y} , {rot.z}");
                //t.localPosition = local_bone_positions[i];
                //t.position = global_bone_positions[i];
            //}
            //t.localRotation = local_bone_rotations[i];
            //t.localRotation = Utils.to_unity_rot(local_bone_rotations[i]);
            t.position = global_bone_positions[i];
            t.rotation = global_bone_rotations[i];
        }
        //boneToTransform[0].rotation *= Quaternion.AngleAxis( 180f, Vector3.forward);
    }

    private Quaternion[] identityQuatArray(int size)
    {
        Quaternion[] ret = new Quaternion[size];
        for (int i = 0; i < size; i++)
            ret[i] = Quaternion.identity;
        return ret;
    }
}
