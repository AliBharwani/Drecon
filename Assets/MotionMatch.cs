using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class MotionMatch : MonoBehaviour
{
    public bool use_deltatime = false;
    private float frametime = 1f / 30f;
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

    public string databaseFilepath = "database";
    public int numNeigh = 1;
    public int searchEveryNFrames = 1;
    public int frameCounter = 1;

    public Transform[] boneToTransform = new Transform[23];
    [HideInInspector]
    public database motionDB;
    int frameIdx = 0;

    Vector3[] curr_bone_positions;
    Vector3[] curr_bone_velocities;
    Quaternion[] curr_bone_rotations;
    Vector3[] curr_bone_angular_velocities;

    Vector3[] bone_positions;
    Vector3[] bone_velocities;
    [HideInInspector]
    public Quaternion[] bone_rotations;
    Vector3[] bone_angular_velocities;

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
        gamepad = Gamepad.current;
        Application.targetFrameRate = 30;
        origin = transform.position;
        if (motionDB == null)
        {
            //motionDB = new database(Application.dataPath + @"/outputs/" + databaseFilepath + ".bin", numNeigh, frame_increments, ignore_surrounding);
            motionDB = database.Instance;
        }
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

        bone_positions = motionDB.bone_positions[frameIdx];
        bone_velocities = motionDB.bone_velocities[frameIdx];
        bone_rotations = motionDB.bone_rotations[frameIdx];
        bone_angular_velocities = motionDB.bone_angular_velocities[frameIdx];


        local_bone_positions = new Vector3[numBones];
        local_bone_rotations = Utils.identity_quat_arr(numBones);
        bone_parents = motionDB.bone_parents;

        random_lstick_input = Random.insideUnitCircle;
        prob_to_change_inputs = 1f / prob_to_change_inputs;

        is_initalized = true;
    }

}
