using System.Linq;
using UnityEngine;
using static mm_v2.Bones;

public class TestDirector : MonoBehaviour
{
    private bool is_initalized;
    char_info kin_char, sim_char;
    private GameObject kinematic_char;
    private GameObject simulated_char;
    public GameObject simulated_char_prefab;
    public GameObject kinematic_char_prefab;

    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies;

    //private ArticulationBody sim_hip_bone; // root of ArticulationBody


    [HideInInspector]
    public static mm_v2.Bones[] state_bones = new mm_v2.Bones[]
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] all_full_dof_bones = new mm_v2.Bones[]
    //{ Bone_LeftArm, Bone_RightArm, };
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine,  Bone_Spine1, Bone_Spine2, Bone_Neck,  Bone_LeftHand, Bone_RightHand};
    [HideInInspector]
    public static mm_v2.Bones[] all_limited_dof_bones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftToe, Bone_RightToe, Bone_LeftForeArm, Bone_RightForeArm};

    [HideInInspector]
    public static mm_v2.Bones[] openloop_bones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand};

  // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public void apply_action()
    {
        //if (!is_initalized)
        //{
        //    my_initalize();
        //    return;
        //}
        Quaternion[] cur_rotations = MMScript.bone_rotations;
        for (int i = 0; i < all_limited_dof_bones.Length; i++)
        {
            int bone_idx = (int)all_limited_dof_bones[i];
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[bone_idx]);
            bool use_xdrive = all_limited_dof_bones[i] == Bone_LeftLeg || all_limited_dof_bones[i] == Bone_RightLeg;
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
        for (int i = 0; i < all_full_dof_bones.Length; i++)
        {
            int bone_idx = (int)all_full_dof_bones[i];
            // Debug.Log($"Cur bone rotations length - {cur_rotations.Length} - bone_idx : {bone_idx}");
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            //Debug.Log($"Setting drive rotation for ab: {ab.gameObject.name}");
            ab.SetDriveRotation(final);
        }

        for (int i = 0; i < openloop_bones.Length; i++)
        {
            int bone_idx = (int)openloop_bones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = sim_char.bone_to_art_body[bone_idx];
            //Debug.Log($"Setting drive rotation for ab: {ab.gameObject.name}");
            ab.SetDriveRotation(final);
        }
    }
    
    void my_initalize()
    {
        MMScript = kinematic_char.GetComponent<mm_v2>();
        if (!MMScript.is_initalized && !Application.isEditor)
            return;
        kin_char = new char_info();
        kin_char.char_trans = kinematic_char.transform;
        kin_char.bone_to_transform = MMScript.boneToTransform;
        kin_char.char_obj = kinematic_char;
        nbodies = kin_char.bone_to_transform.Length;
        kin_char.bone_surface_pts = new Vector3[nbodies][];

        SimCharController = simulated_char.GetComponent<SimCharController>();
        if (Application.isEditor)
            SimCharController.initBoneToArtBodies();
        sim_char = new char_info();
        sim_char.char_trans = simulated_char.transform;
        sim_char.bone_to_transform = SimCharController.boneToTransform;
        sim_char.hip_bone = SimCharController.bone_to_art_body[(int)Bone_Hips];
        sim_char.char_obj = simulated_char;
        sim_char.bone_surface_pts = new Vector3[nbodies][];
        sim_char.bone_to_art_body = SimCharController.bone_to_art_body;

        kin_char.bone_world_pos = new Vector3[state_bones.Length];
        sim_char.bone_world_pos = new Vector3[state_bones.Length];

        origin = kin_char.char_trans.position;
        origin_hip_rot = sim_char.bone_to_transform[(int)Bone_Hips].rotation;
        is_initalized = true;
    }

    private void Awake()
    {
        // Setup the kinematic character
        kinematic_char = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
        // Setup the sim character
        simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        //Application.targetFrameRate = 60;
    }

    void Start()
    {
        my_initalize();
        MMScript.debug_move_every_second = true;
        SimCharController.remove_joint_limits(sim_char.bone_to_art_body);
    }

    Vector3 origin;
    Quaternion origin_hip_rot;
    [ContextMenu("create_and_set_kin_char")]
    private void create_and_set_kin_char()
    {
        if (kinematic_char == null)
            kinematic_char = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
        MMScript = kinematic_char.GetComponent<mm_v2>();
        MMScript.set_random_pose();

    }
    public float physicsSimulateTime = .01f;
    [ContextMenu("Set sim char to match kin char")]
    private void create_and_set_sim_char()
    {
        //Physics.autoSimulation = false;
        if (simulated_char == null)
            simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        my_initalize();
        SimCharController.teleport_sim_char(sim_char, kin_char);
        Physics.Simulate(physicsSimulateTime);
    }
    [ContextMenu("reset")]
    private void delete_sim_char()
    {
        DestroyImmediate(simulated_char);
        DestroyImmediate(kinematic_char);
        kinematic_char = null;
        simulated_char = null;
    }

    int frame = 0;
    private void FixedUpdate()
    {
        if (frame % 5 == 0) {
            SimCharController.teleport_sim_char(sim_char, kin_char);//teleport_sim_char();
            //create_and_set_kin_char();
            //Destroy(MMScript);
        }
        return;
        apply_action();
        // Make sure to teleport sim character if kin character teleported
        bool teleport_sim = MMScript.teleported_last_frame;
        if (teleport_sim)
        {
            sim_char.char_trans.rotation = kin_char.char_trans.rotation;
            sim_char.hip_bone.TeleportRoot(origin, origin_hip_rot);
        }
        return;
    }
    private void Update()
    {
        frame++;
    }

}
