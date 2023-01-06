using System.Collections;
using UnityEngine;
using static mm_v2.Bones;

public class TestDirector : MonoBehaviour
{
    CharInfo kinChar, simChar;
    private GameObject kinematicChar;
    private GameObject simulatedChar;
    public GameObject simulatedCharPrefab;
    public GameObject kinematicCharPrefab;

    public float physicsSimulateTime = 1f / 60f;
    public float timeToSimulate = 10f;
    public float physicsSpeed = .5f;
    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies;

    //private ArticulationBody sim_hip_bone; // root of ArticulationBody


    [HideInInspector]
    public static mm_v2.Bones[] stateBones = new mm_v2.Bones[]
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] allFullDOFBones = new mm_v2.Bones[]
    //{ Bone_LeftArm, Bone_RightArm, };
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine,  Bone_Spine1, Bone_Spine2, Bone_Neck,  Bone_LeftHand, Bone_RightHand};
    [HideInInspector]
    public static mm_v2.Bones[] allLimitedDOFBones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftToe, Bone_RightToe, Bone_LeftForeArm, Bone_RightForeArm};

    [HideInInspector]
    public static mm_v2.Bones[] openloopBones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand};

  // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public void applyAction()
    {
        Quaternion[] cur_rotations = MMScript.bone_rotations;
        for (int i = 0; i < allLimitedDOFBones.Length; i++)
        {
            int bone_idx = (int)allLimitedDOFBones[i];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            Vector3 target = ab.ToTargetRotationInReducedSpace_OLD_DO_NOT_USE(cur_rotations[bone_idx]);
            bool use_xdrive = allLimitedDOFBones[i] == Bone_LeftLeg || allLimitedDOFBones[i] == Bone_RightLeg;
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
        for (int i = 0; i < allFullDOFBones.Length; i++)
        {
            int bone_idx = (int)allFullDOFBones[i];
            // Debug.Log($"Cur bone rotations length - {cur_rotations.Length} - bone_idx : {bone_idx}");
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            //Debug.Log($"Setting drive rotation for ab: {ab.gameObject.name}");
            ab.SetDriveRotation(final);
        }

        for (int i = 0; i < openloopBones.Length; i++)
        {
            int bone_idx = (int)openloopBones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            //Debug.Log($"Setting drive rotation for ab: {ab.gameObject.name}");
            ab.SetDriveRotation(final);
        }
    }
    
    void myInit()
    {
        MMScript = kinematicChar.GetComponent<mm_v2>();
        if (!MMScript.is_initalized && !Application.isEditor)
            return;
        kinChar = new CharInfo();
        kinChar.trans = kinematicChar.transform;
        kinChar.boneToTransform = MMScript.boneToTransform;
        kinChar.charObj = kinematicChar;
        nbodies = kinChar.boneToTransform.Length;
        kinChar.boneSurfacePts = new Vector3[nbodies][];

        SimCharController = simulatedChar.GetComponent<SimCharController>();
        if (Application.isEditor)
            SimCharController.initBoneToArtBodies();
        simChar = new CharInfo();
        simChar.trans = simulatedChar.transform;
        simChar.boneToTransform = SimCharController.boneToTransform;
        simChar.root = SimCharController.bone_to_art_body[(int)Bone_Entity];
        simChar.charObj = simulatedChar;
        simChar.boneSurfacePts = new Vector3[nbodies][];
        simChar.boneToArtBody = SimCharController.bone_to_art_body;
        simulatedChar.SetActive(true);

        kinChar.boneWorldPos = new Vector3[stateBones.Length];
        simChar.boneWorldPos = new Vector3[stateBones.Length];

        origin = kinChar.trans.position;
        origin_rot = simChar.boneToTransform[(int)Bone_Entity].rotation;

    }

    private void Awake()
    {
        // Setup the kinematic character
        kinematicChar = Instantiate(kinematicCharPrefab, Vector3.zero, Quaternion.identity);
        // Setup the sim character
        simulatedChar = Instantiate(simulatedCharPrefab, Vector3.zero, Quaternion.identity);
        //Application.targetFrameRate = 60;
    }

    void Start()
    {
        myInit();
        //MMScript.debug_move_every_second = true;
        //SimCharController.remove_joint_limits(sim_char.bone_to_art_body);
    }

    Vector3 origin;
    Quaternion origin_rot;
    [ContextMenu("Create kin char and set to random pose")]
    private void createAndSetKinChar()
    {
        if (kinematicChar == null)
            kinematicChar = Instantiate(kinematicCharPrefab, Vector3.zero, Quaternion.identity);
        MMScript = kinematicChar.GetComponent<mm_v2>();
        MMScript.set_random_pose();

    }
    [ContextMenu("Set sim char to match kin char")]
    private void create_and_set_sim_char()
    {
        Physics.autoSimulation = false;
        if (simulatedChar == null)
            simulatedChar = Instantiate(simulatedCharPrefab, Vector3.zero, Quaternion.identity);
        myInit();
        SimCharController.teleportSimChar(simChar, kinChar);
        Physics.Simulate(physicsSimulateTime);
    }

    [ContextMenu("Set sim char drives to target kin char")]
    private void createAndSetSimCharDrivesToMatchKinChar()
    {
        //Physics.autoSimulation = false;
        if (simulatedChar == null)
            simulatedChar = Instantiate(simulatedCharPrefab, Vector3.zero, Quaternion.identity);
        myInit();

        Transform kinRoot = kinChar.boneToTransform[(int)Bone_Entity];
        simChar.root.TeleportRoot(kinRoot.position, kinRoot.rotation);
        simChar.root.resetJointPhysics();
 
        for (int i = 1; i < 23; i++)
        {
            ArticulationBody body = simChar.boneToArtBody[i];
            Quaternion targetLocalRot = kinChar.boneToTransform[i].localRotation;
            mm_v2.Bones bone = (mm_v2.Bones)i;
            Vector3 TargetRotationInJointSpace;
            // from https://github.com/Unity-Technologies/marathon-envs/blob/58852e9ac22eac56ca46d1780573cc6c32278a71/UnitySDK/Assets/MarathonEnvs/Scripts/ActiveRagdoll003/DebugJoints.cs
            //Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRot) * body.parentAnchorRotation).eulerAngles;
            TargetRotationInJointSpace = body.ToTargetRotationInReducedSpace(targetLocalRot, false);
            body.SetDriveRotation(targetLocalRot, true);
        }
        //Physics.autoSimulation = false;

    }

    [ContextMenu("reset")]
    private void delete_sim_char()
    {
        DestroyImmediate(simulatedChar);
        DestroyImmediate(kinematicChar);
        kinematicChar = null;
        simulatedChar = null;
    }

    int frame = 0;
    private void FixedUpdate()
    {
        if (frame % 5 == 0) {
            Debug.Log($"Teleporting!");
            SimCharController.teleportSimChar(simChar, kinChar);//teleport_sim_char();
            //create_and_set_kin_char();
            //Destroy(MMScript);
        }
        return;
        //apply_action();
        //// Make sure to teleport sim character if kin character teleported
        //bool teleport_sim = MMScript.teleportedThisFixedUpdaet;
        //if (teleport_sim)
        //{
        //    //sim_char.char_trans.rotation = kin_char.char_trans.rotation;
        //    sim_char.root.TeleportRoot(origin, origin_rot);
        //}
        //return;
    }
    private void Update()
    {
        frame++;
    }

}
