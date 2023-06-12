using System.Collections;
using System.Collections.Generic;
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
    private Quaternion[] startingRotations;

    [HideInInspector]
    public static mm_v2.Bones[] stateBones = new mm_v2.Bones[]
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] allFullDOFBones = new mm_v2.Bones[]
    //{ Bone_LeftArm, Bone_RightArm, };
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Hips, Bone_Spine,  Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftHand, Bone_RightHand, mm_v2.Bones.Bone_LeftShoulder, mm_v2.Bones.Bone_RightShoulder};
    [HideInInspector]
    public static mm_v2.Bones[] allLimitedDOFBones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftToe, Bone_RightToe, Bone_LeftForeArm, Bone_RightForeArm};

    [HideInInspector]
    public static mm_v2.Bones[] openloopBones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand};

    public static mm_v2.Bones[] debugBones = new mm_v2.Bones[]
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
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[bone_idx], true);
            ArticulationDrive drive = ab.zDrive;
            drive.target = target.z;
            ab.zDrive = drive;
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
        simChar.root = SimCharController.boneToArtBody[(int)Bone_Entity];
        simChar.charObj = simulatedChar;
        simChar.boneSurfacePts = new Vector3[nbodies][];
        simChar.boneToArtBody = SimCharController.boneToArtBody;
        simulatedChar.SetActive(true);
        startingRotations = SimCharController.startingRotations;

        kinChar.boneWorldPos = new Vector3[stateBones.Length];
        simChar.boneWorldPos = new Vector3[stateBones.Length];

        //origin = kinChar.trans.position;
        //origin_rot = simChar.boneToTransform[(int)Bone_Entity].rotation;

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


    int frame = 0;
    private void FixedUpdate()
    {
        applyAction();
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

    [ContextMenu("Create kin char and set to random pose")]
    private void createAndSetKinChar()
    {
        if (kinematicChar == null)
            kinematicChar = Instantiate(kinematicCharPrefab, Vector3.zero, Quaternion.identity);
        MMScript = kinematicChar.GetComponent<mm_v2>();
        MMScript.set_random_pose();

    }
    [ContextMenu("Teleport sim char to match kin char")]
    private void createAndTeleportSimChar()
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

        ArticulationBody[] bodies = simChar.boneToArtBody;
        Transform[] targetTransforms = kinChar.boneToTransform;
        List<int> startIndexes = new List<int>();
        List<float> driveTargets = new List<float>();

        simChar.root.GetDofStartIndices(startIndexes);
        simChar.root.GetDriveTargets(driveTargets);
        //Debug.Log("Drive targets before");
        //foreach (var target in driveTargets)
        //    Debug.Log($"{target}");
        //ArtBodyUtils.SetDriveRotations(ref bodies, ref targetTransforms, startingRotations, ref startIndexes, ref driveTargets);

        //Debug.Log("Drive targets after");
        //foreach (var target in driveTargets)
        //    Debug.Log($"{target}");
        //return;
        for (int i = 1; i < 23; i++)
        {
            ArticulationBody body = simChar.boneToArtBody[i];
            Quaternion targetLocalRot = kinChar.boneToTransform[i].localRotation;
            body.SetDriveRotation(targetLocalRot);
        }
        Physics.autoSimulation = false;
    }
    [ContextMenu("Teleport root of sim char to kin char root")]
    private void teleportSimRootToKinRoot()
    {
        myInit();
        Transform kinRoot = kinChar.boneToTransform[(int)Bone_Entity];
        simChar.root.TeleportRoot(kinRoot.position, kinRoot.rotation);
        simChar.root.resetJointPhysics();
    }


    [ContextMenu("reset")]
    private void delete_sim_char()
    {
        DestroyImmediate(simulatedChar);
        DestroyImmediate(kinematicChar);
        kinematicChar = null;
        simulatedChar = null;
    }


    //struct TestStruct {
    //    int value;
    //    public TestStruct(int v)
    //    {
    //        value = v;
    //    }
    //    public static TestStruct operator -(TestStruct a) => new TestStruct(-a.value);

    //    public void isNegative()
    //    {
    //        if (value < 0)
    //        {
    //            Debug.Log("Val is negative");
    //        } else
    //        {
    //            Debug.Log("Val is POSITIVE");
    //        }
    //    }
    //}
    //[ContextMenu("Test c# order of operators")]
    //private void testOperatorOrder()
    //{
    //    TestStruct t = new TestStruct(5);
    //    -t.isNegative();
    //}
}
