using System;
using System.IO;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
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
    internal int EVALUATE_EVERY_K_STEPS = 1;
    internal int N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT = 4;
    internal float EPISODE_END_REWARD = -.5f;
    internal int MAX_EPISODE_LENGTH_SECONDS = 20;
    internal float ACTION_STIFFNESS_HYPERPARAM = .2f;
    //public bool setDriveStiffnessAndDamping = false;
    //public float DriveStiffness = 150f;
    //public float DriveDamping = 20f;
    internal bool resetKinCharOnEpisodeEnd = false;
    internal bool actionsAreEulerRotations = false;
    internal bool actionsAre6DRotations = false;
    internal bool normalizeObservations = false;
    internal bool normalizeLimitedDOFOutputs = true;
    internal bool useGeodesicForAngleDiff = false;
    internal bool normalizeRewardComponents = false;
    internal bool networkControlsAllJoints = false;
    internal float simulationVelocityHalflife;
    internal bool walkOnly = false;

    internal float poseRewardMultiplier = -10f / 23f;
    CharInfo kinChar, simChar;
    GameObject kinematicCharObj;
    internal GameObject simulatedCharObj;

    public BoxCollider groundCollider;
    private float groundColliderY = 0f;
    private float toeColliderRadius = 0f;
    public GameObject simulated_char_prefab;
    public GameObject kinematic_char_prefab;
    public int reportMeanRewardEveryNSteps = 10000;
    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies; 
    private int curFixedUpdate = -1;
    private int lastSimCharTeleportFixedUpdate = 0;
    private bool isInitialized;

    float[] prevActionOutput;
    int numActions;
    int numObservations;
    database motionDB;
    public bool use_debug_mats = false;
    // Used for normalization
    public bool genMinsAndMaxes = false;

    [HideInInspector]
    public static mm_v2.Bones[] stateBones = new mm_v2.Bones[] 
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] fullDOFBones = new mm_v2.Bones[]
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine};
    [HideInInspector]
    public static mm_v2.Bones[] limitedDOFBones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftToe, Bone_RightToe };
    [HideInInspector]
    public static mm_v2.Bones[] openloopBones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand, Bone_LeftShoulder, Bone_RightShoulder};

    Vector3[] bone_pos_mins, bone_pos_maxes, bone_vel_mins, bone_vel_maxes;
    long timeAtStart;
    // 0.2 m side length cube
    // Mass between 0.01 kg and 8 kg
    // The cube is launched at 5 m/s, towards a uniformly sampled location on the vertical axis,
    // from −0.5 m to 0.5 m, centered on the character’s CM
    // Launches occur every second, with the cube remaining in the scene until it is relaunched
    public GameObject projectilePrefab;
    internal GameObject projectile;
    internal Collider projectileCollider;
    internal Rigidbody projectileRB;
    private float lastProjectileLaunchtime = 0f;

    // OVERRIDEN SETTINGS
    internal float LAUNCH_FREQUENCY = 1f;
    internal float LAUNCH_RADIUS = .66f;
    internal float LAUNCH_SPEED = 5f;
    internal bool projectileTraining = false;
    internal int solverIterations;

    // Reward Normalizers 
    //Normalizer posRewardNormalizer, velRewardNormalizer, localPoseRewardNormalizer, cmVelRewardNormalizer, fallFactorNormalizer;
    Normalizer posRewardNormalizer = new Normalizer();
    Normalizer velRewardNormalizer = new Normalizer();
    Normalizer localPoseRewardNormalizer = new Normalizer();
    Normalizer cmVelRewardNormalizer = new Normalizer();

    public override void CollectObservations(VectorSensor sensor)
    {
        if (!isInitialized)
        {
            customInit();
            return;
        }
        sensor.AddObservation(getState());
    }
    // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!isInitialized)
        {
            customInit();
            return;
        }
        float[] curActions = actionBuffers.ContinuousActions.Array;
        float[] finalActions = new float[numActions];
        for (int i = 0; i < numActions; i++)
           finalActions[i] = ACTION_STIFFNESS_HYPERPARAM * curActions[i] + (1 - ACTION_STIFFNESS_HYPERPARAM) * prevActionOutput[i];
        prevActionOutput = finalActions;
        Quaternion[] curRotations = MMScript.bone_rotations;

        if (networkControlsAllJoints)
            applyActionsToAllFullDOFJoints(finalActions, curRotations);
        else if (actionsAre6DRotations)
            applyActionsWith6DRotations(finalActions, curRotations);
        else if (actionsAreEulerRotations)
            applyActionsAsEulerRotations(finalActions, curRotations);
        else
            applyActionsAsAxisAngleRotations(finalActions, curRotations);

        mm_v2.Bones[] limitedDOFBonesToUse = networkControlsAllJoints ? TestDirector.allLimitedDOFBones : limitedDOFBones;
        int limitedDOFActionOffset = networkControlsAllJoints ? TestDirector.allFullDOFBones.Length * 6 :
                                        fullDOFBones.Length * (actionsAre6DRotations ? 6 : 3);

        for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)limitedDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            int finalActionsIdx = i + limitedDOFActionOffset;
            float output = finalActions[finalActionsIdx];
            Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
            var zDrive = ab.zDrive;
            if (normalizeLimitedDOFOutputs) { 
                var scale = (zDrive.upperLimit - zDrive.lowerLimit) / 2f;
                var midpoint = zDrive.lowerLimit + scale;
                var outputZ = (output * scale) + midpoint;
                zDrive.target = targetRotationInJointSpace.z + outputZ;
                ab.zDrive = zDrive;
            } else
            {
                // Angle is in range (-1, 1) => map to (-180, 180)
                float angle = output * 180;
                zDrive.target = targetRotationInJointSpace.z + angle;
                ab.zDrive = zDrive;
            }
        }
        for (int i = 0; i < openloopBones.Length; i++)
        {
            int boneIdx = (int)openloopBones[i];
            Quaternion final = curRotations[boneIdx];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ab.SetDriveRotation(final);
        }
    }

    private void applyActionsAsAxisAngleRotations(float[] finalActions, Quaternion[] curRotations)
    {

        for (int i = 0; i < fullDOFBones.Length; i++)
        {
            int boneIdx = (int)fullDOFBones[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 output = new Vector3(finalActions[i * 3], finalActions[i * 3 + 1], finalActions[i * 3 + 2]);
            float angle = output.magnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            //angle = (angle * 120) - 180;
            Vector3 normalizedOutput = output.normalized;
            Quaternion offset = Quaternion.AngleAxis(angle, normalizedOutput);
            Quaternion final = curRotations[boneIdx] * offset;
            ab.SetDriveRotation(final);
        }
    }

    private void applyActionsAsEulerRotations(float[] finalActions, Quaternion[] curRotations)
    {
        for (int i = 0; i < fullDOFBones.Length; i++)
        {
            int boneIdx = (int)fullDOFBones[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 output = new Vector3(finalActions[i * 3], finalActions[i * 3 + 1], finalActions[i * 3 + 2]);
            Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
            var xdrive = ab.xDrive;
            var scale = (xdrive.upperLimit - xdrive.lowerLimit) / 2f;
            var midpoint = xdrive.lowerLimit + scale;
            var outputX = (output.x * scale) + midpoint;
            xdrive.target = targetRotationInJointSpace.x + outputX;
            ab.xDrive = xdrive;

            var ydrive = ab.yDrive;
            scale = (ydrive.upperLimit - ydrive.lowerLimit) / 2f;
            midpoint = ydrive.lowerLimit + scale;
            var outputY = (output.y * scale) + midpoint;
            ydrive.target = targetRotationInJointSpace.y + outputY;
            ab.yDrive = ydrive;

            var zdrive = ab.zDrive;
            scale = (zdrive.upperLimit - zdrive.lowerLimit) / 2f;
            midpoint = zdrive.lowerLimit + scale;
            var outputZ = (output.z * scale) + midpoint;
            zdrive.target = targetRotationInJointSpace.z + outputZ;
            ab.zDrive = zdrive;    
        }
    }
    private void applyActionsWith6DRotations(float[] finalActions, Quaternion[] curRotations)
    {
        for (int i = 0; i < fullDOFBones.Length; i++)
        {
            int boneIdx = (int)fullDOFBones[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 outputV1 = new Vector3(finalActions[i * 6], finalActions[i * 6 + 1], finalActions[i * 6 + 2]);
            Vector3 outputV2 = new Vector3(finalActions[i * 6 + 3], finalActions[i * 6 + 4], finalActions[i * 6 + 5]);
            Quaternion networkAdjustment = ArtBodyUtils.From6DRepresentation(outputV1, outputV2);
            Quaternion newTargetRot = curRotations[boneIdx] * networkAdjustment;
            ab.SetDriveRotation(newTargetRot.normalized);
        }
    }
    private void applyActionsToAllFullDOFJoints(float[] finalActions, Quaternion[] curRotations)
    {
        for (int i = 0; i < TestDirector.allFullDOFBones.Length; i++)
        {
            int boneIdx = (int)TestDirector.allFullDOFBones[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 outputV1 = new Vector3(finalActions[i * 6], finalActions[i * 6 + 1], finalActions[i * 6 + 2]);
            Vector3 outputV2 = new Vector3(finalActions[i * 6 + 3], finalActions[i * 6 + 4], finalActions[i * 6 + 5]);
            Quaternion networkAdjustment = ArtBodyUtils.From6DRepresentation(outputV1, outputV2);
            Quaternion newTargetRot = curRotations[boneIdx] * networkAdjustment;
            ab.SetDriveRotation(newTargetRot.normalized);
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsout)
    {

        Quaternion[] cur_rotations = MMScript.bone_rotations;
        for (int i = 0; i < fullDOFBones.Length; i++)
        {
            int bone_idx = (int)fullDOFBones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            ab.SetDriveRotation(final);
        }
        for (int i = 0; i < limitedDOFBones.Length; i++)
        {
            mm_v2.Bones bone =  limitedDOFBones[i];
            // Angle is in range (-1, 1) => map to (-180, 180)
            //float angle = final_actions[i] * 180;
            ArticulationBody ab = simChar.boneToArtBody[(int)bone];
            Vector3 target = ab.ToTargetRotationInReducedSpace(cur_rotations[(int) bone], true);
            ArticulationDrive drive = ab.zDrive;
            drive.target = target.z;
            ab.zDrive = drive;
        }
        for (int i = 0; i < openloopBones.Length; i++)
        {
            int bone_idx = (int)openloopBones[i];
            Quaternion final = cur_rotations[bone_idx];
            ArticulationBody ab = simChar.boneToArtBody[bone_idx];
            ab.SetDriveRotation(final);
        }
    }
    private Vector3 feetBozSize;
    void customInit()
    {
        if (use_debug_mats)
        {
            Material RedMatTransparent, WhiteMatTransparent;
#if UNITY_EDITOR
            RedMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/RedMatTransparent.mat", typeof(Material));
            WhiteMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/WhiteMatTransparent.mat", typeof(Material));
#else
            RedMatTransparent = Resources.Load<Material>("RedMatTransparent");
            WhiteMatTransparent = Resources.Load<Material>("WhiteMatTransparent");
#endif
            kinematicCharObj.GetComponent<ArtBodyTester>().set_all_material(WhiteMatTransparent);
            simulatedCharObj.GetComponent<ArtBodyTester>().set_all_material(RedMatTransparent);
        }


        MMScript = kinematicCharObj.GetComponent<mm_v2>();
        if (!MMScript.is_initalized)
            return;
        MMScript.simulation_velocity_halflife = simulationVelocityHalflife;
        MMScript.walk_only = walkOnly;

        kinChar = new CharInfo(nbodies, stateBones.Length);
        kinChar.trans = kinematicCharObj.transform;
        kinChar.boneToTransform = MMScript.boneToTransform;
        kinChar.charObj = kinematicCharObj;
        //nbodies = kinChar.boneToTransform.Length;
        //kinChar.boneSurfacePts = new Vector3[nbodies][];

        SimCharController = simulatedCharObj.GetComponent<SimCharController>();
        SimCharController.is_active = false;
        SimCharController.enabled = true;
        simChar = new CharInfo(nbodies, stateBones.Length);
        simChar.trans = simulatedCharObj.transform;
        simChar.boneToTransform = SimCharController.boneToTransform;
        simChar.root = SimCharController.boneToArtBody[(int)Bone_Entity];
        foreach (var body in simulatedCharObj.GetComponentsInChildren<ArticulationBody>())
        {
            body.solverIterations = solverIterations;
            body.solverVelocityIterations = solverIterations;
        }
        simChar.charObj = simulatedCharObj;
        simChar.boneToArtBody = SimCharController.boneToArtBody;

        for (int i = 0; i < nbodies; i++)
        {
            if (i == (int)Bone_LeftFoot || i == (int)Bone_RightFoot)
            {
                kinChar.boneToCollider[i] = ArtBodyTester.getChildBoxCollider(kinChar.boneToTransform[i].gameObject);
                simChar.boneToCollider[i] = ArtBodyTester.getChildBoxCollider(simChar.boneToTransform[i].gameObject);
                feetBozSize = kinChar.boneToCollider[i].GetComponent<BoxCollider>().size;
            } else { 
                kinChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(kinChar.boneToTransform[i].gameObject);
                simChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(simChar.boneToTransform[i].gameObject);
            }
            kinChar.boneSurfacePts[i] = new Vector3[6];
            kinChar.boneSurfaceVels[i] = new Vector3[6];
            simChar.boneSurfacePts[i] = new Vector3[6];
            simChar.boneSurfaceVels[i] = new Vector3[6];
        }
        groundColliderY = groundCollider.bounds.max.y;
        toeColliderRadius = simChar.boneToCollider[(int)Bone_RightToe].GetComponent<CapsuleCollider>().radius;
        projectile = Instantiate(projectilePrefab, simulatedCharObj.transform.position + Vector3.up, Quaternion.identity);
        projectileCollider = projectile.GetComponent<Collider>();
        projectileRB = projectile.GetComponent<Rigidbody>();
        if (!projectileTraining)
            projectile.SetActive(false);

            UpdateCMData(false);
        bone_pos_mins = new Vector3[stateBones.Length];
        bone_pos_maxes = new Vector3[stateBones.Length];
        bone_vel_mins = new Vector3[stateBones.Length];
        bone_vel_maxes = new Vector3[stateBones.Length];
        if (!genMinsAndMaxes)
            ReadMinsAndMaxes();
        //else
        //MMScript.run_max_speed = true;
        if (networkControlsAllJoints)
            numActions = TestDirector.allFullDOFBones.Length * 6 + TestDirector.allLimitedDOFBones.Length; // 102
        else 
            numActions = (fullDOFBones.Length * (actionsAre6DRotations ? 6 : 3)) + limitedDOFBones.Length;
        //Debug.Log($"numActions: {numActions}");
        numObservations = 88 + numActions; // 131 or 187
        prevActionOutput = new float[numActions];
        isInitialized = true;
    }

    public void Awake()
    {
        timeAtStart = DateTimeOffset.Now.ToUnixTimeSeconds();
        //Debug.Log("MLAgents Director Awake called");
        //Unity.MLAgents.Policies.BehaviorParameters behavior_params = gameObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (motionDB == null)
            motionDB = database.Instance;
        nbodies = motionDB.nbones();
        kinematicCharObj = Instantiate(kinematic_char_prefab, Vector3.zero, Quaternion.identity);
        simulatedCharObj = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        if (Academy.Instance.IsCommunicatorOn)
        {
            int numFixedUpdatesPerSecond = Mathf.CeilToInt(1f / Time.fixedDeltaTime);
            MaxStep = numFixedUpdatesPerSecond * MAX_EPISODE_LENGTH_SECONDS;
        }
        customInit();
        //SimCharController.set_art_body_rot_limits();
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log("OnEpisodeBegin() called");
        if (resetKinCharOnEpisodeEnd)
        {
            MMScript.Reset();
            MMScript.FixedUpdate();
        }
        float verticalOffset = getVerticalOffset();
        //Debug.Log($"Vertical offset: {verticalOffset}");
        SimCharController.teleportSimChar(simChar, kinChar, true, verticalOffset + .025f);
        lastSimCharTeleportFixedUpdate = curFixedUpdate;
    }


    private void FixedUpdate()
    {
        if (!isInitialized)
        {
            customInit();
            return;
        }
        curFixedUpdate++;

        // Make sure to teleport sim character if kin character teleported
        //Debug.Log($"simCharTeleported: {simCharTeleported}");
        //if (simCharTeleported)
        //    Debug.Log($"BOOM : {simCharTeleported}");
        // only update velocity if we did not teleport last frame
        bool updateVelocity = lastSimCharTeleportFixedUpdate + 1 != curFixedUpdate;
        if (MMScript.teleportedThisFixedUpdate)
        {
            //sim_char.char_trans.rotation = kin_char.char_trans.rotation;
            simChar.root.TeleportRoot(Vector3.zero, kinChar.trans.rotation);
            updateVelocity = false;
        }
        // Update CMs 
        UpdateCMData(updateVelocity);
        UpdateBoneObsState(updateVelocity);
        UpdateBoneSurfacePts(updateVelocity);
        bool episodeEnded = calcAndSetRewards();
        if (episodeEnded)
            return;
        // request Decision
        if (curFixedUpdate % EVALUATE_EVERY_K_STEPS == 0)
            RequestDecision();
        //updateMeanReward();
        if (genMinsAndMaxes && curFixedUpdate % 300 == 0)
            WriteMinsAndMaxes();
        if (projectileTraining)
            FireProjectile();
        
    }

    private void FireProjectile()
    {
        if (Time.time - lastProjectileLaunchtime < LAUNCH_FREQUENCY)
            return;
        lastProjectileLaunchtime = Time.time;
        projectileRB.mass = UnityEngine.Random.Range(.01f, 8f);
        // To get XZ position, consider characters position on plane, pick random spot on unit cirlce,
        // and go RADIUS units towards that spot 
        // Y position is uniformly sampled from −0.5 m to 0.5 m, centered on the character’s CM
        float YTarget = UnityEngine.Random.Range(simChar.cm.y - .5f, simChar.cm.y + .5f);
        Vector2 randomUnitCircle = UnityEngine.Random.insideUnitCircle.normalized * LAUNCH_RADIUS;
        Vector3 finalPosition = new Vector3(simChar.cm.x + randomUnitCircle.x, simChar.cm.y, simChar.cm.z + randomUnitCircle.y);
        projectile.transform.position = finalPosition;
        // We want the projectile to be at YTarget when it reaches the target
        // We know its acceleration (-9.8), its final position (YTarget), its starting Y position (simChar.cm.y), we need its
        // current velocity 
        float timeToTravel = LAUNCH_RADIUS / LAUNCH_SPEED; // So if it's traveling 1m at 5m/s, its timeToTravel is .2 seconds
        float changeInY = YTarget - simChar.cm.y;
        float totalAcceleration = timeToTravel * -9.8f; // this will be negative ofc
        float curYVelocity = -totalAcceleration + (changeInY / timeToTravel);
        projectileRB.AddForce(-randomUnitCircle.x * LAUNCH_SPEED, curYVelocity, -randomUnitCircle.y * LAUNCH_SPEED, ForceMode.VelocityChange);
    }

    private void WriteMinsAndMaxes()
    {
        Stream stream;
#if UNITY_EDITOR
        stream = File.Open(@"Assets/Normalization/NormalizationData-" + timeAtStart.ToString() + ".bin", FileMode.Create);
#else
        throw new Exception("Should not be recording mins and maxes outside of Editor");
#endif
        using (stream)
        {
            using (var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8))
            {
                FileUtils.WriteVector(writer, MIN_VELOCITY);
                FileUtils.WriteVector(writer, MAX_VELOCITY);
                for (int i = 0; i < stateBones.Length; i++ )
                {
                    FileUtils.WriteVector(writer, bone_pos_mins[i]);
                    FileUtils.WriteVector(writer, bone_pos_maxes[i]);
                    FileUtils.WriteVector(writer, bone_vel_mins[i]);
                    FileUtils.WriteVector(writer, bone_vel_maxes[i]);
                    //Debug.Log($"{state_bones[i]} Min vel: {bone_vel_mins[i]} Max Vel: {bone_vel_maxes[i]}");
                }
            }
        }

    }
    private void ReadMinsAndMaxes()
    {
        Stream stream;
#if UNITY_EDITOR
        stream = File.Open(@"Assets/Normalization/NormalizationData.bin", FileMode.Open);
#else
        var textAsset = Resources.Load<TextAsset>("NormalizationData");
        stream = new MemoryStream(textAsset.bytes);
#endif
        using (stream)
        {
            using (var reader = new BinaryReader(stream, System.Text.Encoding.UTF8))
            {
                FileUtils.ReadVector(reader, ref MIN_VELOCITY);
                FileUtils.ReadVector(reader, ref MAX_VELOCITY);
                //Debug.Log($"Min vel: {MIN_VELOCITY} Max Vel: {MAX_VELOCITY}");
                for (int i = 0; i < stateBones.Length; i++)
                {
                    FileUtils.ReadVector(reader, ref bone_pos_mins[i]);
                    FileUtils.ReadVector(reader, ref bone_pos_maxes[i]);
                    FileUtils.ReadVector(reader, ref bone_vel_mins[i]);
                    FileUtils.ReadVector(reader, ref bone_vel_maxes[i]);
                    //Debug.Log($"{state_bones[i]} Min Pos: {bone_pos_mins[i]} Max Pos: {bone_pos_maxes[i]}");
                    //Debug.Log($"{state_bones[i]} Min vel: {bone_vel_mins[i]} Max Vel: {bone_vel_maxes[i]}");
                }
            }
        }
    }

    private void UpdateCMData(bool updateVelocity)
    {
        Vector3 newSimCM = getCM(simChar.boneToTransform);
        simChar.cmVel = updateVelocity ? (newSimCM - simChar.cm) / Time.fixedDeltaTime : simChar.cmVel;
        simChar.cm = newSimCM;
        Vector3 newKinCM = getCM(kinChar.boneToTransform);
        kinChar.cmVel = updateVelocity ? (newKinCM - kinChar.cm) / Time.fixedDeltaTime : kinChar.cmVel;
        kinChar.cm = newKinCM;
        if (genMinsAndMaxes && updateVelocity) { 
            SimCharController.updated_mins_and_maxes(kinChar.cmVel, ref MIN_VELOCITY, ref MAX_VELOCITY);
            SimCharController.updated_mins_and_maxes(simChar.cmVel, ref MIN_VELOCITY, ref MAX_VELOCITY);
        }
    }

    private void UpdateBoneSurfacePts(bool updateVelocity)
    {
        for (int i = 1; i < 23; i++)
        {

            Vector3[] newKinBoneSurfacePts = new Vector3[6];
            Vector3[] newSimBoneSurfacePts = new Vector3[6];

            getSixPointsOnCollider(kinChar.boneToCollider[i], ref newKinBoneSurfacePts, (mm_v2.Bones) i);
            getSixPointsOnCollider(simChar.boneToCollider[i], ref newSimBoneSurfacePts, (mm_v2.Bones) i);

            Vector3[] prevKinSurfacePts = kinChar.boneSurfacePts[i];
            Vector3[] prevSimSurfacePts = simChar.boneSurfacePts[i];

            for (int j = 0; j < 6; j++)
            {
                newKinBoneSurfacePts[j] = resolvePosInKinematicRefFrame(newKinBoneSurfacePts[j]);
                newSimBoneSurfacePts[j] = resolvePosInSimRefFrame(newSimBoneSurfacePts[j]);

                if (updateVelocity) {
                    kinChar.boneSurfaceVels[i][j] = (newKinBoneSurfacePts[j] - prevKinSurfacePts[j]) / Time.fixedDeltaTime;
                    simChar.boneSurfaceVels[i][j] = (newSimBoneSurfacePts[j] - prevSimSurfacePts[j]) / Time.fixedDeltaTime;
                }
            }
            kinChar.boneSurfacePts[i] = newKinBoneSurfacePts;
            simChar.boneSurfacePts[i] = newSimBoneSurfacePts;
        }
    }

    private void UpdateBoneObsState(bool updateVelocity)
    {
        CharInfo curInfo = kinChar;
        for (int i = 0; i < 2; i++)
        {
            float[] copyInto = curInfo.boneState;
            int copyIdx = 0;
            for (int j = 0; j < stateBones.Length; j++)
            {
                mm_v2.Bones bone = stateBones[j];
                // Compute position of bone
                Vector3 boneWorldPos = curInfo.boneToTransform[(int)bone].position;
                Vector3 boneLocalPos = i == 0 ? resolvePosInKinematicRefFrame(boneWorldPos) : resolvePosInSimRefFrame(boneWorldPos);
                //Vector3 bone_relative_pos = Utils.quat_inv_mul_vec3(relative_rot, bone_local_pos);
                Vector3 prevBonePos = curInfo.boneWorldPos[j];
                Vector3 boneVel = (boneWorldPos - prevBonePos) / Time.fixedDeltaTime;
                boneVel = resolveVelInKinematicRefFrame(boneVel);
                if (genMinsAndMaxes && curFixedUpdate > 30)
                {
                    SimCharController.updated_mins_and_maxes(boneLocalPos, ref bone_pos_mins[j], ref bone_pos_maxes[j]);
                    if (updateVelocity)
                        SimCharController.updated_mins_and_maxes(boneVel, ref bone_vel_mins[j], ref bone_vel_maxes[j]);
                }
                if (normalizeObservations)
                {
                    boneLocalPos = normalizeBonePos(boneLocalPos, j);
                    boneVel = normalizeBoneVel(boneVel, j);
                }
                copyVecIntoArray(ref copyInto, ref copyIdx, boneLocalPos);
                if (updateVelocity)
                    copyVecIntoArray(ref copyInto, ref copyIdx, boneVel);
                else
                    copyIdx += 3;
                curInfo.boneWorldPos[j] = boneWorldPos;
            }
            curInfo = simChar;
        }
    }

    public void AssignLayer(int layer)
    {
        simulatedCharObj.layer = layer;
        foreach (var child in simulatedCharObj.GetComponentsInChildren<Transform>())
            child.gameObject.layer = layer;

        projectile.layer = layer;
    }


    private float meanReward = 0f;
    internal float finalReward = 0f;
    // returns TRUE if episode ended
    private bool calcAndSetRewards()
    {
        bool heads1mApart;
        double posReward, velReward, localPoseReward, cmVelReward, fallFactor;
        calcFallFactor(out fallFactor, out heads1mApart);
        if (heads1mApart)
        {
            finalReward = EPISODE_END_REWARD;
            //updateMeanReward(-.5f);
            SetReward(EPISODE_END_REWARD);
            EndEpisode();
            return true;
        }
        // Calc them even if we don't use them for normalizer sake
        calcPosAndVelReward(out posReward, out velReward);
        calcLocalPoseReward(out localPoseReward);
        calcCMVelReward(out cmVelReward);
        if (curFixedUpdate - N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT < lastSimCharTeleportFixedUpdate)
        {
            finalReward = 0f;
        } else
        {
            finalReward = (float)(fallFactor * (posReward + velReward + localPoseReward + cmVelReward));
            Debug.Log($"finalReward: {finalReward} fall_factor: {fallFactor}, pos_reward: {posReward}, vel_reward: {velReward}, local_pose_reward: {localPoseReward}, cm_vel_reward: {cmVelReward}");
        }
        //Debug.Log($"final_reward: {finalReward}");
        //updateMeanReward(final_reward);
        SetReward(finalReward);
        return false;
    }

    private void updateMeanReward()
    {
        if (curFixedUpdate % reportMeanRewardEveryNSteps == 0)
        {
            Debug.Log($"Step {curFixedUpdate} mean reward last {reportMeanRewardEveryNSteps} is: {meanReward}");
            meanReward = 0f;
        }
        meanReward += (finalReward / (float)reportMeanRewardEveryNSteps);
    }
   
    // Roughly 6.81 m/s
    Vector3 MAX_VELOCITY = new Vector3(4.351938f, 1.454015f, 5.032811f);
    Vector3 MIN_VELOCITY = new Vector3(-4.351710f, -1.688771f, -4.900798f);
    Vector3 normalizeCMVelocity(Vector3 vel)
    {
        float new_x = normalizeFloat(vel.x, MIN_VELOCITY.x, MAX_VELOCITY.x);
        float new_y = normalizeFloat(vel.y, MIN_VELOCITY.y, MAX_VELOCITY.y);
        float new_z = normalizeFloat(vel.z, MIN_VELOCITY.z, MAX_VELOCITY.z);
        return new Vector3(new_x, new_y, new_z);
    }
    Vector2 MIN_DESIRED_SPEED = new Vector2(-2.5f, -2f);
    Vector2 MAX_DESIRED_SPEED = new Vector2(2.5f, 3f);

    Vector2 normalizeDesiredVelocity(Vector3 vel)
    {
        float new_x = normalizeFloat(vel.x, MIN_DESIRED_SPEED.x, MAX_DESIRED_SPEED.x);
        float new_z = normalizeFloat(vel.z, MIN_DESIRED_SPEED.y, MAX_DESIRED_SPEED.y);
        return new Vector3(new_x, 0f, new_z);
    }
    Vector3 normalizeBonePos(Vector3 pos, int idx)
    {
        float new_x = normalizeFloat(pos.x, bone_pos_mins[idx].x, bone_pos_maxes[idx].x);
        float new_y = normalizeFloat(pos.y, bone_pos_mins[idx].y, bone_pos_maxes[idx].y);
        float new_z = normalizeFloat(pos.z, bone_pos_mins[idx].z, bone_pos_maxes[idx].z);
        return new Vector3(new_x, new_y, new_z);
    }
    Vector3 normalizeBoneVel(Vector3 vel, int idx)
    {
        float new_x = normalizeFloat(vel.x, bone_vel_mins[idx].x, bone_vel_maxes[idx].x);
        float new_y = normalizeFloat(vel.y, bone_vel_mins[idx].y, bone_vel_maxes[idx].y);
        float new_z = normalizeFloat(vel.z, bone_vel_mins[idx].z, bone_vel_maxes[idx].z);
        return new Vector3(new_x, new_y, new_z);
    }
    float normalizeFloat(float f, float min, float max)
    {
        return (f - min) / (max - min + float.Epsilon);
    }


    /*
    At each control step the policy is provided with a state s in R^110

    adding my own: difference in center of mass for each character
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
    Idx to value mapping: 
    0-2   : kinematic cm vel
    3-5   : sim cm vel
    6-8   : difference between sim cm vel and kin cm vel
    9-10  : desired velocity
    11-12 : the diff between sim cm horizontal vel and desired vel

    */
    float[] getState()
    {
        float[] state = new float[numObservations];
        int state_idx = 0;
        Vector3 cm_distance = kinChar.cm - simChar.cm; // Since we terminate when head distance > 1, cm distance should be between 0~1 anyway
        copyVecIntoArray(ref state, ref state_idx, cm_distance);

        Vector3 kin_cm_vel_normalized = resolveVelInKinematicRefFrame(kinChar.cmVel);
        AddGizmoLine(kinChar.cm, kinChar.cm + kinChar.cmVel, Color.red);
        if (normalizeObservations)
            kin_cm_vel_normalized = normalizeCMVelocity(kin_cm_vel_normalized);
        
        copyVecIntoArray(ref state, ref state_idx, kin_cm_vel_normalized);
        Vector3 sim_cm_vel_normalized = resolveVelInKinematicRefFrame(simChar.cmVel);
        if (normalizeObservations)
            sim_cm_vel_normalized = normalizeCMVelocity(sim_cm_vel_normalized);
        copyVecIntoArray(ref state, ref state_idx, sim_cm_vel_normalized);

        // Copy v (sim) - v(kin)
        Vector3 vel_diff = simChar.cmVel - kinChar.cmVel;
        copyVecIntoArray(ref state, ref state_idx, normalizeObservations ? normalizeCMVelocity(vel_diff) : vel_diff);

        // The desired horizontal CM velocity from user-input is also considered v(des) - R^2
        Vector3 desired_vel = MMScript.desired_velocity;
        desired_vel = resolveVelInKinematicRefFrame(desired_vel);

        Vector3 desired_vel_normalized = desired_vel;
        if (normalizeObservations)
            desired_vel_normalized = normalizeDesiredVelocity(desired_vel);
        copyVecIntoArray(ref state, ref state_idx, new Vector2(desired_vel_normalized.x, desired_vel_normalized.z));

        //The diff between current simulated character horizontal
        //CM velocity and v(des) = v(diff) R ^ 2
        Vector3 v_diff = desired_vel_normalized - sim_cm_vel_normalized;
        copyVecIntoArray(ref state, ref state_idx, new Vector2(v_diff.x, v_diff.z));

        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then (s(sim) - s(kin))
        for (int i = 0; i < 36; i++)
            state[state_idx++] = kinChar.boneState[i];
        for (int i = 0; i < 36; i++)
            // In order to keep it between [-1, 1] 
            state[state_idx++] = normalizeObservations ? (simChar.boneState[i] - kinChar.boneState[i]) / 2 : simChar.boneState[i] - kinChar.boneState[i];
        for (int i = 0; i < numActions; i++)
            state[state_idx++] = prevActionOutput[i];

   
        if (state_idx != numObservations)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything, 6D: {actionsAre6DRotations}");
   
        //for(int i = 0; i < 110; i++)
        //{
        //    if (state[i] > 1 || state[i] < -1) { 
        //        Debug.Log($"State[{i}] is {state[i]}");
        //        if (i >= 9 && i <= 10)
        //        {
        //            Debug.Log($"Desired vel: {desired_vel} ");
        //        }
        //    }
        //}
        if (genMinsAndMaxes)
            state = new float[numObservations];

        return state;

    }

    // Gets CoM in world position
    public static Vector3 getCM(Transform[] bone_to_transform, Vector3[] global_bone_positions = null)
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
            Vector3 child_center = global_bone_positions == null ? getChildColliderCenter(t.gameObject) : global_bone_positions[i];
            CoM += mass * child_center;
            total_mass += mass;

        }
        return CoM / total_mass;
    }

    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    Vector3 resolveVelInKinematicRefFrame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, vel);
    }
    Vector3 resolvePosInKinematicRefFrame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - kinChar.cm);
    }
    Vector3 resolvePosInSimRefFrame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return Utils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - simChar.cm);
    }

    void copyVecIntoArray(ref float[] state, ref int start_idx, Vector3 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        state[start_idx + 2] = v.z;
        start_idx += 3;
    }
    void copyVecIntoArray(ref float[] state, ref int start_idx, Vector2 v)
    {
        state[start_idx] = v.x;
        state[start_idx + 1] = v.y;
        start_idx += 2;
    }

    public static Vector3 getChildColliderCenter(GameObject child)
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

    void calcPosAndVelReward(out double posReward, out double velReward)
    {
        // Position reward
        double posDiffsSum = 0f;
        double velDiffsSum = 0f;
        for (int i = 1; i < 23; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                posDiffsSum += (kinChar.boneSurfacePts[i][j] - simChar.boneSurfacePts[i][j]).magnitude;
                velDiffsSum += (kinChar.boneSurfaceVels[i][j] - simChar.boneSurfaceVels[i][j]).magnitude;
            }
        }
        posReward = Math.Exp((-10f / (nbodies * 6)) *  posDiffsSum );
        velReward = Math.Exp((-1f / (nbodies * 6)) *  velDiffsSum );
        if (normalizeRewardComponents)
        {
            posReward = posRewardNormalizer.getNormalized((float)posReward);
            velReward = velRewardNormalizer.getNormalized((float)velReward);
        }
    }

    void calcLocalPoseReward(out double poseReward)
    {
        double totalLoss = 0;
        for (int i = 1; i < 23; i++)
        {
            Transform kinBone = kinChar.boneToTransform[i];
            Transform simBone = simChar.boneToTransform[i];
            float loss;
            // After some testing, the results from either of these should be identical

            if (useGeodesicForAngleDiff)
            {
                Matrix4x4 kinRotation = Matrix4x4.Rotate(kinBone.localRotation);
                Matrix4x4 simRotation = Matrix4x4.Rotate(simBone.localRotation);
                Matrix4x4 lossMat = (simRotation * kinRotation.transpose);
                float trace = lossMat[0, 0] + lossMat[1, 1] + lossMat[2, 2];
                // Need clamping because Acos will throw NAN for values outside [-1, 1]
                float traceClamped = Mathf.Clamp((trace - 1) / 2, -1f, 1f);
                loss = Mathf.Acos(traceClamped);
            //geodesicTotalRewardSum += loss;
            } else { 

                // From Stack Overflow:
                //If you want to find a quaternion diff such that diff * q1 == q2, then you need to use the multiplicative inverse:
                // diff * q1 = q2  --->  diff = q2 * inverse(q1)
                Quaternion diff = simBone.localRotation * Quaternion.Inverse(kinBone.localRotation);
                //Vector3 axis;
                // https://stackoverflow.com/questions/21513637/dot-product-of-two-quaternion-rotations
                // angle = 2*atan2(q.vec.length(), q.w)
                //double sqrd_dot =  Math.Pow(Quaternion.Dot(kin_bone.localRotation, sim_bone.localRotation), 2);
                //double angle = Math.Acos(2 * sqrd_dot - 1);
                //diff.ToAngleAxis(out angle, out axis);

                Vector3 diff_vec = new Vector3(diff.x, diff.y, diff.z);
                double angle = 2 * Math.Atan2(diff_vec.magnitude, diff.w);
                // We want the magnitude of the diff so we take abs value
                angle = Math.Abs(GeoUtils.wrap_radians((float)angle));
                //double unity_angle = Quaternion.Angle(sim_bone.localRotation, kin_bone.localRotation);
                loss = (float) angle;
            }
            totalLoss += loss;
            //Debug.Log($"Bone: {(mm_v2.Bones)i} Quaternion loss: {loss}, geoDesicloss {geodesicLoss}");
        }
        poseReward = Math.Exp(-1f * poseRewardMultiplier * totalLoss);
        if (normalizeRewardComponents)
        {
            poseReward = localPoseRewardNormalizer.getNormalized((float)poseReward);
        }
        //double geodesicTotalReward = Math.Exp(-1f * poseRewardMultiplier * geodesicTotalRewardSum);
        //Debug.Log($"pose_reward_sum: {pose_reward_sum} Quaternion reward: {pose_reward} " +
        //    $"  geodesicTotalRewardSum: {geodesicTotalRewardSum} geodesicTotalReward: {geodesicTotalReward}");

    }

    void calcCMVelReward(out double cmVelReward)
    {
        cmVelReward = Math.Exp(-1d * (kinChar.cmVel - simChar.cmVel).magnitude);
        if (normalizeRewardComponents)
            cmVelReward = cmVelRewardNormalizer.getNormalized((float)cmVelReward);
    }

    void calcFallFactor(out double fallFactor, out bool heads1mApart)
    {
        Vector3 kinHeadPos = kinChar.boneToTransform[(int)Bone_Head].position;
        Vector3 simHeadPos = simChar.boneToTransform[(int)Bone_Head].position;
        float headDistance = (kinHeadPos - simHeadPos).magnitude;
        heads1mApart = headDistance > 1f;
        fallFactor = Math.Clamp(1.3 - 1.4 * headDistance, 0d, 1d);
    }

    // Get 6 points on capsule object
    // first we start off at the center, and the 6 points are given by
    // center plus-minus radius on y dimension
    // center plus-minus radius on z dimension
    // and since capsule is oriented on x dimension, the tippy tops are,
    // as we previously calculated,
    // center plus minus height/2 on x dimension 
    public static void getSixPointsOnCapsule(GameObject obj, ref Vector3[] outputs)
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

    private float getBottomMostPointOnFoot(Transform foot)
    {
        float x = feetBozSize.x / 2;
        float y = feetBozSize.y / 2;
        float z = feetBozSize.z / 2;
        Vector3 topLeft = foot.TransformPoint(new Vector3(x, -y, z));
        Vector3 topRight = foot.TransformPoint(new Vector3(x, -y, -z));
        Vector3 bottomLeft = foot.TransformPoint(new Vector3(-x, -y, z));
        Vector3 bottomRight = foot.TransformPoint(new Vector3(-x, -y, -z));
        AddGizmoSphere(topLeft, Color.red);
        AddGizmoSphere(topRight, Color.red);
        AddGizmoSphere(bottomLeft, Color.red);
        AddGizmoSphere(bottomRight, Color.red);

        return Mathf.Min(topLeft.y, topRight.y, bottomLeft.y, bottomRight.y);
    }

    private float getVerticalOffset()
    {
        ClearGizmos();
        Transform leftFoot = kinChar.boneToCollider[(int)Bone_LeftFoot].transform;
        Transform rightFoot = kinChar.boneToCollider[(int)Bone_RightFoot].transform;
        float minPointOnFoot = Mathf.Min(getBottomMostPointOnFoot(leftFoot), getBottomMostPointOnFoot(rightFoot));
        Transform leftToe = kinChar.boneToTransform[(int)Bone_LeftToe];
        Transform rightToe = kinChar.boneToTransform[(int)Bone_RightToe];
        float minToeY = Mathf.Min(leftToe.position.y, rightToe.position.y) - toeColliderRadius;
        float maxGroundPenetration = Mathf.Max(0f, groundColliderY - Mathf.Min(minPointOnFoot , minToeY));
        return maxGroundPenetration;
    }
    public static void getSixPointsOnCollider(GameObject obj, ref Vector3[] outputs, mm_v2.Bones bone)
    {
        if (bone == Bone_LeftFoot || bone == Bone_RightFoot)
            getSixPointsOnBox(obj, ref outputs);
        else
            getSixPointsOnCapsule(obj, ref outputs);

    }
    public static void getSixPointsOnBox(GameObject obj, ref Vector3[] outputs)
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
    private void debugCapsuleSurfacePts()
    {
        customInit();
        for (int i = 1; i < 23; i++)
        {
            // Get bone, transform, child capsule object
            mm_v2.Bones bone = (mm_v2.Bones)i;
            GameObject kin_collider_obj = kinChar.boneToCollider[i];
            Vector3[] gizmos = new Vector3[6];
            getSixPointsOnCollider(kin_collider_obj, ref gizmos, bone);            
            foreach (Vector3 v in gizmos)
                AddGizmoSphere(v, Color.blue);
        }
    }

    public float gizmoSphereRad = .01f;


    List<(Vector3 pos, Color color)> gizmoSpheres;
    List<(Vector3 a, Vector3 b, Color color)> gizmoLines;

    private void AddGizmoSphere(Vector3 v, Color c)
    {
        if (gizmoSpheres == null)
            gizmoSpheres = new List<(Vector3, Color)>();
        gizmoSpheres.Add((v, c));
    }
    private void AddGizmoLine(Vector3 a, Vector3 b, Color color)
    {
        if (gizmoLines == null)
            gizmoLines = new List<(Vector3, Vector3, Color)>();
        gizmoLines.Add((a, b, color));
    }
    [ContextMenu("Reset gizmos")]
    private void ClearGizmos()
    {
        if (gizmoSpheres != null)
            gizmoSpheres.Clear();
        if (gizmoLines != null)
            gizmoLines.Clear();
    }
    public bool drawGizmos = false;
    private void OnDrawGizmos()
    {
        if (!drawGizmos)
            return;
        if (gizmoSpheres != null)
        {
            foreach ((Vector3 v, Color c) in gizmoSpheres)
            {
                Gizmos.color = c;
                Gizmos.DrawSphere(v, gizmoSphereRad);
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

    [ContextMenu("Testing")]
    private void testFunc()
    {
        Quaternion q1 = Quaternion.AngleAxis(180f, Vector3.right);
        Quaternion q2 = Quaternion.AngleAxis(0f, Vector3.right);
        Debug.Log($"{Quaternion.Dot(q1, q2)}");
        Debug.Log($"{Quaternion.Dot(q1, q2)}");

    }
}
