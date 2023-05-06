using System;
using System.IO;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
using UnityEditor;
using System.Text;

public struct CharInfo
{
    public Transform[]  boneToTransform;
    public GameObject[] boneToCollider;
    public mm_v2 MMScript;
    public GameObject charObj;
    public Transform  trans;
    public ArticulationBody root;
    public Vector3 cm; // prev center of mass
    public Vector3 cmVel;
    public Vector3[] boneWorldPos;
    public Vector3[][] boneSurfacePts;
    public Vector3[][] boneSurfacePtsWorldSpace;
    public Vector3[][] boneSurfaceVels;
    public ArticulationBody[] boneToArtBody;
    public float[] boneState;
    public CharInfo(int nbodies, int numStateBones) : this()
    {
        boneSurfacePts = new Vector3[nbodies][];
        boneSurfacePtsWorldSpace = new Vector3[nbodies][];
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
    private ConfigWriter _config;
    CharInfo kinChar, simChar;
    GameObject kinematicCharObj;
    internal GameObject simulatedCharObj;

    public BoxCollider groundCollider;
    private float groundColliderY = 0f;
    private float toeColliderRadius = 0f;
    public GameObject simulated_char_type2_prefab;
    public GameObject kinematic_char_type2_prefab;
    public GameObject simulated_handmade_char_prefab;
    public GameObject kinematic_handmade_char_prefab;
    public int reportMeanRewardEveryNSteps = 10000;
    private mm_v2 MMScript;
    private SimCharController SimCharController;
    private int nbodies; 
    private int curFixedUpdate = -1;
    private int lastSimCharTeleportFixedUpdate = -1;
    private bool isInitialized;

    float[] prevActionOutput;
    int numActions;
    int numObservations;
    database motionDB;
    public bool use_debug_mats = false;
    // Used for normalization
    private Vector3 lastKinRootPos = Vector3.zero;

    [HideInInspector]
    public static mm_v2.Bones[] stateBones = new mm_v2.Bones[] 
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] fullDOFBones = new mm_v2.Bones[]
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine};

    [HideInInspector]
    public static mm_v2.Bones[] extendedfullDOFBones = new mm_v2.Bones[]
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Hips, Bone_Spine,  Bone_Spine1, Bone_Spine2, Bone_LeftShoulder, Bone_RightShoulder};
    [HideInInspector]
    public static mm_v2.Bones[] limitedDOFBones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg };
    [HideInInspector]
    public static mm_v2.Bones[] extendedLimitedDOFBones = new mm_v2.Bones[]
    {  Bone_LeftLeg, Bone_RightLeg, Bone_LeftForeArm, Bone_RightForeArm};
    [HideInInspector]
    public static mm_v2.Bones[] openloopBones = new mm_v2.Bones[]
      {  Bone_Hips, Bone_Spine1, Bone_Spine2, Bone_Neck, Bone_Head, Bone_LeftForeArm, Bone_LeftHand, Bone_RightForeArm, Bone_RightHand, Bone_LeftShoulder, Bone_RightShoulder};

    public static mm_v2.Bones[] alwaysOpenloopBones = new mm_v2.Bones[]
    { Bone_Neck, Bone_Head, Bone_LeftHand, Bone_RightHand, Bone_LeftToe, Bone_RightToe};

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
    public  bool debug = false;
    public bool updateVelOnTeleport = true;
    public bool resetKinToSimOnFail = false;
    // Reward Normalizers 
    //Normalizer posRewardNormalizer, velRewardNormalizer, localPoseRewardNormalizer, cmVelRewardNormalizer, fallFactorNormalizer;
    Normalizer posRewardNormalizer = new Normalizer();
    Normalizer velRewardNormalizer = new Normalizer();
    Normalizer localPoseRewardNormalizer = new Normalizer();
    Normalizer cmVelRewardNormalizer = new Normalizer();
    private Unity.MLAgents.Policies.BehaviorParameters behaviorParameters;

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(getState());
    }
    private void applyActions(float[] finalActions)
    {

        Quaternion[] curRotations = MMScript.bone_rotations;

        mm_v2.Bones[] fullDOFBonesToUse = _config.networkControlsAllJoints ? extendedfullDOFBones : fullDOFBones;
        int actionIdx = 0;
        if (_config.actionsAre6DRotations)
            applyActionsWith6DRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
        else if (_config.actionsAreEulerRotations)
            applyActionsAsEulerRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
        else
            applyActionsAsAxisAngleRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);

        mm_v2.Bones[] limitedDOFBonesToUse = _config.networkControlsAllJoints ? extendedLimitedDOFBones : limitedDOFBones;

        for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
        {
            // Only inlcuded for compatibility with CoMVelFixed Run - skip toe input steps
            //if (i == 2 && numActions == 42)
            //    actionIdx += 2;
            int boneIdx = (int)limitedDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            float output = finalActions[actionIdx];
            actionIdx++;
            Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
            var zDrive = ab.zDrive;
            if (_config.fullRangeEulerOutputs)
            {
                var outputZ = output * 180f;
                zDrive.target = targetRotationInJointSpace.z + outputZ;
                ab.zDrive = zDrive;
            }
            else
            {
                var scale = zDrive.upperLimit - zDrive.lowerLimit;
                float angle = output * scale;
                zDrive.target = targetRotationInJointSpace.z + angle;
                ab.zDrive = zDrive;
            }
        }
        mm_v2.Bones[] openloopBonesBonesToUse = _config.networkControlsAllJoints ? alwaysOpenloopBones : openloopBones;
        for (int i = 0; i < openloopBonesBonesToUse.Length; i++)
        {
            int boneIdx = (int)openloopBonesBonesToUse[i];
            Quaternion final = curRotations[boneIdx];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ab.SetDriveRotation(final);
        }
        if (_config.setDriveTargetVelocities) { 
            //Quaternion[] curBoneRots = MMScript.bone_rotations;
            for (int i = 1; i < 23; i++)
            {
                ArticulationBody ab = simChar.boneToArtBody[i];
                //Quaternion deltaRotation = curBoneRots[i] * Quaternion.Inverse(prevKinRots[i]);
                //float angle = 2 * Mathf.Acos(Mathf.Clamp(deltaRotation.w, -1, 1));
                //Vector3 axis = new Vector3(deltaRotation.x, deltaRotation.y, deltaRotation.z).normalized;
                //float angularVelocity = angle / Time.fixedDeltaTime; // Angular velocity in radians per second
                //Debug.Log($"{(mm_v2.Bones)i} has deltaRotation.w: {deltaRotation.w} angle:{angle} angularVelocity: {angularVelocity}");
                Vector3 targetAngularVelocity = MMScript.bone_angular_velocities[i]; //angularVelocity * axis;
                ab.SetDriveTargetVelocity(targetAngularVelocity);
            }
            //prevKinRots = curBoneRots;
        }
    }
    bool isFirstAction = true;
    // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        //Debug.Log($"{Time.frameCount} : Applying Python Action on ML Agent");

        float[] curActions = actionBuffers.ContinuousActions.Array;

        // TODO: Exp. with different methods of low pass filtering; instead of filtering the floats
        // one by one maybe I should slerp the resultant quaternions instead? 
        if (_config.actionsAre6DRotations && isFirstAction)
        {
            setFirstActionsAsIdentityRots(curActions);
            isFirstAction = false;
        }
        float[] finalActions = new float[numActions];
        for (int i = 0; i < numActions; i++)
           finalActions[i] = _config.ACTION_STIFFNESS_HYPERPARAM * curActions[i] + (1 - _config.ACTION_STIFFNESS_HYPERPARAM) * prevActionOutput[i];
        prevActionOutput = finalActions;
        if (debug)
        {
            //Utils.debugArray(curActions, "curActions: ");
            StringBuilder debugStr = new StringBuilder();
            int actionIdx = 0;
            mm_v2.Bones[] fullDOFBonesToUse = _config.networkControlsAllJoints ? extendedfullDOFBones : fullDOFBones;
            for (int i = 0; i < fullDOFBonesToUse.Length; i++)
            {
                mm_v2.Bones bone = fullDOFBonesToUse[i];
                Vector3 output = new Vector3(curActions[actionIdx], curActions[actionIdx + 1], curActions[actionIdx + 2]);
                debugStr.Append($"{bone.ToString().Substring(5)}: {output} ");
                actionIdx += 3;
            }
            mm_v2.Bones[] limitedDOFBonesToUse = _config.networkControlsAllJoints ? extendedLimitedDOFBones : limitedDOFBones;
            for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
            {
                mm_v2.Bones bone = limitedDOFBonesToUse[i];
                debugStr.Append($"{bone}: {curActions[actionIdx]} | {finalActions[actionIdx]} ");
                actionIdx += 1;
            }
            //Debug.Log(debugStr.ToString());
        }
        applyActions(finalActions);
    }
    private void setFirstActionsAsIdentityRots(float[] actions)
    {
        mm_v2.Bones[] extendedBonesToUse = _config.networkControlsAllJoints ? extendedfullDOFBones : fullDOFBones;
        int actionIdx = 0;
        for(int i = 0; i < extendedBonesToUse.Length; i++)
        {
            Vector3 v1 = new Vector3(actions[actionIdx++], actions[actionIdx++], actions[actionIdx++]);
            Vector3 v2 = new Vector3(actions[actionIdx++], actions[actionIdx++], actions[actionIdx++]);
            initialRotInverses[i] = ArtBodyUtils.MatrixFrom6DRepresentation(v1, v2).transpose;
            //Debug.Log($"Initial geodesics for {i}: {ArtBodyUtils.geodesicBetweenTwoRotationMatrices(Matrix4x4.identity, initialRotInverses[i].transpose)}");
        }
    }

    private void applyActionsAsAxisAngleRotations(float[] finalActions, Quaternion[] curRotations, mm_v2.Bones[] fullDOFBonesToUse, ref int actionIdx)
    {

        for (int i = 0; i < fullDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)fullDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 output = new Vector3(finalActions[actionIdx], finalActions[actionIdx + 1], finalActions[actionIdx + 2]) * 120;
            actionIdx += 3;
            float angle = output.magnitude;
            // Angle is in range (0,3) => map to (-180, 180)
            //angle = (angle * 120) - 180;
            //Vector3 normalizedOutput = output.normalized;
            Quaternion offset = Quaternion.AngleAxis(angle, output);
            Quaternion final = offset * curRotations[boneIdx] ;
            ab.SetDriveRotation(final);
        }

    }

    private void applyActionsAsEulerRotations(float[] finalActions, Quaternion[] curRotations, mm_v2.Bones[] fullDOFBonesToUse, ref int actionIdx)
    {
        for (int i = 0; i < fullDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)fullDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 output = new Vector3(finalActions[actionIdx], finalActions[actionIdx + 1], finalActions[actionIdx + 2]);
            actionIdx += 3;
            Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
            float scale;

            var xdrive = ab.xDrive;
            scale = (xdrive.upperLimit - xdrive.lowerLimit) / 2f;
            float outputX = output.x * scale*2;
            if (_config.fullRangeEulerOutputs)
            {
                outputX = output.x * 180f;
            }
            xdrive.target = targetRotationInJointSpace.x + outputX;
            ab.xDrive = xdrive;

            var ydrive = ab.yDrive;
            scale = (ydrive.upperLimit - ydrive.lowerLimit) / 2f;
            float outputY = output.y * scale * 2;
            if (_config.fullRangeEulerOutputs)
            {
                outputY = output.y * 180f;
            }
            ydrive.target = targetRotationInJointSpace.y + outputY;
            ab.yDrive = ydrive;

            var zdrive = ab.zDrive;
            scale = (zdrive.upperLimit - zdrive.lowerLimit) / 2f;
            float outputZ = output.z * scale * 2;
            if (_config.fullRangeEulerOutputs)
            {
                outputZ = output.z * 180f;
            }
            zdrive.target = targetRotationInJointSpace.z + outputZ;
            ab.zDrive = zdrive;    
        }
    }
    private void applyActionsWith6DRotations(float[] finalActions, Quaternion[] curRotations, mm_v2.Bones[] fullDOFBonesToUse, ref int actionIdx)
    {
        for (int i = 0; i < fullDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)fullDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 outputV1 = new Vector3(finalActions[actionIdx], finalActions[actionIdx + 1], finalActions[actionIdx + 2]);
            Vector3 outputV2 = new Vector3(finalActions[actionIdx + 3], finalActions[actionIdx + 4], finalActions[actionIdx + 5]);
            actionIdx += 6;
            //Quaternion networkAdjustment = ArtBodyUtils.From6DRepresentation(outputV1, outputV2, ref initialRotInverses[i], _config.adjust6DRots);
            Matrix4x4 networkAdjustment = ArtBodyUtils.From6DRepresentation(outputV1, outputV2);
            Matrix4x4 rotationMatrix = Matrix4x4.TRS(Vector3.zero, curRotations[boneIdx].normalized, Vector3.one);
            Matrix4x4 finalRot = networkAdjustment * rotationMatrix;
            Quaternion newTargetRot =  Quaternion.LookRotation(finalRot.GetColumn(2), finalRot.GetColumn(1));
            ab.SetDriveRotation(newTargetRot);
            //ab.SetDriveRotation(newTargetRot.normalized);
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
    private Vector3 leftfootColliderCenter;
    private Vector3 rightfootColliderCenter;
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

        MMScript.search_time = _config.searchTime;
        if (!MMScript.is_initalized)
            return;

        kinChar = new CharInfo(nbodies, stateBones.Length);
        kinChar.trans = kinematicCharObj.transform;
        kinChar.boneToTransform = MMScript.boneToTransform;
        kinChar.charObj = kinematicCharObj;
        kinChar.MMScript = MMScript;
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
            body.solverIterations = _config.solverIterations;
            body.solverVelocityIterations = _config.solverIterations;
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
                if (i == (int)Bone_LeftFoot)
                    leftfootColliderCenter = simChar.boneToCollider[i].GetComponent<BoxCollider>().center;
                else if (i == (int)Bone_RightFoot)
                    rightfootColliderCenter = simChar.boneToCollider[i].GetComponent<BoxCollider>().center;
            } else { 
                kinChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(kinChar.boneToTransform[i].gameObject);
                simChar.boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(simChar.boneToTransform[i].gameObject);
            }
        }
        groundColliderY = groundCollider.bounds.max.y;
        toeColliderRadius = simChar.boneToCollider[(int)Bone_RightToe].GetComponent<CapsuleCollider>().radius;
        projectile = Instantiate(projectilePrefab, simulatedCharObj.transform.position + Vector3.up, Quaternion.identity);
        projectileCollider = projectile.GetComponent<Collider>();
        projectileRB = projectile.GetComponent<Rigidbody>();
        if (!_config.projectileTraining)
            projectile.SetActive(false);

        behaviorParameters = GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        numActions = behaviorParameters.BrainParameters.ActionSpec.NumContinuousActions;
        resetKinToSimOnFail &= behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
        //if (behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly)
        //    Debug.Log($"{behaviorParameters.BrainParameters.Norm}");
        //if (_config.networkControlsAllJoints)
        //    numActions = (extendedfullDOFBones.Length * (_config.actionsAre6DRotations ? 6 : 3)) + extendedLimitedDOFBones.Length; // 40 or 76
        //else 
        //    numActions = (fullDOFBones.Length * (_config.actionsAre6DRotations ? 6 : 3)) + limitedDOFBones.Length; // 25 or 44
        //Debug.Log($"numActions: {numActions}");
        numObservations = 88 + numActions; // 111 or 132 or 128 or 164
        isInitialized = true;
        if (_config.actionsAre6DRotations)
        {
            initialRotInverses = new Matrix4x4[_config.networkControlsAllJoints ? extendedfullDOFBones.Length : fullDOFBones.Length];
        }
        if (resetKinToSimOnFail)
        {
            foreach(var col in simChar.trans.GetComponentsInChildren<ArticulationBody>())
            {
                var cr = col.gameObject.AddComponent<CollisionReporter>();
                cr.director = this;
            }
        }
        curFixedUpdate = _config.EVALUATE_EVERY_K_STEPS - 1;
        resetData();
    }

    Quaternion[] prevKinRots = Utils.identity_quat_arr(23);
    private void resetData()
    {
        for (int i = 0; i < nbodies; i++)
        {
            kinChar.boneSurfacePts[i] = new Vector3[6];
            kinChar.boneSurfacePtsWorldSpace[i] = new Vector3[6];
            kinChar.boneSurfaceVels[i] = new Vector3[6]; 
            simChar.boneSurfacePts[i] = new Vector3[6];
            simChar.boneSurfacePtsWorldSpace[i] = new Vector3[6];
            simChar.boneSurfaceVels[i] = new Vector3[6];
        }
        prevActionOutput = new float[numActions];
        kinChar.boneState = new float[36];
        simChar.boneState = new float[36];
        //if (_config.setDriveTargetVelocities)
        //    prevKinRots = MMScript.bone_rotations;
        UpdateKinCMData(false);
        UpdateSimCMData(false);
    }
    Matrix4x4[] initialRotInverses;


    public void Awake()
    {
        //Debug.Log("MLAgents Director Awake called");
        //Unity.MLAgents.Policies.BehaviorParameters behavior_params = gameObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
        if (motionDB == null)
            motionDB = database.Instance;
        _config = ConfigWriter.Instance;
        nbodies = motionDB.nbones();
        kinematicCharObj = Instantiate(_config.useCapsuleFeet ? kinematic_char_type2_prefab : kinematic_handmade_char_prefab  , Vector3.zero, Quaternion.identity);
        simulatedCharObj = Instantiate(_config.useCapsuleFeet ? simulated_char_type2_prefab : simulated_handmade_char_prefab  , Vector3.zero, Quaternion.identity);
        debug = debug && !Academy.Instance.IsCommunicatorOn;
        if (Academy.Instance.IsCommunicatorOn)
        {
            int numFixedUpdatesPerSecond = Mathf.CeilToInt(1f / Time.fixedDeltaTime);
            MaxStep = numFixedUpdatesPerSecond * _config.MAX_EPISODE_LENGTH_SECONDS;
        }
        customInit();
        //SimCharController.set_art_body_rot_limits();
    }
    public override void OnEpisodeBegin()
    {
        //Debug.Log("OnEpisodeBegin() called");
        if (_config.resetKinCharOnEpisodeEnd)
        {
            MMScript.Reset();
            MMScript.FixedUpdate();
        }
        float verticalOffset = getVerticalOffset();
        //Debug.Log($"Vertical offset: {verticalOffset}");
        projectile.transform.position = kinChar.boneToTransform[(int)Bone_Entity].position + 2 * Vector3.right;
        SimCharController.teleportSimChar(simChar, kinChar, verticalOffset + .02f, !_config.resetKinCharOnEpisodeEnd && updateVelOnTeleport);
        lastSimCharTeleportFixedUpdate = curFixedUpdate;
        teleportSinceLastGetState = true;
        Physics.Simulate(.00001f);
        resetData();
        kinChar.cmVel = Vector3.zero;
        simChar.cmVel = Vector3.zero;
        //RequestDecision();
        //Debug.Log($"Teleoport happens on {curFixedUpdate}");
    }

    bool updateVelocity;
    private void FixedUpdate()
    {
        if (!isInitialized)
        {
            customInit();
            return;
        }
        curFixedUpdate++;
        //bool endedEpisodeAtEndOfLastFrame = lastEpisodeEndingFrame == (curFixedUpdate - 1);
        //if (endedEpisodeAtEndOfLastFrame)
        //    Debug.Log($"endedEpisodeAtEndOfLastFrame, curFixedUpdate: {curFixedUpdate}");
        //Debug.Log($"{Time.frameCount} : ML Agent updated");

        // Make sure to teleport sim character if kin character teleported
        //Debug.Log($"simCharTeleported: {simCharTeleported}");
        //if (simCharTeleported)
        //    Debug.Log($"BOOM : {simCharTeleported}");
        // only update velocity if we did not teleport last frame
        updateVelocity = lastSimCharTeleportFixedUpdate + 1 < curFixedUpdate;
        //Debug.Log($"lastKinRootPos: {lastKinRootPos} simChar.root.transform.position: {simChar.root.transform.position} ");
        if (MMScript.teleportedThisFixedUpdate)
        {
            Vector3 preTeleportSimCharPosOffset = lastKinRootPos - simChar.root.transform.position;
            //Debug.Log($"preTeleportSimCharPosOffset: {preTeleportSimCharPosOffset} lastKinRootPos: {lastKinRootPos} simChar.root.transform.position: {simChar.root.transform.position} ");
            //simChar.root.TeleportRoot(newRootPosition, kinChar.trans.rotation);
            SimCharController.teleportSimCharRoot(simChar, MMScript.origin, preTeleportSimCharPosOffset);
            //Debug.Log($"{Time.frameCount}: Teleporting sim char root");
            teleportSinceLastGetState = true;

            updateVelocity = false;
        }
        // Update CMs 
        //Debug.Log($"curFixedUpdate: {curFixedUpdate} updateVelocity: {updateVelocity}");
        UpdateKinCMData(updateVelocity);
        //Debug.Log($"{Time.frameCount}: FixedUpdate kin cm: {kinChar.cm} kin cm vel: {kinChar.cmVel} sim cm: {simChar.cm} sim cm vel: {simChar.cmVel} ");
        UpdateBoneObsState(updateVelocity, Time.fixedDeltaTime);
        // UpdateBoneSurfacePts(updateVelocity);
        //bool episodeEnded = calcAndSetRewards();
        //if (episodeEnded)
        //    return;
        // request Decision
        //if (!updateVelocity)
        //    return;
        ClearGizmos();
        AddGizmoSphere(kinChar.cm, Color.blue);
        AddGizmoSphere(simChar.cm, Color.red);
        if (curFixedUpdate % _config.EVALUATE_EVERY_K_STEPS == 0)
            RequestDecision();
        else {
            applyActions(prevActionOutput);
        }
        //updateMeanReward();
        if (_config.projectileTraining)
            FireProjectile();
        lastKinRootPos = kinChar.trans.position;
    }

    private void FireProjectile()
    {
        if (Time.time - lastProjectileLaunchtime < _config.LAUNCH_FREQUENCY)
            return;
        lastProjectileLaunchtime = Time.time;
        projectileRB.mass = UnityEngine.Random.Range(.01f, 8f);
        // To get XZ position, consider characters position on plane, pick random spot on unit cirlce,
        // and go RADIUS units towards that spot 
        // Y position is uniformly sampled from −0.5 m to 0.5 m, centered on the character’s CM
        float YTarget = UnityEngine.Random.Range(simChar.cm.y - .5f, simChar.cm.y + .5f);
        Vector2 randomUnitCircle = UnityEngine.Random.insideUnitCircle.normalized * _config.LAUNCH_RADIUS;
        Vector3 finalPosition = new Vector3(simChar.cm.x + randomUnitCircle.x, simChar.cm.y, simChar.cm.z + randomUnitCircle.y);
        projectile.transform.position = finalPosition;
        // We want the projectile to be at YTarget when it reaches the target
        // We know its acceleration (-9.8), its final position (YTarget), its starting Y position (simChar.cm.y), we need its
        // current velocity 
        float timeToTravel = _config.LAUNCH_RADIUS / _config.LAUNCH_SPEED; // So if it's traveling 1m at 5m/s, its timeToTravel is .2 seconds
        float changeInY = YTarget - simChar.cm.y;
        float totalAcceleration = timeToTravel * -9.8f; // this will be negative ofc
        float curYVelocity = -totalAcceleration + (changeInY / timeToTravel);
        projectileRB.AddForce(-randomUnitCircle.x * _config.LAUNCH_SPEED, curYVelocity, -randomUnitCircle.y * _config.LAUNCH_SPEED, ForceMode.VelocityChange);
    }

    private void UpdateKinCMData(bool updateVelocity)
    {
        //Debug.Log($"Updating CM data, update vel: {updateVelocity}");
        Vector3 newKinCM = getCM(kinChar.boneToTransform);
        kinChar.cmVel = updateVelocity ? (newKinCM - kinChar.cm) / Time.fixedDeltaTime : kinChar.cmVel;
        kinChar.cm = newKinCM;
        
    }
    private void UpdateSimCMData(bool updateVelocity)
    {
        //Debug.Log($"Updating CM data, update vel: {updateVelocity}");
        Vector3 newSimCM = getCM(simChar.boneToTransform);
        simChar.cmVel = updateVelocity ? (newSimCM - simChar.cm) / Time.fixedDeltaTime : simChar.cmVel;
        simChar.cm = newSimCM;
    }

    private void UpdateBoneSurfacePts(bool updateVelocity)
    {
        for (int i = 1; i < 23; i++)
        {

            Vector3[] newKinBoneSurfacePts = new Vector3[6];
            Vector3[] newSimBoneSurfacePts = new Vector3[6];
            Vector3[] newKinBoneSurfacePtsWorld = new Vector3[6];
            Vector3[] newSimBoneSurfacePtsWorld = new Vector3[6];

            getSixPointsOnCollider(kinChar.boneToCollider[i], ref newKinBoneSurfacePts, (mm_v2.Bones) i);
            getSixPointsOnCollider(simChar.boneToCollider[i], ref newSimBoneSurfacePts, (mm_v2.Bones) i);

            //Vector3[] prevKinSurfacePts = kinChar.boneSurfacePts[i];
            //Vector3[] prevSimSurfacePts = simChar.boneSurfacePts[i];
            Vector3[] prevKinSurfacePts = kinChar.boneSurfacePtsWorldSpace[i];
            Vector3[] prevSimSurfacePts = simChar.boneSurfacePtsWorldSpace[i];

            for (int j = 0; j < 6; j++)
            {
                newKinBoneSurfacePtsWorld[j] = newKinBoneSurfacePts[j];
                newSimBoneSurfacePtsWorld[j] = newSimBoneSurfacePts[j];

                newKinBoneSurfacePts[j] = resolvePosInKinematicRefFrame(newKinBoneSurfacePts[j]);
                newSimBoneSurfacePts[j] = resolvePosInSimRefFrame(newSimBoneSurfacePts[j]);

                if (updateVelocity) {
                    kinChar.boneSurfaceVels[i][j] = (newKinBoneSurfacePtsWorld[j] - prevKinSurfacePts[j]) / Time.fixedDeltaTime;
                    simChar.boneSurfaceVels[i][j] = (newSimBoneSurfacePtsWorld[j] - prevSimSurfacePts[j]) / Time.fixedDeltaTime;
                    //Debug.Log($"simChar.boneSurfaceVels[i][j] : {simChar.boneSurfaceVels[i][j]}");
                }
            }

            kinChar.boneSurfacePtsWorldSpace[i] = newKinBoneSurfacePtsWorld;
            simChar.boneSurfacePtsWorldSpace[i] = newSimBoneSurfacePtsWorld;

            kinChar.boneSurfacePts[i] = newKinBoneSurfacePts;
            simChar.boneSurfacePts[i] = newSimBoneSurfacePts;
        }
    }

    private void UpdateBoneObsState(bool updateVelocity, float dt)
    {
        foreach (bool isKinChar in new bool[]{true, false}) {
            CharInfo curInfo = isKinChar ? kinChar : simChar;
            float[] copyInto = curInfo.boneState;
            int copyIdx = 0;
            for (int j = 0; j < stateBones.Length; j++)
            {
                mm_v2.Bones bone = stateBones[j];
                // Compute position of bone
                Vector3 boneWorldPos = curInfo.boneToTransform[(int)bone].position;
                Vector3 boneLocalPos = isKinChar ? resolvePosInKinematicRefFrame(boneWorldPos) : resolvePosInSimRefFrame(boneWorldPos);
                //Vector3 bone_relative_pos = Utils.quat_inv_mul_vec3(relative_rot, bone_local_pos);
                Vector3 prevBonePos = curInfo.boneWorldPos[j];
                Vector3 boneVel = (boneWorldPos - prevBonePos) / dt;
                boneVel = resolveVelInKinematicRefFrame(boneVel);
                copyVecIntoArray(ref copyInto, ref copyIdx, boneLocalPos);
                //copyVecIntoArray(ref copyInto, ref copyIdx, updateVelocity ? boneVel : Vector3.zero);
                if (updateVelocity)
                    copyVecIntoArray(ref copyInto, ref copyIdx, boneVel);
                else
                    copyIdx += 3;
                curInfo.boneWorldPos[j] = boneWorldPos;
                //if (debug)
                //    Debug.Log($"{(isKinChar ? "Kin: " : "Sim: ")} {bone} world pos: {boneWorldPos} | local pos: {boneLocalPos} : vel: {(updateVelocity ? boneVel : Vector3.zero)}");
            }
        }
    }

    public void AssignLayer(int layer, int projectileLayer)
    {
        simulatedCharObj.layer = layer;
        foreach (var child in simulatedCharObj.GetComponentsInChildren<Transform>())
            child.gameObject.layer = layer;

        projectile.layer = projectileLayer;
    }


    private float meanReward = 0f;
    internal float finalReward = 0f;
    internal int lastEpisodeEndingFrame = 0;
    private bool shouldEndThisFrame = false;
    // returns TRUE if episode ended
    public bool calcAndSetRewards()
    {

        bool heads1mApart;
        double posReward, velReward, localPoseReward, cmVelReward, fallFactor;
        calcFallFactor(out fallFactor, out heads1mApart);
        if (heads1mApart && resetKinToSimOnFail && !shouldEndThisFrame)
        {
            MMScript.ResetAndTeleport(simChar.trans.position, simChar.trans.rotation);
            MMScript.FixedUpdate();
            UpdateKinCMData(false);
            UpdateBoneObsState(false, Time.fixedDeltaTime);
        }
        else if ((heads1mApart && curFixedUpdate > lastSimCharTeleportFixedUpdate + 1) || shouldEndThisFrame)
        {
            finalReward = _config.EPISODE_END_REWARD;
            //updateMeanReward(-.5f);
            SetReward(_config.EPISODE_END_REWARD);
            //Debug.Log("=================================================");
            Debug.Log($"{Time.frameCount}: Calling end epsidoe on: {curFixedUpdate}, lasted {curFixedUpdate - lastEpisodeEndingFrame} frames ({(curFixedUpdate - lastEpisodeEndingFrame)/60f} sec)");
            //Debug.Log("=================================================");
            lastEpisodeEndingFrame = curFixedUpdate;
            shouldEndThisFrame = false;
            EndEpisode();
#if UNITY_EDITOR
            //if (debug)
            //    EditorApplication.isPaused = true;
#endif

            return true;
        }
#if UNITY_EDITOR
        //if (debug && fallFactor < 0.02)
        //    EditorApplication.isPaused = true;
#endif
        UpdateSimCMData(updateVelocity);
        //Debug.Log($"{Time.frameCount}: Rewards kin cm: {kinChar.cm} kin cm vel: {kinChar.cmVel} sim cm: {simChar.cm} sim cm vel: {simChar.cmVel} ");
        UpdateBoneSurfacePts(updateVelocity);
        // Calc them even if we don't use them for normalizer sake
        calcPosAndVelReward(out posReward, out velReward);
        calcLocalPoseReward(out localPoseReward);
        calcCMVelReward(out cmVelReward);
        if (curFixedUpdate - _config.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT < lastSimCharTeleportFixedUpdate)
        {
            finalReward = 0f;
        }
        else
        {
            finalReward = (float)(fallFactor * (posReward + velReward + localPoseReward + cmVelReward));
        }
        //Debug.Log($"finalReward: {finalReward} fall_factor: {fallFactor}, pos_reward: {posReward}, vel_reward: {velReward}, local_pose_reward: {localPoseReward}, cm_vel_reward: {cmVelReward}");
        //updateMeanReward(final_reward);
        AddReward(finalReward);
        return false;
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
    0-2   : cm distance
    3-5   : kinematic cm vel
    6-8   : sim cm vel
    9-11   : difference between sim cm vel and kin cm vel
    12-13  : desired velocity
    14-15 : the diff between sim cm horizontal vel and desired vel
    16-51 : kin char state
    52-87 : sim state - kin state
    88-110 : numActions (
    */
    Vector3 kinCMVelLastGetState, simCMVelLastGetState;
    Vector3 kinCMLastGetState, simCMLastGetState;
    bool teleportSinceLastGetState = true;
    float[] getState()
    {
        //bool updateVel = curFixedUpdate - lastSimCharTeleportFixedUpdate < _config.EVALUATE_EVERY_K_STEPS;
        //float decisionPeriod = Time.fixedDeltaTime * _config.EVALUATE_EVERY_K_STEPS;
        //kinCMVelLastGetState = teleportSinceLastGetState ? kinCMVelLastGetState : ((kinChar.cm - kinCMLastGetState) / decisionPeriod);
        //simCMVelLastGetState = teleportSinceLastGetState ? simCMVelLastGetState : ((simChar.cm - simCMLastGetState) / decisionPeriod);
        //UpdateBoneObsState(!teleportSinceLastGetState, decisionPeriod);
        //ClearGizmos();

        Vector3 cmDistance = kinChar.cm - simChar.cm;
        //bool endedEpisode = lastEpisodeEndingFrame >= (curFixedUpdate - 1);
        //Debug.Log($"{Time.frameCount}: getState kinChar.cmVel {kinChar.cmVel} simChar.cmVel {simChar.cmVel} kinCMVelLastGetState: {kinCMVelLastGetState} simCMVelLastGetState: {simCMVelLastGetState}");
        Vector3 kinCMVelInKinRefFrame = resolveVelInKinematicRefFrame(kinChar.cmVel);        
        Vector3 simCMVelInKinRefFrame = resolveVelInKinematicRefFrame(simChar.cmVel);
        Vector3 desiredVel = resolveVelInKinematicRefFrame(MMScript.desired_velocity);
        Vector3 velDiffSimMinusDesired = simCMVelInKinRefFrame - desiredVel;

        float[] state = new float[numObservations];
        int state_idx = 0;
        copyVecIntoArray(ref state, ref state_idx, cmDistance);
        copyVecIntoArray(ref state, ref state_idx, kinCMVelInKinRefFrame);
        copyVecIntoArray(ref state, ref state_idx, simCMVelInKinRefFrame);
        copyVecIntoArray(ref state, ref state_idx, simCMVelInKinRefFrame - kinCMVelInKinRefFrame);
        copyVecIntoArray(ref state, ref state_idx, new Vector2(desiredVel.x, desiredVel.z));
        copyVecIntoArray(ref state, ref state_idx, new Vector2(velDiffSimMinusDesired.x, velDiffSimMinusDesired.z));
        //if (debug)
        //    Debug.Log($"Desired Velocity: {new Vector2(desiredVel.x, desiredVel.z)} kinCMVelInKinRefFrame: {kinCMVelInKinRefFrame} simCMVelInKinRefFrame: {simCMVelInKinRefFrame} velDiffSimMinusDesired: {new Vector2(velDiffSimMinusDesired.x, velDiffSimMinusDesired.z)}");
        

        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then (s(sim) - s(kin))
        for (int i = 0; i < 36; i++)
            state[state_idx++] = simChar.boneState[i];
        for (int i = 0; i < 36; i++)
            state[state_idx++] = simChar.boneState[i] - kinChar.boneState[i];
        for (int i = 0; i < numActions; i++)
            state[state_idx++] = prevActionOutput[i];
   
        if (state_idx != numObservations)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything, 6D: {_config.actionsAre6DRotations}");

        //kinCMLastGetState = kinChar.cm;
        //simCMLastGetState = simChar.cm;
        //teleportSinceLastGetState = false;
        //if (debug)
        //    Utils.debugArray(state, $"{curFixedUpdate} state: ");
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
            var ab = t.GetComponent<ArticulationBody>();
            float mass = t.GetComponent<ArticulationBody>().mass;
            Vector3 child_center = global_bone_positions == null ? getChildColliderCenter(t.gameObject) : global_bone_positions[i];
            CoM += mass * child_center;
            total_mass += ab.mass;

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
        if (debug)
            Debug.Log($"velDiffsSum: {velDiffsSum} velReward: {velReward}");
        if (_config.normalizeRewardComponents)
        {
            posReward = posRewardNormalizer.getNormalized((float)posReward);
            velReward = velRewardNormalizer.getNormalized((float)velReward);
        }
    }

    void calcLocalPoseReward(out double poseReward)
    {
        double totalLoss = 0;
        for (int i = 0; i < 23; i++)
        {
            Transform kinBone = kinChar.boneToTransform[i];
            Transform simBone = simChar.boneToTransform[i];
            float loss;
            // After some testing, the results from either of these should be identical

            if (_config.useGeodesicForAngleDiff)
            {
                Matrix4x4 kinRotation = Matrix4x4.Rotate(kinBone.localRotation);
                Matrix4x4 simRotation = Matrix4x4.Rotate(simBone.localRotation);
                //Matrix4x4 lossMat = (simRotation * kinRotation.transpose);
                //float trace = lossMat[0, 0] + lossMat[1, 1] + lossMat[2, 2];
                // Need clamping because Acos will throw NAN for values outside [-1, 1]
                //float traceClamped = Mathf.Clamp((trace - 1) / 2, -1f, 1f);
                //loss = Mathf.Acos(traceClamped);
                loss = ArtBodyUtils.geodesicBetweenTwoRotationMatrices(kinRotation, simRotation);
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
        poseReward = Math.Exp((-10f/nbodies) * _config.poseRewardMultiplier * totalLoss);
        if (_config.normalizeRewardComponents)
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
        if (_config.normalizeRewardComponents)
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

    public void processCollision(Collision collision)
    {
        //Debug.Log($"collision");
        if (!resetKinToSimOnFail)
            return;
        foreach (ContactPoint contact in collision.contacts)
        {
            string colliderName = contact.thisCollider.gameObject.name;
            if (!colliderName.ToLower().Contains("toe") && !colliderName.ToLower().Contains("foot") && contact.otherCollider.gameObject.name == "Ground")
            {
                shouldEndThisFrame = true;
                //EndEpisode();
                //Debug.Log($"{Time.frameCount}: Calling end epsidoe on: {curFixedUpdate}, lasted {curFixedUpdate - lastEpisodeEndingFrame} frames ({(curFixedUpdate - lastEpisodeEndingFrame) / 60f} sec)");
                //lastEpisodeEndingFrame = curFixedUpdate;
            }
            //Debug.Log($"{colliderName} collided with {contact.otherCollider.gameObject.name}");
        }
    }



    // ================================ UTIL ====================================
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

    private float getBottomMostPointOnFoot(Transform foot, Vector3 center)
    {
        float x = feetBozSize.x / 2;
        float y = feetBozSize.y / 2;
        float z = feetBozSize.z / 2;
        Vector3 topLeft = foot.TransformPoint(center + new Vector3(x, -y, z));
        Vector3 topRight = foot.TransformPoint(center + new Vector3(x, -y, -z));
        Vector3 bottomLeft = foot.TransformPoint(center + new Vector3(-x, -y, z));
        Vector3 bottomRight = foot.TransformPoint(center + new Vector3(-x, -y, -z));
        //AddGizmoSphere(topLeft, Color.red);
        //AddGizmoSphere(topRight, Color.red);
        //AddGizmoSphere(bottomLeft, Color.red);
        //AddGizmoSphere(bottomRight, Color.red);

        return Mathf.Min(topLeft.y, topRight.y, bottomLeft.y, bottomRight.y);
    }

    private float getVerticalOffset()
    {
        ClearGizmos();
        Transform leftFoot = kinChar.boneToCollider[(int)Bone_LeftFoot].transform;
        Transform rightFoot = kinChar.boneToCollider[(int)Bone_RightFoot].transform;
        float minPointOnFoot = Mathf.Min(getBottomMostPointOnFoot(leftFoot, leftfootColliderCenter), getBottomMostPointOnFoot(rightFoot, rightfootColliderCenter));
        Transform leftToe = kinChar.boneToTransform[(int)Bone_LeftToe];
        Transform rightToe = kinChar.boneToTransform[(int)Bone_RightToe];
        float minToeY = Mathf.Min(leftToe.position.y, rightToe.position.y) - toeColliderRadius;
        float maxGroundPenetration = Mathf.Max(0f, groundColliderY - Mathf.Min(minPointOnFoot , minToeY));
        //Debug.Log($"minToeY: {minToeY} minPointOnFoot: {minPointOnFoot} maxGroundPenetration: {maxGroundPenetration}");
        return maxGroundPenetration;
    }
    public void getSixPointsOnCollider(GameObject obj, ref Vector3[] outputs, mm_v2.Bones bone)
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

    private void OnGUI()
    {
        if (!_config.rewardsInGUI)
            return;
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 16;
        GUI.Label(new Rect(100, 175, 600, 100), "Last Reward: " + finalReward.ToString(), fontSize);
    }
}
