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

    float[] prevActionOutput;
    int numActions;
    int numObservations;
    database motionDB;
    public bool kinUseDebugMats = false;
    public bool simUseDebugMats = false;

    // Used for normalization
    private Vector3 lastKinRootPos = Vector3.zero;

    [HideInInspector]
    public static mm_v2.Bones[] stateBones = new mm_v2.Bones[]
    {  Bone_LeftToe, Bone_RightToe, Bone_Spine, Bone_Head, Bone_LeftForeArm, Bone_RightForeArm };
    [HideInInspector]
    public static mm_v2.Bones[] fullDOFBones = new mm_v2.Bones[]
    {  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Hips};

    //public static mm_v2.Bones[] fullDOFBones = new mm_v2.Bones[]
    //{  Bone_LeftUpLeg, Bone_RightUpLeg, Bone_LeftFoot, Bone_RightFoot, Bone_LeftArm, Bone_RightArm, Bone_Spine};

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
    public bool debug = false;
    public bool updateVelOnTeleport = true;
    public bool resetKinToSimOnFail = false;
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
        switch (_config.actionRotType)
        {
            case ActionRotationType.AxisAngle:
                applyActionsAsAxisAngleRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
                break;
            case ActionRotationType.Euler:
                applyActionsAsEulerRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
                break;
            case ActionRotationType.SixD:
                applyActionsWith6DRotations(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
                break; 
            case ActionRotationType.Exp:
                applyActionsAsExp(finalActions, curRotations, fullDOFBonesToUse, ref actionIdx);
                break;
        }

        mm_v2.Bones[] limitedDOFBonesToUse = _config.networkControlsAllJoints ? extendedLimitedDOFBones : limitedDOFBones;

        for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)limitedDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            float output = finalActions[actionIdx];
            actionIdx++;
            var zDrive = ab.zDrive;
            float target;
            float range = zDrive.upperLimit - zDrive.lowerLimit;
            if (_config.setRotsDirectly)
            {
                var midpoint = zDrive.lowerLimit + (range/2);
                target = (output * (range/2)) + midpoint;
            }
            else
            {
                float angle = output * range;
                Vector3 targetRotationInJointSpace = ab.ToTargetRotationInReducedSpace(curRotations[boneIdx], true);
                target = targetRotationInJointSpace.z + angle;
            }
            zDrive.target = target;
            ab.zDrive = zDrive;
        }
        mm_v2.Bones[] openloopBonesBonesToUse = _config.networkControlsAllJoints ? alwaysOpenloopBones : openloopBones;
        for (int i = 0; i < openloopBonesBonesToUse.Length; i++)
        {
            int boneIdx = (int)openloopBonesBonesToUse[i];
            Quaternion final = curRotations[boneIdx];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            ab.SetDriveRotation(final);
        }
        if (_config.setDriveTargetVelocities)
            for (int i = 1; i < 23; i++)
                simChar.boneToArtBody[i].SetDriveTargetVelocity(MMScript.bone_angular_velocities[i], curRotations[i]);

    }
    // 7 joints with 3 DOF with outputs as scaled angle axis = 21 outputs
    // plus 4 joints with 1 DOF with outputs as scalars = 25 total outputs
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        bool actionsAre6D = _config.actionRotType == ActionRotationType.SixD;
        float[] curActions = actionBuffers.ContinuousActions.Array;
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
                debugStr.Append($"{bone.ToString().Substring(5)}: {output} " + (actionsAre6D ? $"{ new Vector3(curActions[actionIdx + 3], curActions[actionIdx + 4], curActions[actionIdx + 5])} " : ""));
                actionIdx += actionsAre6D ? 6 : 3;
            }
            mm_v2.Bones[] limitedDOFBonesToUse = _config.networkControlsAllJoints ? extendedLimitedDOFBones : limitedDOFBones;
            for (int i = 0; i < limitedDOFBonesToUse.Length; i++)
            {
                mm_v2.Bones bone = limitedDOFBonesToUse[i];
                debugStr.Append($"{bone}: {curActions[actionIdx]} | {finalActions[actionIdx]} ");
                actionIdx += 1;
            }
            Debug.Log(debugStr.ToString());
        }
        applyActions(finalActions);
    }
    private void applyActionsAsExp(float[] finalActions, Quaternion[] curRotations, mm_v2.Bones[] fullDOFBonesToUse, ref int actionIdx)
    {

        for (int i = 0; i < fullDOFBonesToUse.Length; i++)
        {
            int boneIdx = (int)fullDOFBonesToUse[i];
            ArticulationBody ab = simChar.boneToArtBody[boneIdx];
            Vector3 output = new Vector3(finalActions[actionIdx], finalActions[actionIdx + 1], finalActions[actionIdx + 2]);
            actionIdx += 3;
            Quaternion offset = MathUtils.quat_exp(output * _config.alphaForExpMap / 2f);
            Quaternion final = _config.setRotsDirectly ? offset : _config.outputIsBase ? curRotations[boneIdx] * offset : offset * curRotations[boneIdx];
            ab.SetDriveRotation(final);
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
            Quaternion final = _config.setRotsDirectly ? offset : _config.outputIsBase ? curRotations[boneIdx] * offset : offset * curRotations[boneIdx];
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
            float scale, midpoint;

            var xdrive = ab.xDrive;
            scale = (xdrive.upperLimit - xdrive.lowerLimit) / 2f;
            midpoint = xdrive.lowerLimit + scale;
            float outputX = _config.setRotsDirectly ? (output.x * scale) + midpoint : output.x * scale * 2;
            if (_config.fullRangeEulerOutputs)
            {
                outputX = output.x * 180f;
            }
            xdrive.target = _config.setRotsDirectly ? outputX : targetRotationInJointSpace.x + outputX;
            ab.xDrive = xdrive;

            var ydrive = ab.yDrive;
            scale = (ydrive.upperLimit - ydrive.lowerLimit) / 2f;
            midpoint = ydrive.lowerLimit + scale;
            float outputY = _config.setRotsDirectly ? (output.y * scale) + midpoint : output.y * scale * 2;
            if (_config.fullRangeEulerOutputs)
            {
                outputY = output.y * 180f;
            }
            ydrive.target = _config.setRotsDirectly ? outputY : targetRotationInJointSpace.y + outputY;
            ab.yDrive = ydrive;

            var zdrive = ab.zDrive;
            scale = (zdrive.upperLimit - zdrive.lowerLimit) / 2f;
            midpoint = zdrive.lowerLimit + scale;
            float outputZ = _config.setRotsDirectly ? (output.z * scale) + midpoint : output.z * scale * 2;
            if (_config.fullRangeEulerOutputs)
            {
                outputZ = output.z * 180f;
            }
            zdrive.target = _config.setRotsDirectly ? outputZ : targetRotationInJointSpace.z + outputZ;
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
            Quaternion newTargetRot;
            //Quaternion networkAdjustment = ArtBodyUtils.From6DRepresentation(outputV1, outputV2, ref initialRotInverses[i], _config.adjust6DRots);
            if (_config.sixDRotMethod == SixDRotationMethod.TRSTimesMatrix)
            {
                Matrix4x4 networkAdjustment = ArtBodyUtils.MatrixFrom6DRepresentation(outputV1, outputV2);
                Matrix4x4 rotationMatrix = Matrix4x4.TRS(Vector3.zero, curRotations[boneIdx].normalized, Vector3.one);
                Matrix4x4 finalRot = networkAdjustment * rotationMatrix;
                newTargetRot = Quaternion.LookRotation(finalRot.GetColumn(2), finalRot.GetColumn(1));
            }
            else if (_config.sixDRotMethod == SixDRotationMethod.RotateObjectWithOrthonormalVector)
            {
                Quaternion offset = ArtBodyUtils.RotateObjectWithOrthonormalVector(outputV1, outputV2);
                newTargetRot = _config.setRotsDirectly ? offset : offset * curRotations[boneIdx];
            }
            else if (_config.sixDRotMethod == SixDRotationMethod.MatrixToQuat)
            {
                Quaternion offset = ArtBodyUtils.QuatFrom6DRepresentation(outputV1, outputV2);
                newTargetRot = _config.setRotsDirectly ? offset : offset * curRotations[boneIdx];
            }
            else
                newTargetRot = Quaternion.identity;

            ab.SetDriveRotation(newTargetRot);
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
        if (kinUseDebugMats || simUseDebugMats)
        {
            Material RedMatTransparent, WhiteMatTransparent;
#if UNITY_EDITOR
            RedMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/RedMatTransparent.mat", typeof(Material));
            WhiteMatTransparent = (Material)AssetDatabase.LoadAssetAtPath("Assets/Resources/WhiteMatTransparent.mat", typeof(Material));
#else
            RedMatTransparent = Resources.Load<Material>("RedMatTransparent");
            WhiteMatTransparent = Resources.Load<Material>("WhiteMatTransparent");
#endif
            if (kinUseDebugMats)
                kinematicCharObj.GetComponent<ArtBodyTester>().set_all_material(WhiteMatTransparent);
            if (simUseDebugMats && !_config.useSkinnedMesh)
                simulatedCharObj.GetComponent<ArtBodyTester>().set_all_material(RedMatTransparent);
        }
        if (_config.useSkinnedMesh)
        {
            foreach (var rend in simulatedCharObj.GetComponentsInChildren<Renderer>())
                rend.enabled = false;
            var skin = simulatedCharObj.GetComponentInChildren<SkinnedMeshRenderer>(true);
            if (skin != null)
            {
                skin.gameObject.SetActive(true);
                skin.enabled = true;
            }
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
        bool isInference = behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
        _config.clampKinCharToSim &= isInference;
        if (!isInference)
            _config.friction = 1f;
        numObservations = behaviorParameters.BrainParameters.VectorObservationSize; // 111 or 132 or 128 or 164
        if (_config.clampKinCharToSim) 
            foreach(var col in simChar.trans.GetComponentsInChildren<ArticulationBody>()) { 
                col.gameObject.AddComponent<CollisionReporter>().director = this;
                col.collisionDetectionMode = CollisionDetectionMode.Continuous;
            }

        curFixedUpdate = _config.EVALUATE_EVERY_K_STEPS - 1;
        resetData();
    }

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
        UpdateBoneObsState(false, Time.fixedDeltaTime, true);
        UpdateBoneSurfacePts(false);
    }

    public void Awake()
    {
        if (motionDB == null)
            motionDB = database.Instance;
        _config = ConfigWriter.Instance;
        nbodies = motionDB.nbones();
        kinematicCharObj = Instantiate(_config.useCapsuleFeet ? kinematic_char_type2_prefab : kinematic_handmade_char_prefab  , Vector3.zero, Quaternion.identity);
        simulatedCharObj = Instantiate(_config.useCapsuleFeet ? simulated_char_type2_prefab : simulated_handmade_char_prefab  , Vector3.zero, Quaternion.identity);
        debug = debug && !Academy.Instance.IsCommunicatorOn;
        if (Academy.Instance.IsCommunicatorOn)
        {
            int numStepsPerSecond = (int)(Mathf.Ceil(1f / Time.fixedDeltaTime) / _config.EVALUATE_EVERY_K_STEPS);
            MaxStep = numStepsPerSecond * _config.MAX_EPISODE_LENGTH_SECONDS;
        }
        customInit();
    }
    public override void OnEpisodeBegin()
    {
        if (_config.resetKinCharOnEpisodeEnd)
        {
            MMScript.Reset();
            MMScript.FixedUpdate();
        }
        float verticalOffset = getVerticalOffset();
        projectile.transform.position = kinChar.boneToTransform[(int)Bone_Entity].position + 2 * Vector3.right;
        SimCharController.teleportSimChar(simChar, kinChar, verticalOffset + .02f, !_config.resetKinCharOnEpisodeEnd && updateVelOnTeleport);
        lastSimCharTeleportFixedUpdate = curFixedUpdate;
        Physics.Simulate(.00001f);
        resetData();
        kinChar.cmVel = Vector3.zero;
        simChar.cmVel = Vector3.zero;
    }

    bool updateVelocity;
    private void FixedUpdate()
    {
        curFixedUpdate++;
        if (MMScript.teleportedThisFixedUpdate)
        {
            Vector3 preTeleportSimCharPosOffset = lastKinRootPos - simChar.root.transform.position;
            SimCharController.teleportSimCharRoot(simChar, MMScript.origin, preTeleportSimCharPosOffset);
            lastSimCharTeleportFixedUpdate = curFixedUpdate;
        }
        updateVelocity = lastSimCharTeleportFixedUpdate + 1 < curFixedUpdate;
        UpdateKinCMData(updateVelocity);
        UpdateBoneObsState(updateVelocity, Time.fixedDeltaTime);

        if (curFixedUpdate % _config.EVALUATE_EVERY_K_STEPS == 0)
            RequestDecision();
        else 
            applyActions(prevActionOutput);
        
        if (_config.projectileTraining)
            FireProjectile();
        lastKinRootPos = kinChar.trans.position;
    }

    private void FireProjectile()
    {
        if (Time.time - lastProjectileLaunchtime < _config.LAUNCH_FREQUENCY)
            return;
        lastProjectileLaunchtime = Time.time;
        projectileRB.mass = UnityEngine.Random.Range(_config.PROJECTILE_MIN_WEIGHT, _config.PROJECTILE_MAX_WEIGHT);
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
        Vector3 newKinCM = getCM(kinChar.boneToTransform);
        kinChar.cmVel = updateVelocity ? (newKinCM - kinChar.cm) / Time.fixedDeltaTime : kinChar.cmVel;
        kinChar.cm = newKinCM;
        
    }
    private void UpdateSimCMData(bool updateVelocity)
    {
        Vector3 newSimCM = getCM(simChar.boneToTransform);
        simChar.cmVel = updateVelocity ? (newSimCM - simChar.cm) / Time.fixedDeltaTime : simChar.cmVel;
        simChar.cm = newSimCM;
    }

    private void UpdateBoneSurfacePts(bool updateVelocity)
    {
        foreach (bool isKinChar in new bool[] { true, false })
            for (int i = 1; i < 23; i++)
            {
                var charInfo = isKinChar ? kinChar : simChar;
                Vector3[] newSurfacePts = new Vector3[6];
                Vector3[] newWorldSurfacePts = new Vector3[6];
                Utils.getSixPointsOnCollider(charInfo.boneToCollider[i], ref newWorldSurfacePts, (mm_v2.Bones)i);
                Vector3[] prevWorldSurfacePts = charInfo.boneSurfacePtsWorldSpace[i];

                for (int j = 0; j < 6; j++)
                {
                    newSurfacePts[j] = isKinChar ? resolvePosInKinematicRefFrame(newWorldSurfacePts[j]) :  resolvePosInSimRefFrame(newWorldSurfacePts[j]);
                    if (updateVelocity)
                    {
                        Vector3 surfaceVel = (newWorldSurfacePts[j] - prevWorldSurfacePts[j]) / Time.fixedDeltaTime;
                        charInfo.boneSurfaceVels[i][j] = isKinChar ? resolveVelInKinematicRefFrame(surfaceVel) : resolveVelInSimRefFrame(surfaceVel);
                    }
                }
                charInfo.boneSurfacePtsWorldSpace[i] = newWorldSurfacePts;
                charInfo.boneSurfacePts[i] = newSurfacePts;
            }
    }

    private void UpdateBoneObsState(bool updateVelocity, float dt, bool zeroVelocity = false, bool updateKinOnly = false)
    {
        foreach (bool isKinChar in new bool[]{true, false}) {
            if (updateKinOnly && !isKinChar)
                continue;
            CharInfo curInfo = isKinChar ? kinChar : simChar;
            float[] copyInto = curInfo.boneState;
            int copyIdx = 0;
            for (int j = 0; j < stateBones.Length; j++)
            {
                mm_v2.Bones bone = stateBones[j];
                Vector3 boneWorldPos = curInfo.boneToTransform[(int)bone].position;
                Vector3 boneLocalPos = isKinChar ? resolvePosInKinematicRefFrame(boneWorldPos) : resolvePosInSimRefFrame(boneWorldPos);
                Vector3 prevBonePos = curInfo.boneWorldPos[j];
                Vector3 boneVel = (boneWorldPos - prevBonePos) / dt;
                boneVel = zeroVelocity ? Vector3.zero : isKinChar ? resolveVelInKinematicRefFrame(boneVel) : resolveVelInSimRefFrame(boneVel);
                Utils.copyVecIntoArray(ref copyInto, ref copyIdx, boneLocalPos);

                if (updateVelocity || zeroVelocity)
                    Utils.copyVecIntoArray(ref copyInto, ref copyIdx, boneVel);
                else
                    copyIdx += 3;
                curInfo.boneWorldPos[j] = boneWorldPos;
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


    internal float finalReward = 0f;
    internal int lastEpisodeEndingFrame = 0;
    private bool shouldEndThisFrame = false;
    public void LateFixedUpdate()
    {
        calcAndSetRewards();

        bool inInferenceMode = behaviorParameters.BehaviorType == Unity.MLAgents.Policies.BehaviorType.InferenceOnly;
        if (!inInferenceMode)
            return;
        if (_config.clampKinCharToSim) {
            Quaternion horizontalHeadingRotation = Quaternion.Euler(0f, simChar.trans.rotation.eulerAngles.y, 0f);
            MMScript.clamp_pos_and_rot(new Vector3(simChar.cm.x, 0f, simChar.cm.z), horizontalHeadingRotation);
        }
        UpdateKinCMData(false);
        UpdateBoneObsState(false, Time.fixedDeltaTime, false, true);
        return;
    }
    // returns TRUE if episode ended
    public bool calcAndSetRewards()
    {
        bool heads1mApart;
        double posReward, velReward, localPoseReward, cmVelReward, fallFactor;
        calcFallFactor(out fallFactor, out heads1mApart);
        if ((heads1mApart && curFixedUpdate > lastSimCharTeleportFixedUpdate + 1 && !_config.clampKinCharToSim) || (_config.clampKinCharToSim && shouldEndThisFrame))
        {
            finalReward = _config.EPISODE_END_REWARD;
            SetReward(_config.EPISODE_END_REWARD);
            Debug.Log($"{Time.frameCount}: Calling end episode on: {curFixedUpdate}, lasted {curFixedUpdate - lastEpisodeEndingFrame} frames ({(curFixedUpdate - lastEpisodeEndingFrame)/60f} sec)");
            lastEpisodeEndingFrame = curFixedUpdate;
            shouldEndThisFrame = false;
            EndEpisode();
#if UNITY_EDITOR
            //if (debug)
                //EditorApplication.isPaused = true;
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
        if (curFixedUpdate - _config.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT < lastEpisodeEndingFrame)
            finalReward = 0f;
        else
            finalReward = (float)(fallFactor * (posReward + velReward + localPoseReward + cmVelReward));
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
    16-51 : sim char state
    52-87 : sim state - kin state
    88-110 : numActions (
    */ 
    float[] getState()
    {

        Vector3 cmDistance = resolvePosInKinematicRefFrame(simChar.cm);
        //bool endedEpisode = lastEpisodeEndingFrame >= (curFixedUpdate - 1);
        //Debug.Log($"{Time.frameCount}: getState kinChar.cmVel {kinChar.cmVel} simChar.cmVel {simChar.cmVel} kinCMVelLastGetState: {kinCMVelLastGetState} simCMVelLastGetState: {simCMVelLastGetState}");
        Vector3 kinCMVelInKinRefFrame = resolveVelInKinematicRefFrame(kinChar.cmVel);        
        Vector3 simCMVelInKinRefFrame = resolveVelInSimRefFrame(simChar.cmVel);
        Vector3 desiredVel = resolveVelInKinematicRefFrame(MMScript.desired_velocity);
        Vector3 velDiffSimMinusDesired = simCMVelInKinRefFrame - desiredVel;

        float[] state = new float[numObservations];
        int state_idx = 0;
        Utils.copyVecIntoArray(ref state, ref state_idx, cmDistance);
        Utils.copyVecIntoArray(ref state, ref state_idx, kinCMVelInKinRefFrame);
        Utils.copyVecIntoArray(ref state, ref state_idx, simCMVelInKinRefFrame);
        Utils.copyVecIntoArray(ref state, ref state_idx, simCMVelInKinRefFrame - kinCMVelInKinRefFrame);
        Utils.copyVecIntoArray(ref state, ref state_idx, new Vector2(desiredVel.x, desiredVel.z));
        Utils.copyVecIntoArray(ref state, ref state_idx, new Vector2(velDiffSimMinusDesired.x, velDiffSimMinusDesired.z));

        if (_config.addOrientationDataToObsState)
        {
            float yawDiffKinAndSim = (Quaternion.Inverse(kinChar.trans.rotation) * simulatedCharObj.transform.rotation).GetYAngle();
            float yawDiffDesiredAndSim = (Quaternion.Inverse(MMScript.desired_rotation) * simulatedCharObj.transform.rotation).GetYAngle();
            Utils.copyVecIntoArray(ref state, ref state_idx, MathUtils.getContinuousRepOf2DAngle(yawDiffKinAndSim));
            Utils.copyVecIntoArray(ref state, ref state_idx, MathUtils.getContinuousRepOf2DAngle(yawDiffDesiredAndSim));
        }

        // In the paper, instead of adding s(sim) and s(kin), they add s(sim) and then (s(sim) - s(kin))
        for (int i = 0; i < 36; i++)
            state[state_idx++] = simChar.boneState[i];
        for (int i = 0; i < 36; i++)
            state[state_idx++] = simChar.boneState[i] - kinChar.boneState[i];
        for (int i = 0; i < numActions; i++)
            state[state_idx++] = prevActionOutput[i];
   
        if (state_idx != numObservations)
            throw new Exception($"State may not be properly intialized - length is {state_idx} after copying everything");

        //if (debug)
        //    Utils.debugArray(state, $"{curFixedUpdate} state: ");
        return state;

    }

    // Gets CoM in world position
    public static Vector3 getCM(Transform[] boneToTransform, Vector3[] globalBonePositions = null)
    {
        // We start at 1 because 0 is the root bone with no colliders
        // to calculate CM, we get the masses and centers of each capsule and
        // sum them together and divide by the total mass
        float totalMass = 0f;
        Vector3 CoM = Vector3.zero;
        for (int i = 1; i < boneToTransform.Length; i++)
        {
            Transform t = boneToTransform[i];
            var ab = t.GetComponent<ArticulationBody>();
            float mass = t.GetComponent<ArticulationBody>().mass;
            Vector3 childCenter = globalBonePositions == null ? Utils.getChildColliderCenter(t.gameObject) : globalBonePositions[i];
            CoM += mass * childCenter;
            totalMass += ab.mass;

        }
        return CoM / totalMass;
    }

    // Velocity is different in that we only need to make its rotation
    // local to the kinematic character, whereas with pos we also need to
    // position at the character's CM position
    Vector3 resolveVelInKinematicRefFrame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(kinChar.trans.rotation, vel);
    }
    Vector3 resolveVelInSimRefFrame(Vector3 vel)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(_config.resolveSimReferenceFrameWithSimRotation ? simChar.trans.rotation : kinChar.trans.rotation, vel);
    }
    Vector3 resolvePosInKinematicRefFrame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(kinChar.trans.rotation, pos - kinChar.cm);
    }
    Vector3 resolvePosInSimRefFrame(Vector3 pos)
    {
        // using same logic as in desired_velocity_update
        return MathUtils.quat_inv_mul_vec3(_config.resolveSimReferenceFrameWithSimRotation  ? simChar.trans.rotation : kinChar.trans.rotation, pos - simChar.cm);
    }

    void calcPosAndVelReward(out double posReward, out double velReward)
    {
        // Position reward
        double posDiffsSum = 0f;
        double velDiffsSum = 0f;
        for (int i = 1; i < 23; i++)
        {
            //if (_config.outputIsBase && ((mm_v2.Bones)i == Bone_LeftFoot || (mm_v2.Bones)i == Bone_RightFoot))
            //    continue;
            for (int j = 0; j < 6; j++)
            {
                posDiffsSum += (kinChar.boneSurfacePts[i][j] - simChar.boneSurfacePts[i][j]).magnitude;
                velDiffsSum += (kinChar.boneSurfaceVels[i][j] - simChar.boneSurfaceVels[i][j]).magnitude;
            }
        }
        posReward = Math.Exp((-10f / (nbodies)) *  posDiffsSum );
        velReward = Math.Exp((-1f / (nbodies)) *  velDiffsSum );
    }

    void calcLocalPoseReward(out double poseReward)
    {
        double totalLoss = 0;
        for (int i = 0; i < 23; i++)
        {
            //if (_config.outputIsBase && ((mm_v2.Bones)i == Bone_LeftFoot || (mm_v2.Bones)i == Bone_RightFoot))
            //    continue;
            Transform kinBone = kinChar.boneToTransform[i];
            Transform simBone = simChar.boneToTransform[i];
            // From Stack Overflow:
            //If you want to find a quaternion diff such that diff * q1 == q2, then you need to use the multiplicative inverse:
            // diff * q1 = q2  --->  diff = q2 * inverse(q1)
            Quaternion diff = simBone.localRotation * Quaternion.Inverse(kinBone.localRotation);
            // https://stackoverflow.com/questions/21513637/dot-product-of-two-quaternion-rotations
            Vector3 diff_vec = new Vector3(diff.x, diff.y, diff.z);
            double angle = 2 * Math.Atan2(diff_vec.magnitude, diff.w);
            // We want the magnitude of the diff so we take abs value
            angle = Math.Abs(GeoUtils.wrap_radians((float)angle));
            totalLoss += (float)angle;
            //Debug.Log($"Bone: {(mm_v2.Bones)i} Quaternion loss: {angle} totalLoss: {totalLoss}");
        }
        poseReward = Math.Exp((-10f/nbodies) * totalLoss);
    }

    void calcCMVelReward(out double cmVelReward)
    {
        cmVelReward = Math.Exp(-1d * (resolveVelInKinematicRefFrame(kinChar.cmVel) - resolveVelInSimRefFrame(simChar.cmVel)).magnitude);
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
        if (!_config.clampKinCharToSim)
            return;
        foreach (ContactPoint contact in collision.contacts)
        {
            string colliderName = contact.thisCollider.gameObject.name;
            if (!colliderName.ToLower().Contains("toe") && !colliderName.ToLower().Contains("foot") && !colliderName.ToLower().Contains("leg_") && contact.otherCollider.gameObject.name == "Ground")
            {
                Debug.Log($"Collider name: {colliderName} other collider name: {contact.otherCollider.gameObject.name}");
                shouldEndThisFrame = true;
            }
            //Debug.Log($"{colliderName} collided with {contact.otherCollider.gameObject.name}");
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
        return Mathf.Min(topLeft.y, topRight.y, bottomLeft.y, bottomRight.y);
    }

    private float getVerticalOffset()
    {
        // ClearGizmos();
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
            Utils.getSixPointsOnCollider(kin_collider_obj, ref gizmos, bone);            
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

    private void OnGUI()
    {
        if (!_config.rewardsInGUI)
            return;
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 16;
        GUI.Label(new Rect(100, 175, 600, 100), "Last Reward: " + finalReward.ToString(), fontSize);
    }
}
