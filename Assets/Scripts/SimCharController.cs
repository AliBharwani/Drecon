using System;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using static MotionMatchingAnimator.Bones;
public class SimCharController : MonoBehaviour
{
    public bool apply_all_local_rots;
    public bool isKinematic;
    public int frameIdx = 500;
    public MotionMatchingAnimator.Bones[] bones_to_apply;
    public MotionMatchingAnimator.Bones debug_bone;
    public Transform[] boneToTransform = new Transform[23];
    public ArticulationBody[] boneToArtBody = new ArticulationBody[23];
    public bool debug = false;
    internal Quaternion[] startingRotations = new Quaternion[23];
    private ConfigManager _config;
    public static MocapDB db;

    void Awake()
    {
        _config = ConfigManager.Instance;
        db = MocapDB.Instance;
        //foreach (var t in GetComponentsInChildren<Transform>())
        //    t.gameObject.AddComponent<CollisionDebugger>();
        initBoneToCollider();
        initBoneToArtBodies();
        initArticulationDrives();
        setupIgnoreCollisions();
        if (!debug)
            return;
#if UNITY_EDITOR
        if (Application.isEditor)
            UnityEditor.EditorWindow.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
#endif
        if (isKinematic)
        {
            foreach (var body in GetComponentsInChildren<ArticulationBody>())
                body.enabled = false;
            foreach (var collider in GetComponentsInChildren<Collider>())
                collider.enabled = false;
        }

    }

    private void initArticulationDrives()
    {
        for(int i = 1; i < 23; i++)
        {
            //Debug.Log($"Setting {(mm_v2.Bones)i} to {_config.boneToStiffness[i
            bool musclePowerExists = _config.MusclePowers.Any(x => x.Bone == (MotionMatchingAnimator.Bones)i);
            Vector3 musclePower = musclePowerExists ? _config.MusclePowers.First(x => x.Bone == (MotionMatchingAnimator.Bones)i).PowerVector : Vector3.zero;
            if (musclePowerExists)
                boneToArtBody[i].SetAllDriveStiffness(musclePower);
            else
                boneToArtBody[i].SetAllDriveStiffness(_config.boneToStiffness[i]);

            Vector3 damping;
            if (_config.dampingScalesWithStiffness)
                damping = musclePowerExists ? musclePower * .1f : Vector3.one * _config.boneToStiffness[i] * .1f;
            else
                damping = Vector3.one * _config.damping;
            boneToArtBody[i].SetAllDriveDamping(damping);
            boneToArtBody[i].SetAllForceLimit(_config.forceLimit);
        }
    }

    public Collider[] boneToCollider;
    private void initBoneToCollider()
    {
        boneToCollider = new Collider[db.nbones()];
        for (int i = 1; i < db.nbones(); i++)
        {
            Transform trans = boneToTransform[i];
            if (i == (int) Bone_LeftFoot || i == (int) Bone_RightFoot)
                boneToCollider[i] = UnityObjUtils.getChildBoxCollider(trans.gameObject).GetComponent<Collider>();
            else
                boneToCollider[i] = UnityObjUtils.getChildCapsuleCollider(trans.gameObject).GetComponent<Collider>();
            if (boneToCollider[i] == null)
                Debug.Log($"Could not find collider for {(MotionMatchingAnimator.Bones)i }");
        }
    }

    private void setupIgnoreCollisions()
    {
        if (!_config.selfCollision)
            return;
        Physics.IgnoreCollision(boneToCollider[(int)Bone_LeftArm], boneToCollider[(int)Bone_Spine2]);
        Physics.IgnoreCollision(boneToCollider[(int)Bone_RightArm], boneToCollider[(int)Bone_Spine2]);

        int[] torsoColliders = new int[] { (int)Bone_Neck, (int)Bone_LeftShoulder, (int)Bone_RightShoulder, (int) Bone_Spine2,
                                            (int) Bone_Spine1, (int) Bone_Spine, (int) Bone_Hips, (int) Bone_Head};

        int[] feetColliders = new int[] { (int)Bone_LeftUpLeg, (int)Bone_RightUpLeg, (int)Bone_LeftLeg, (int)Bone_RightLeg, (int)Bone_LeftFoot, (int)Bone_LeftToe, (int)Bone_RightFoot, (int)Bone_RightToe };

        for (int i = 0; i < torsoColliders.Length; i++)
            for (int j = i + 1; j < torsoColliders.Length; j++)
                Physics.IgnoreCollision(boneToCollider[torsoColliders[i]], boneToCollider[torsoColliders[j]]);

        for (int i = 0; i < feetColliders.Length; i++)
            for (int j = i + 1; j < feetColliders.Length; j++)
                Physics.IgnoreCollision(boneToCollider[feetColliders[i]], boneToCollider[feetColliders[j]]);

        for (int i = 2; i < db.nbones(); i++) // start at 2 because hip has no parent collider
        {
            int parent = db.bone_parents[i];
            Physics.IgnoreCollision(boneToCollider[i], boneToCollider[parent]);
            Physics.IgnoreCollision(boneToCollider[i], boneToCollider[(int)Bone_Head]);
            Physics.IgnoreCollision(boneToCollider[i], boneToCollider[(int)Bone_Neck]);
            if (i == (int) Bone_LeftUpLeg || i == (int) Bone_RightUpLeg)
                Physics.IgnoreCollision(boneToCollider[i], boneToCollider[(int)Bone_Spine]);
           
        }
    }

    [ContextMenu("initBoneToArtBodies()")]
    public void initBoneToArtBodies()
    {
        for (int i = 0; i < 23; i++)
        {
            MotionMatchingAnimator.Bones bone = (MotionMatchingAnimator.Bones)i;
            ArticulationBody ab = boneToTransform[i].GetComponent<ArticulationBody>();
            boneToArtBody[i] = ab;
        }
    }

    void FixedUpdate()
    {
        if (!debug)
            return;
        frameIdx++;
        playFrameIdx();
    }

    public bool setDriveTargetVels = false;
    private void playFrameIdx()
    {
        Vector3[] curr_bone_positions = db.bone_positions[frameIdx];
        Quaternion[] curr_bone_rotations = db.bone_rotations[frameIdx];
        Vector3[] curr_bone_angular_vel = db.bone_angular_velocities[frameIdx];
        for (int i = 1; i < 23; i++)
        {
            MotionMatchingAnimator.Bones bone = (MotionMatchingAnimator.Bones)i;
            if (!apply_all_local_rots && !bones_to_apply.Contains(bone))
                continue;
            Transform t = boneToTransform[i];
            Quaternion local_rot = curr_bone_rotations[i];

            if (isKinematic)
            {
                t.localPosition = curr_bone_positions[i];
                t.localRotation = local_rot;
                continue;
            }
            ArticulationBody ab = boneToArtBody[i];
            if (ab == null)
                continue;
            ab.SetDriveRotation(local_rot);
            if (setDriveTargetVels) {
                Quaternion curRot = curr_bone_rotations[i];
                Vector3 targetAngularVelocity = curr_bone_angular_vel[i];
                ab.SetDriveTargetVelocity(targetAngularVelocity, curRot);
            }

        }

    }

    public static void teleportSimCharRoot(CharInfo simChar, Vector3 newKinCharPos, Vector3 simCharPosOffset)
    {
        if (db == null)
            db = getDB();

        // simPosOffset = how far off was the sim char from the kin char before teleport? 
        // Before teleport: simCharPos + simCharPosOffset = kinCharPos ; simCharPosOffset = kinCharPos - simCharPos
        // After teleport: newSimCharPos + simCharPosOffset = newKinCharPos ; newSimCharPos = newKinCharPos - simCharPosOffset
        Vector3 newSimCharPos = newKinCharPos - simCharPosOffset;
        simChar.root.TeleportRoot(newSimCharPos, simChar.root.transform.rotation);
        for (int i = 1; i < 23; i++)
        {
            MotionMatchingAnimator.Bones bone = (MotionMatchingAnimator.Bones)i;
            ArticulationBody body = simChar.boneToArtBody[i];
            if (body.jointType != ArticulationJointType.SphericalJoint)
            {
                continue;
            }
            Quaternion targetLocalRot = simChar.boneToTransform[i].localRotation;
            setArtBodyDrivesToRotationAndReset(body, targetLocalRot, false);
        }
    }
    private static void setArtBodyDrivesToRotationAndReset(ArticulationBody body, Quaternion targetRot, bool resetEverything, bool doNotSetZRot = false)
    {
        Vector3 TargetRotationInJointSpace = body.ToTargetRotationInReducedSpace(targetRot, false);
        if (body.dofCount == 3)
        {
            body.resetJointPosition(doNotSetZRot ? new Vector3(TargetRotationInJointSpace.x, TargetRotationInJointSpace.y, 0f) : TargetRotationInJointSpace, resetEverything);
            TargetRotationInJointSpace *= Mathf.Rad2Deg;
            var drive = body.xDrive;
            drive.target = TargetRotationInJointSpace.x;
            body.xDrive = drive;

            drive = body.yDrive;
            drive.target = TargetRotationInJointSpace.y;
            body.yDrive = drive;

            drive = body.zDrive;
            drive.target = TargetRotationInJointSpace.z;
            body.zDrive = drive;
        }
        else if (body.dofCount == 1)
        {
            float new_target = 0f;
            if (body.twistLock != ArticulationDofLock.LockedMotion)
                new_target = TargetRotationInJointSpace.x;
            else if (body.swingYLock != ArticulationDofLock.LockedMotion)
                new_target = TargetRotationInJointSpace.y;
            else if (body.swingZLock != ArticulationDofLock.LockedMotion)
                new_target = TargetRotationInJointSpace.z;
            body.resetJointPosition(new_target, resetEverything);
            TargetRotationInJointSpace *= Mathf.Rad2Deg;
            var drive = body.zDrive;
            drive.target = TargetRotationInJointSpace.z;
            body.zDrive = drive;
        }
    }
    public static void teleportSimChar(CharInfo sim_char, CharInfo kin_char,  float verticalOffset = .15f, bool setVelocities = false)
        {
        if (db == null)
            db = getDB();
        sim_char.trans.rotation = kin_char.trans.rotation;
        Transform kin_root = kin_char.boneToTransform[(int)Bone_Entity];
        Transform kinHips = kin_char.boneToTransform[(int)Bone_Hips];
        Transform simHips = sim_char.boneToTransform[(int)Bone_Hips];
        // Adding this to root transform position will give hip transform position
        Vector3 simHipPositionOffset = sim_char.trans.position - simHips.position;
        // we need to set: 
        // simRootPosition + simHipPositionOffset = kinHipPosition 
        // simRootPosition = kinHipPosition - simHipPositionOffset

        // We teleport the sim char a little higher to prevent it from clipping into the ground and bouncing off
        sim_char.root.TeleportRoot(kinHips.position + simHipPositionOffset + Vector3.up * verticalOffset, kin_root.rotation);
        sim_char.root.resetJointPhysics();
        if (setVelocities)
        {
            sim_char.root.velocity = kin_char.MMScript.curr_bone_velocities[0];
        }
        for (int i = 1; i < 23; i++)
        {
            MotionMatchingAnimator.Bones bone = (MotionMatchingAnimator.Bones)i;
            ArticulationBody body = sim_char.boneToArtBody[i];
            if (body.jointType != ArticulationJointType.SphericalJoint)
            {
                body.resetJointPhysics();
                continue;
            }
            Quaternion targetLocalRot = kin_char.boneToTransform[i].localRotation;
            bool isFootBone = bone == Bone_LeftFoot || bone == Bone_RightFoot;
            setArtBodyDrivesToRotationAndReset(body, targetLocalRot, true, isFootBone);

        }

    }

    [ContextMenu("Remove joint limits")]
    internal void removeJointLimits()
    {
        for(int i = 1; i < 23; i++)
        {
            var body = boneToTransform[i].GetComponent<ArticulationBody>();
            if (body.isRoot ||  body.jointType != ArticulationJointType.SphericalJoint )
                continue;
            body.twistLock = ArticulationDofLock.FreeMotion;
            body.swingYLock = ArticulationDofLock.FreeMotion;
            body.swingZLock = ArticulationDofLock.FreeMotion;
        }
    }

    [ContextMenu("applyTransformsFromFrameZero ")]
    internal void applyTransformsFromFrameZero()
    {
        // This is neccessary because the prefabs are based off the FBX file
        // However, the BVH file starts off by defining offsets for each bone
        // In the kinematic character, these offsets are baked into the positions and rotations
        // information created by the python script and directly applied to the character
        // The simulated character however does have that 
        if (db == null)
            db = getDB();
        int numBones = db.nbones();
        Vector3[] globalBonePositions = new Vector3[numBones];
        Quaternion[] globalBoneRotations = new Quaternion[numBones];
        forward_kinematics_full(db, 0, ref globalBonePositions, ref globalBoneRotations);
        apply_global_pos_and_rot(globalBonePositions,   globalBoneRotations, boneToTransform);
    }

    private static MocapDB getDB()
    {
        if (db == null)
        {
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPlaying)
                db = MocapDB.Instance;
            else
                db = new MocapDB();
#else
            db = MocapDB.Instance;
#endif
        }
        return db;
    }
    [ContextMenu("Set all art body rot limits")]
    public void setArtBodyRotLimitsFromDB()
    {
        if (db == null)
            db = getDB();
        int num_frames = db.nframes();


        for (int j = 1; j < db.nbones(); j++)
        {
            //Debug.Log($"Bone {(mm_v2.Bones)j}");
            ArticulationBody ab = boneToTransform[j].GetComponent<ArticulationBody>();

            float min_x, min_y, min_z;
            min_x = min_y = min_z = float.PositiveInfinity;
            float max_x, max_y, max_z;
            max_x = max_y = max_z = float.NegativeInfinity;

            for (int i = 0; i < num_frames; i++)
            {
                Quaternion debug_bone_rot = db.bone_rotations[i][j];
                Vector3 target_rot = ab.ToTargetRotationInReducedSpace(debug_bone_rot, true);
                //Debug.Log(target_rot.ToString("f6"));
                min_x = Mathf.Min(min_x, target_rot.x);
                min_y = Mathf.Min(min_y, target_rot.y);
                min_z = Mathf.Min(min_z, target_rot.z);

                max_x = Mathf.Max(max_x, target_rot.x);
                max_y = Mathf.Max(max_y, target_rot.y);
                max_z = Mathf.Max(max_z, target_rot.z);
            }
            var drive = ab.xDrive;
            drive.lowerLimit = min_x;
            drive.upperLimit = max_x;
            ab.xDrive = drive;

            drive = ab.yDrive;
            drive.lowerLimit = min_y;
            drive.upperLimit = max_y;
            ab.yDrive = drive;

            drive = ab.zDrive;
            drive.lowerLimit = min_z;
            drive.upperLimit = max_z;
            ab.zDrive = drive;
        }
    }

    [ContextMenu("populateStartingRotations")]
    private void populateStartingRotations()
    {
        for (int i = 0; i < 23; i++)
            startingRotations[i] = boneToTransform[i].localRotation;
    }

    [ContextMenu("Turn off capsule renderers")]
    private void turnOffCapsuleRenderers()
    {
        foreach (var rend in GetComponentsInChildren<MeshRenderer>())
            rend.gameObject.SetActive(false);
    }

    private static void forward_kinematics_full(
     MocapDB db,
     int frame,
     ref Vector3[] global_bone_positions,
     ref Quaternion[] global_bone_rotations
    )
    {
        int[] bone_parents = db.bone_parents;

        for (int i = 0; i < bone_parents.Length; i++)
        {
            // Assumes bones are always sorted from root onwards
            if (bone_parents[i] == -1)
            {
                global_bone_positions[i] = db.bone_positions[frame][i];
                global_bone_rotations[i] = db.bone_rotations[frame][i];
            }
            else
            {
                Vector3 parent_position = global_bone_positions[bone_parents[i]];
                Quaternion parent_rotation = global_bone_rotations[bone_parents[i]];
                global_bone_positions[i] = MathUtils.quat_mul_vec3(parent_rotation, db.bone_positions[frame][i]) + parent_position;
                global_bone_rotations[i] = parent_rotation * db.bone_rotations[frame][i];
            }
        }
    }

    private static void apply_global_pos_and_rot(Vector3[] global_bone_positions,
      Quaternion[] global_bone_rotations, Transform[] boneToTransform)
    {
        //Debug.Log(numBones); // 23
        for (int i = 0; i < boneToTransform.Length; i++)
        {
            Transform t = boneToTransform[i];
            t.position = global_bone_positions[i];
            t.rotation = global_bone_rotations[i];
        }
    }
}
