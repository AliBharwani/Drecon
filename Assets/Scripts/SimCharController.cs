using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using static mm_v2.Bones;
public class SimCharController : MonoBehaviour
{
    public bool apply_all_local_rots;
    public bool isKinematic;
    public int frameIdx = 500;
    public mm_v2.Bones[] bones_to_apply;
    public mm_v2.Bones debug_bone;
    public Transform[] boneToTransform = new Transform[23];
    public ArticulationBody[] boneToArtBody = new ArticulationBody[23];
    public bool is_active = false;
    internal Quaternion[] startingRotations = new Quaternion[23];
    private ConfigWriter _config;
    void Awake()
    {
        _config = ConfigWriter.Instance;
        db = database.Instance;
        initBoneToCollider();
        initBoneToArtBodies();
        initArticulationDrives();
        setupIgnoreCollisions();
        if (!is_active)
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
    [ContextMenu("initArticulationDrives()")]

    private void initArticulationDrives()
    {
        for(int i = 1; i < 23; i++)
        {
            //Debug.Log($"Setting {(mm_v2.Bones)i} to {_config.boneToStiffness[i
            if (_config.MusclePowers.Any(x => x.Bone == (mm_v2.Bones)i))
                boneToArtBody[i].SetAllDriveStiffness(_config.MusclePowers.First(x => x.Bone == (mm_v2.Bones)i).PowerVector);
            else
                boneToArtBody[i].SetAllDriveStiffness(_config.boneToStiffness[i]);
            boneToArtBody[i].SetAllDriveDamping(_config.damping);
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
                boneToCollider[i] = ArtBodyTester.getChildBoxCollider(trans.gameObject).GetComponent<Collider>();
            else
                boneToCollider[i] = ArtBodyTester.getChildCapsuleCollider(trans.gameObject).GetComponent<Collider>();
            if (boneToCollider[i] == null)
                Debug.Log($"Could not find collider for {(mm_v2.Bones)i }");
        }
    }

    private void setupIgnoreCollisions()
    {
        Physics.IgnoreCollision(boneToCollider[(int)Bone_LeftUpLeg], boneToCollider[(int)Bone_RightUpLeg]);
        Physics.IgnoreCollision(boneToCollider[(int)Bone_LeftArm], boneToCollider[(int)Bone_Spine2]);
        Physics.IgnoreCollision(boneToCollider[(int)Bone_RightArm], boneToCollider[(int)Bone_Spine2]);

        int[] torsoColliders = new int[] { (int)Bone_Neck, (int)Bone_LeftShoulder, (int)Bone_RightShoulder, (int) Bone_Spine2,
                                            (int) Bone_Spine1, (int) Bone_Spine, (int) Bone_Hips, (int) Bone_Head};
        for (int i = 0; i < torsoColliders.Length; i++)
            for (int j = i + 1; j < torsoColliders.Length; j++)
                Physics.IgnoreCollision(boneToCollider[torsoColliders[i]], boneToCollider[torsoColliders[j]]);
        for (int i = 2; i < db.nbones(); i++) // start at 2 because hip has no parent collider
        {
            int parent = db.bone_parents[i];
            Physics.IgnoreCollision(boneToCollider[i], boneToCollider[parent]);
            if (i == (int) Bone_LeftUpLeg || i == (int) Bone_RightUpLeg)
                Physics.IgnoreCollision(boneToCollider[i], boneToCollider[(int)Bone_Spine]);
           
        }
    }

    [ContextMenu("initBoneToArtBodies()")]
    public void initBoneToArtBodies()
    {
        for (int i = 0; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            ArticulationBody ab = boneToTransform[i].GetComponent<ArticulationBody>();
            boneToArtBody[i] = ab;
        }
    }

    void FixedUpdate()
    {
        if (!is_active)
            return;
        frameIdx++;
        playFrameIdx();
    }

    private void playFrameIdx()
    {
        Vector3[] curr_bone_positions = db.bone_positions[frameIdx];
        Quaternion[] curr_bone_rotations = db.bone_rotations[frameIdx];
        for (int i = 1; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            if (!apply_all_local_rots && !bones_to_apply.Contains(bone))
                continue;
            Transform t = boneToTransform[i];
            Quaternion local_rot = curr_bone_rotations[i];
            bool print_debug = bone == debug_bone;

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
            

        }

    }

    public static database db;

    public static void teleportSimChar(CharInfo sim_char, CharInfo kin_char, bool setDriveTargets = false, float verticalOffset = .15f)
    {
        if (db == null)
            db = getDB();
        //Debug.Log($"Teleport sim char called");
        //Destroy(simulated_char);
        //simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        //my_initalize();
        sim_char.trans.rotation = kin_char.trans.rotation;
        Transform kin_root = kin_char.boneToTransform[(int)Bone_Entity];
        Transform kinHips = kin_char.boneToTransform[(int)Bone_Hips];
        Transform simHips = sim_char.boneToTransform[(int)Bone_Hips];
        // Adding this to root transform position will give hip transform position
        Vector3 simHipPositionOffset = sim_char.trans.position - simHips.position;
        // we need to set: 
        // simRootPosition + simHipPositionOffset = kinHipPosition 
        // simRootPosition = kinHipPosition - simHipPositionOffset
        //Debug.Log($"simRootPosition: {sim_char.trans.position}  simHipPositionOffset : {simHipPositionOffset} kinHipPosition: {kinHips.position}");

        // We teleport the sim char a little higher to prevent it from clipping into the ground and bouncing off
        //Vector3 verticalOffset = ;
        sim_char.root.TeleportRoot(kinHips.position + simHipPositionOffset + Vector3.up * verticalOffset, kin_root.rotation);
        sim_char.root.resetJointPhysics();

        for (int i = 1; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            ArticulationBody body = sim_char.boneToArtBody[i];

            if (body.jointType != ArticulationJointType.SphericalJoint)
            {
                body.resetJointPhysics();
                continue;
            }
            Quaternion targetLocalRot = kin_char.boneToTransform[i].localRotation;
            // from https://github.com/Unity-Technologies/marathon-envs/blob/58852e9ac22eac56ca46d1780573cc6c32278a71/UnitySDK/Assets/MarathonEnvs/Scripts/ActiveRagdoll003/DebugJoints.cs
            Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRot) * body.parentAnchorRotation).eulerAngles ;
            TargetRotationInJointSpace = new Vector3(
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.x),
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.y),
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.z)) * Mathf.Deg2Rad;

            if (body.dofCount == 3)
            {
                body.resetJointPosition(TargetRotationInJointSpace);
                var drive = body.xDrive;
                drive.target = setDriveTargets ? TargetRotationInJointSpace.x : 0f;
                body.xDrive = drive;

                drive = body.yDrive;
                drive.target = setDriveTargets ? TargetRotationInJointSpace.y : 0f;
                body.yDrive = drive;

                drive = body.zDrive;
                drive.target = setDriveTargets ? TargetRotationInJointSpace.z : 0f;
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
                body.resetJointPosition(new_target);
                var drive = body.zDrive;
                drive.target = setDriveTargets ? TargetRotationInJointSpace.z : 0f;
                body.zDrive = drive;
            }
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

    private static database getDB()
    {
        if (db == null)
        {
#if UNITY_EDITOR
            if (UnityEditor.EditorApplication.isPlaying)
                db = database.Instance;
            else
                db = new database();
#else
            db = new database();
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
#if UNITY_EDITOR
            if (!UnityEditor.EditorApplication.isPlaying && TestDirector.allLimitedDOFBones.Contains((mm_v2.Bones)j))
            { 
                Debug.Log($"Bone {(mm_v2.Bones)j} Mins: x: {min_x} , y: {min_y} , z: {min_z}");
                Debug.Log($"Bone {(mm_v2.Bones)j} Maxess: x: {max_x} , y: {max_y} , z: {max_z}");
            }
#endif
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

    // Useful for normalizing inputs 
    [ContextMenu("Find mins and maxes for velocities and positions")]
        public void min_max_debugger()
    {
        int num_state_bones = MLAgentsDirector.stateBones.Length;
        Vector3 vel_min = Vector3.positiveInfinity;
        Vector3 vel_max = Vector3.negativeInfinity;
        Vector3[] bone_pos_mins = new Vector3[num_state_bones];
        Vector3[] bone_pos_maxes = new Vector3[num_state_bones];
        Vector3[] bone_vel_mins = new Vector3[num_state_bones];
        Vector3[] bone_vel_maxes = new Vector3[num_state_bones];
        find_mins_and_maxes(boneToTransform, ref vel_min, ref vel_max, ref bone_pos_mins, ref bone_pos_maxes, ref bone_vel_mins, ref bone_vel_maxes);
        Debug.Log($"vel_min: {vel_min.ToString("f6")}");
        Debug.Log($"vel_max: {vel_max.ToString("f6")}");
        for (int j = 0; j < num_state_bones; j++)
        {
            //Debug.Log($"{MLAgentsDirector.state_bones[j]} bone pos min: {bone_pos_mins[j].ToString("f6")}");
            //Debug.Log($"{MLAgentsDirector.state_bones[j]} bone pos max: {bone_pos_maxes[j].ToString("f6")}");
            Debug.Log($"{MLAgentsDirector.stateBones[j]} bone vel min: {bone_vel_mins[j].ToString("f6")} || bone vel max: {bone_vel_maxes[j].ToString("f6")}");
            //Debug.Log($"{MLAgentsDirector.state_bones[j]} bone vel max: {bone_vel_maxes[j].ToString("f6")}");

        }
    }
    public static void find_mins_and_maxes(Transform[] boneToTransform,
        ref Vector3 cm_vel_min,
        ref Vector3 cm_vel_max,
        ref Vector3[] bone_pos_mins,
        ref Vector3[] bone_pos_maxes,
        ref Vector3[] bone_vel_mins,
        ref Vector3[] bone_vel_maxes)
    {
        if (db == null)
            db = getDB();
        int num_frames = db.nframes();
        Vector3 last_cm = Vector3.zero;
        Vector3[] global_pos = new Vector3[23];
        Quaternion[] global_rots = new Quaternion[23];
        int num_state_bones = MLAgentsDirector.stateBones.Length;
        Vector3[] state_bone_pos = new Vector3[num_state_bones];

        cm_vel_min = Vector3.positiveInfinity;
        cm_vel_max = Vector3.negativeInfinity;
        bone_pos_mins = new Vector3[num_state_bones];
        bone_pos_maxes = new Vector3[num_state_bones];
        bone_vel_mins = new Vector3[num_state_bones];
        bone_vel_maxes = new Vector3[num_state_bones];

        float frame_time = 1f / 30f;
        // 157 is where running data starts in the bvh database
        // 7153 is where it ends 
        for (int i = 0; i < num_frames; i++)
        {
            forward_kinematics_full(db, i, ref global_pos, ref global_rots);
            bool update_velocity = !db.range_starts.Contains(i);
            //apply_global_pos_and_rot(global_pos, global_rots, boneToTransform);
            Vector3 cm = MLAgentsDirector.getCM(boneToTransform, global_pos);
            Vector3 cm_vel =  (cm - last_cm) / frame_time;
            cm_vel = Utils.quat_inv_mul_vec3(global_rots[0], cm_vel);
            if (update_velocity)
                updated_mins_and_maxes(cm_vel, ref cm_vel_min, ref cm_vel_max);

            last_cm = cm;
            for (int j = 0; j < num_state_bones; j++)
            {
                // Get state bone pose
                Vector3 bone_pos = global_pos[(int)MLAgentsDirector.stateBones[j]];
                // Resolve in reference frame (multiply by inverse of root rotation and subtract root position)
                Vector3 local_bone_pos = Utils.quat_inv_mul_vec3(global_rots[0], bone_pos - cm);
                Vector3 bone_vel = (bone_pos - state_bone_pos[j]) / frame_time;
                bone_vel = Utils.quat_inv_mul_vec3(global_rots[0], bone_vel);
                // Compare min maxes
                updated_mins_and_maxes(local_bone_pos, ref  bone_pos_mins[j], ref bone_pos_maxes[j]);
                if (update_velocity)
                    updated_mins_and_maxes(bone_vel, ref bone_vel_mins[j], ref bone_vel_maxes[j]);
                state_bone_pos[j] = bone_pos;
            }
        }

    }

    public static void updated_mins_and_maxes(Vector3 val, ref Vector3 mins, ref Vector3 maxes)
    {
        maxes = update_maxes(maxes, val);
        mins = update_mins(mins, val);
    }

    public static Vector3 update_maxes(Vector3 vel_max, Vector3 cm_vel)
    {
        return new Vector3(Mathf.Max(vel_max.x, cm_vel.x), Mathf.Max(vel_max.y, cm_vel.y),Mathf.Max(vel_max.z, cm_vel.z));
    }

    public static Vector3 update_mins(Vector3 vel_min, Vector3 cm_vel)
    {
        return new Vector3(Mathf.Min(vel_min.x, cm_vel.x), Mathf.Min(vel_min.y, cm_vel.y), Mathf.Min(vel_min.z, cm_vel.z));
    }

    private static void forward_kinematics_full(
     database db,
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
                global_bone_positions[i] = Utils.quat_mul_vec3(parent_rotation, db.bone_positions[frame][i]) + parent_position;
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
