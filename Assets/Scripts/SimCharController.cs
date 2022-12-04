using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class SimCharController : MonoBehaviour
{
    public bool apply_all_local_rots;
    public bool set_art_bodies;
    public bool set_target_velocities;
    public int frameIdx = 500;
    public mm_v2.Bones[] bones_to_apply;
    public mm_v2.Bones debug_bone;
    public Transform[] boneToTransform = new Transform[23];
    public ArticulationBody[] bone_to_art_body = new ArticulationBody[23];
    public int start_delay = 60;
    Gamepad gamepad;
    database motionDB;
    public float stiffness = 120f;
    public float damping = 3f;
    public float force_limit = 200f;

    void Start()
    {
#if UNITY_EDITOR
        if (Application.isEditor)
            UnityEditor.EditorWindow.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
#endif

        //Application.targetFrameRate = 30;
        //motionDB = new database(Application.dataPath + @"/outputs/database.bin");
        motionDB = database.Instance;

        initBoneToArtBodies();
    }

    public void initBoneToArtBodies()
    {
        for (int i = 0; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            ArticulationBody ab = boneToTransform[i].GetComponent<ArticulationBody>();
            bone_to_art_body[i] = ab;
        }
    }

    void FixedUpdate()
    {
       // frameIdx++;
        //playFrameIdx();
    }

    private void playFrameIdx()
    {
        Vector3[] curr_bone_positions = motionDB.bone_positions[frameIdx];
        Quaternion[] curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        for (int i = 1; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            if (!apply_all_local_rots && !bones_to_apply.Contains(bone))
                continue;
            Transform t = boneToTransform[i];
            Quaternion local_rot = curr_bone_rotations[i];
            bool print_debug = bone == debug_bone;

            if (!set_art_bodies)
            {
                t.localPosition = curr_bone_positions[i];
                t.localRotation = local_rot;
                continue;
            }
            ArticulationBody ab = bone_to_art_body[i];
            if (ab == null || !bones_to_apply.Contains(bone))
                continue;

            if (!set_target_velocities)
            {
                // Angle is in range (-1, 1) => map to (-180, 180)
                Vector3 target = ab.ToTargetRotationInReducedSpace(local_rot);
                bool use_xdrive = bone == mm_v2.Bones.Bone_LeftLeg || bone == mm_v2.Bones.Bone_RightLeg;
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
                ab.SetDriveRotation(local_rot, print_debug);
            }

        }

    }

    public static database db;
    public static bool[] anchor_rot_found = new bool[23];
    public static Quaternion[] bone_anchor_rot = new Quaternion[23];
    public static Quaternion get_bone_parent_rotation(int bone_idx, ArticulationBody[] bone_to_art_body)
    {
        bool is_hip = (mm_v2.Bones)bone_idx == mm_v2.Bones.Bone_Hips;
        if (anchor_rot_found[bone_idx] == true || is_hip)
            return is_hip ? Quaternion.identity : bone_anchor_rot[bone_idx];
        if (db == null)
            db = database.Instance;
        int parent_idx = db.bone_parents[bone_idx];
        Quaternion grandparent_rotation = get_bone_parent_rotation(parent_idx, bone_to_art_body);
        Quaternion parent_rotation = grandparent_rotation * bone_to_art_body[bone_idx].parentAnchorRotation;
        Debug.Log($"{(mm_v2.Bones)bone_idx}'s parent's rotation is  {(mm_v2.Bones)parent_idx}'s parent's rotation times current parentAnchorRotation");
        bone_anchor_rot[bone_idx] = parent_rotation;
        anchor_rot_found[bone_idx] = true;
        return parent_rotation;
    }

    public static void teleport_sim_char(char_info sim_char, char_info kin_char)
    {
//        if (db == null)
//        {
//#if UNITY_EDITOR
//            db = new database();
//#else
//            db = database.Instance;
//#endif
//        }

        //Destroy(simulated_char);
        //simulated_char = Instantiate(simulated_char_prefab, Vector3.zero, Quaternion.identity);
        //my_initalize();
        sim_char.char_trans.rotation = kin_char.char_trans.rotation;
        Transform kin_hip = kin_char.bone_to_transform[(int)mm_v2.Bones.Bone_Hips];
        sim_char.hip_bone.TeleportRoot(kin_hip.position, kin_hip.rotation);
        sim_char.hip_bone.resetJointPhysics();
        //Quaternion[] global_ab_rots = new Quaternion[23];
        //global_ab_rots[1] = sim_char.hip_bone.anchorRotation;
        for (int i = 2; i < 23; i++)
        {
            ArticulationBody body = sim_char.bone_to_art_body[i];
            //Quaternion parent_anchor_rot = get_bone_parent_rotation(i, sim_char.bone_to_art_body);// global_ab_rots[i - 1];
            //global_ab_rots[i] = parent_anchor_rot * body.anchorRotation;
            if (body.jointType != ArticulationJointType.SphericalJoint)
            {
                body.resetJointPhysics();
                continue;
            }
            Quaternion targetLocalRot = kin_char.bone_to_transform[i].localRotation;
            // from https://github.com/Unity-Technologies/marathon-envs/blob/58852e9ac22eac56ca46d1780573cc6c32278a71/UnitySDK/Assets/MarathonEnvs/Scripts/ActiveRagdoll003/DebugJoints.cs
            Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRot) * body.parentAnchorRotation).eulerAngles;
            //TargetRotationInJointSpace = body.ToTargetRotationInReducedSpace(targetLocalRot);
            //TargetRotationInJointSpace = (Quaternion.Inverse(body.anchorRotation) * targetLocalRot * body.parentAnchorRotation).eulerAngles;
            //TargetRotationInJointSpace = (targetLocalRot * Quaternion.Inverse(body.anchorRotation)).eulerAngles;
            TargetRotationInJointSpace = new Vector3(
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.x),
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.y),
                Mathf.DeltaAngle(0, TargetRotationInJointSpace.z)) * Mathf.Deg2Rad;

            if (body.dofCount == 3)
            {
                //if (body.twistLock == ArticulationDofLock.LockedMotion)
                //    TargetRotationInJointSpace.x = 0f;
                //if (body.swingYLock == ArticulationDofLock.LockedMotion)
                //    TargetRotationInJointSpace.y = 0f;
                //if (body.swingZLock == ArticulationDofLock.LockedMotion)
                //    TargetRotationInJointSpace.z = 0f;
                body.resetJointPosition(TargetRotationInJointSpace);
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
            }
        }

    }

    internal void remove_joint_limits(ArticulationBody[] bodies)
    {
        for(int i = 1; i < 23; i++)
        {
            var body = bodies[i];
            if (body.isRoot || !(body.jointType != ArticulationJointType.SphericalJoint))
                continue;
            body.twistLock = ArticulationDofLock.FreeMotion;
            body.swingYLock = ArticulationDofLock.FreeMotion;
            body.swingZLock = ArticulationDofLock.FreeMotion;
        }
    }
    [ContextMenu("Setup art bodies")]
    void setup_art_bodies()
    {
        for (int i = 0; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            if (!apply_all_local_rots && !bones_to_apply.Contains(bone))
                continue;
            ArticulationBody ab = boneToTransform[i].GetComponent<ArticulationBody>();
            bone_to_art_body[i] = ab;
            if (ab == null)
                continue;
            ab.SetAllDriveStiffness(stiffness);
            ab.SetAllDriveDamping(damping);
            ab.SetAllForceLimit(force_limit);
        }
        // we need to make sure the body starts the right angles, because after this we will only be applying 
        // velocities

    }
    [ContextMenu("Find max rotations for each dimension for a bone")]
    private void find_rot_limits()
    {
        motionDB = database.Instance;
        //motionDB = new database(Application.dataPath + @"/outputs/database.bin" );
        int num_frames = motionDB.nframes();
        int j = (int)debug_bone;
        ArticulationBody ab = boneToTransform[j].GetComponent<ArticulationBody>();
        float min_x, min_y, min_z;
        min_x = min_y = min_z = float.PositiveInfinity;
        float max_x, max_y, max_z;
        max_x = max_y = max_z = float.NegativeInfinity;
        for (int i = 0; i < num_frames; i++)
        {
            Quaternion debug_bone_rot = motionDB.bone_rotations[i][j];
            Vector3 target_rot = ab.ToTargetRotationInReducedSpace(debug_bone_rot);
            //Debug.Log(target_rot.ToString("f6"));
            min_x = Mathf.Min(min_x, target_rot.x);
            min_y = Mathf.Min(min_y, target_rot.y);
            min_z = Mathf.Min(min_z, target_rot.z);

            max_x = Mathf.Max(max_x, target_rot.x);
            max_y = Mathf.Max(max_y, target_rot.y);
            max_z = Mathf.Max(max_z, target_rot.z);
        }
        Debug.Log($"Mins: x: {min_x} , y: {min_y} , z: {min_z}");

        Debug.Log($"Maxess: x: {max_x} , y: {max_y} , z: {max_z}");
    }
    // Useful for normalizing inputs 
    [ContextMenu("Find mins and maxes for velocities and positions")]
        public void min_max_debugger()
    {
        int num_state_bones = MLAgentsDirector.state_bones.Length;
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
            Debug.Log($"{j} bone pos min: {bone_pos_mins[j].ToString("f6")}");
            Debug.Log($"{j} bone pos max: {bone_pos_maxes[j].ToString("f6")}");
            Debug.Log($"{j} bone vel min: {bone_vel_mins[j].ToString("f6")}");
            Debug.Log($"{j} bone vel max: {bone_vel_maxes[j].ToString("f6")}");

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
        database motionDB = database.Instance;
        //database motionDB = new database(Application.dataPath + @"/outputs/database.bin");
        int num_frames = motionDB.nframes();
        Vector3 last_cm = Vector3.zero;
        Vector3[] global_pos = new Vector3[23];
        Quaternion[] global_rots = new Quaternion[23];
        int num_state_bones = MLAgentsDirector.state_bones.Length;
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
            forward_kinematics_full(motionDB, i, ref global_pos, ref global_rots);
            //apply_global_pos_and_rot(global_pos, global_rots, boneToTransform);
            Vector3 cm = get_cm(boneToTransform, global_pos);
            Vector3 cm_vel = (cm - last_cm) / frame_time;
            cm_vel = Utils.quat_inv_mul_vec3(global_rots[0], cm_vel);
            // probably a glitch
            if (cm_vel.magnitude < 10f)
                updated_mins_and_maxes(cm_vel, ref cm_vel_min, ref cm_vel_max);

            last_cm = cm;
            for (int j = 0; j < num_state_bones; j++)
            {
                // Get state bone pose
                Vector3 bone_pos = global_pos[(int)MLAgentsDirector.state_bones[j]];
                // Resolve in reference frame (multiply by inverse of root rotation and subtract root position)
                Vector3 local_bone_pos = Utils.quat_inv_mul_vec3(global_rots[0], bone_pos - cm);
                Vector3 bone_vel = (bone_pos - state_bone_pos[j]) / frame_time;
                bone_vel = Utils.quat_inv_mul_vec3(global_rots[0], bone_vel);
                // Compare min maxes
                updated_mins_and_maxes(local_bone_pos,ref  bone_pos_mins[j], ref bone_pos_maxes[j]);
                updated_mins_and_maxes(bone_vel, ref bone_vel_mins[j], ref bone_vel_maxes[j]);
                state_bone_pos[j] = bone_pos;
            }
        }

    }

    public static Vector3 get_cm(Transform[] bone_to_transform, Vector3[] global_bone_positions)
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
            CoM += mass * global_bone_positions[i];
            total_mass += mass;
        }
        return CoM / total_mass;
    }

    private static void updated_mins_and_maxes(Vector3 val, ref Vector3 mins, ref Vector3 maxes)
    {
        maxes = update_maxes(maxes, val);
        mins = update_mins(mins, val);
    }

    private static Vector3 update_maxes(Vector3 vel_max, Vector3 cm_vel)
    {
        return new Vector3(Mathf.Max(vel_max.x, cm_vel.x), Mathf.Max(vel_max.y, cm_vel.y),Mathf.Max(vel_max.z, cm_vel.z));
    }

    private static Vector3 update_mins(Vector3 vel_min, Vector3 cm_vel)
    {
        return new Vector3(Mathf.Min(vel_min.x, cm_vel.x), Mathf.Min(vel_min.y, cm_vel.y), Mathf.Min(vel_min.z, cm_vel.z));
    }

    private static void forward_kinematics_full(
     database motionDB,
     int frame,
     ref Vector3[] global_bone_positions,
     ref Quaternion[] global_bone_rotations
    )
    {
        int[] bone_parents = motionDB.bone_parents;

        for (int i = 0; i < bone_parents.Length; i++)
        {
            // Assumes bones are always sorted from root onwards
            if (bone_parents[i] == -1)
            {
                global_bone_positions[i] = motionDB.bone_positions[frame][i];
                global_bone_rotations[i] = motionDB.bone_rotations[frame][i];
            }
            else
            {
                Vector3 parent_position = global_bone_positions[bone_parents[i]];
                Quaternion parent_rotation = global_bone_rotations[bone_parents[i]];
                global_bone_positions[i] = Utils.quat_mul_vec3(parent_rotation, motionDB.bone_positions[frame][i]) + parent_position;
                global_bone_rotations[i] = parent_rotation * motionDB.bone_rotations[frame][i];
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
