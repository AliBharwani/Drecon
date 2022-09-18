using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class BVHJointTester : MonoBehaviour
{
    public bool apply_all_local_rots;
    public bool set_art_bodies;
    public bool set_target_velocities;
    public int frameIdx = 500;
    public mm_v2.Bones[] bones_to_apply;
    public mm_v2.Bones debug_bone;
    public Transform[] boneToTransform = new Transform[23];
    private ArticulationBody[] bone_to_art_body = new ArticulationBody[23];
    public int start_delay = 60;
    Gamepad gamepad;
    database motionDB;
    public float stiffness = 120f;
    public float damping = 3f;

    void Start()
    {
        if (Application.isEditor)
            UnityEditor.EditorWindow.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        gamepad = Gamepad.current;
        Application.targetFrameRate = 30;
        motionDB = new database(Application.dataPath + @"/outputs/database.bin", 1, true, 10, 10);
        for (int i = 0; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            if (!bones_to_apply.Contains(bone))
                continue;
            ArticulationBody ab = boneToTransform[i].GetComponent<ArticulationBody>();
            bone_to_art_body[i] = ab;
            if (ab == null)
                continue;
            if (set_target_velocities)
            {
                ab.SetAllDriveStiffness(1f);
                //ab.SetAllDriveDamping(0f);
                Quaternion[] curr_bone_rotations = motionDB.bone_rotations[frameIdx];
                ab.SetDriveRotation(curr_bone_rotations[i]);

            }
            else {    
                ab.SetAllDriveStiffness(stiffness);
                ab.SetAllDriveDamping(damping);
            }
            
        
        }
        // we need to make sure the body starts the right angles, because after this we will only be applying 
        // velocities

    }

    void Update()
    {
        if (start_delay > 0)
        {
            if (start_delay == 1 && set_target_velocities)
            {
                for (int i = 0; i < 23; i++)
                {
                    mm_v2.Bones bone = (mm_v2.Bones)i;
                    if (!bones_to_apply.Contains(bone))
                        continue;
                    ArticulationBody ab = bone_to_art_body[i];
                    if (ab == null)
                        continue;
                    ab.SetAllDriveStiffness(0f);
                    ab.SetAllDriveDamping(1f);
                }
            }
            start_delay--;
            return;
        }
        frameIdx++;
        playFrameIdx();
    }

    private void playFrameIdx()
    {
        Vector3[] curr_bone_positions = motionDB.bone_positions[frameIdx];
        Quaternion[] curr_bone_rotations = motionDB.bone_rotations[frameIdx];
        Vector3[] cur_angular_vel = motionDB.bone_angular_velocities[frameIdx];
        for (int i = 0; i < 23; i++)
        {
            mm_v2.Bones bone = (mm_v2.Bones)i;
            if (!apply_all_local_rots && !bones_to_apply.Contains(bone))
                continue;
            Transform t = boneToTransform[i];
            Quaternion local_rot = curr_bone_rotations[i];
            if (!set_art_bodies)
            {
                t.localPosition = curr_bone_positions[i];
                t.localRotation = local_rot;
                continue;
            }
            ArticulationBody ab = bone_to_art_body[i];
            if (ab == null || !bones_to_apply.Contains(bone))
                continue;
            bool print_debug = bone == debug_bone;
            //if (print_debug)
                //if (set_target_velocities)
                    //Debug.Log($"Local target velocity: {cur_angular_vel[i].ToString("f6")}");
                //else
                    //Debug.Log($"Local rot: {local_rot.ToString("f6")}");

            if (set_target_velocities)
            {
                Vector3 scaled_angle_axis_vel = cur_angular_vel[i];
                float angle = scaled_angle_axis_vel.magnitude;
                Vector3 axis = scaled_angle_axis_vel.normalized;
                //Debug.Log($"Original angle: {angle} , axis: {axis.ToString("f6")}");
                Quaternion q = Quaternion.AngleAxis(angle, axis);// * Quaternion.Inverse(ab.anchorRotation);
                Quaternion q2 =  Quaternion.AngleAxis(angle, axis)* Quaternion.Inverse(ab.anchorRotation);

                Vector3 rot_in_reduced_space = ab.ToTargetRotationInReducedSpace(q);
                Vector3 final_vel =  q.ToEulerAnglesInRange180();  // rot_in_reduced_space * 30; //  multiply by 30 because 30fps
                if (set_target_velocities) {
                    //Debug.Log($"Local target velocity: {rot_in_reduced_space.ToString("f6")}");
                    Debug.Log($"Joint velocity: {ab.jointVelocity[0]} , {ab.jointVelocity[1]} , {ab.jointVelocity[2]}");
                }
                ab.SetDriveTargetVelocity(final_vel, print_debug);
            }
            else
                ab.SetDriveRotation(local_rot, print_debug);

        }

    }

}
