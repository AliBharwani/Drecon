using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class mm_v2 : MonoBehaviour
{

    enum Bones
    {
        Bone_Entity = 0,
        Bone_Hips = 1,
        Bone_LeftUpLeg = 2,
        Bone_LeftLeg = 3,
        Bone_LeftFoot = 4,
        Bone_LeftToe = 5,
        Bone_RightUpLeg = 6,
        Bone_RightLeg = 7,
        Bone_RightFoot = 8,
        Bone_RightToe = 9,
        Bone_Spine = 10,
        Bone_Spine1 = 11,
        Bone_Spine2 = 12,
        Bone_Neck = 13,
        Bone_Head = 14,
        Bone_LeftShoulder = 15,
        Bone_LeftArm = 16,
        Bone_LeftForeArm = 17,
        Bone_LeftHand = 18,
        Bone_RightShoulder = 19,
        Bone_RightArm = 20,
        Bone_RightForeArm = 21,
        Bone_RightHand = 22,
    };

    public string databaseFilepath = "database_v1";
    public Transform[] boneToTransform = new Transform[21];
    database motionDB;
    int frameIdx = 0;

    Vector3[] global_bone_positions;
    Vector3[] global_bone_velocities;
    Quaternion[] global_bone_rotations;
    Vector3[] global_bone_angular_velocities;

    Vector3[] local_bone_positions;
    Quaternion[] local_bone_rotations;
    int[] bone_parents;

    void Start()
    {
        if (Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        Application.targetFrameRate = 30;
        motionDB = new database(Application.dataPath + @"/outputs/" + databaseFilepath + ".bin");
        global_bone_positions = new Vector3[motionDB.nbones()];
        global_bone_velocities = new Vector3[motionDB.nbones()];
        global_bone_rotations = new Quaternion[motionDB.nbones()];
        global_bone_angular_velocities = new Vector3[motionDB.nbones()];

        local_bone_positions = new Vector3[motionDB.nbones()];
        local_bone_rotations = new Quaternion[motionDB.nbones()];
        bone_parents = motionDB.bone_parents;
        //int walkFrameIdx = (351 - 194) + (7086 - 90);
        //frameIdx = walkFrameIdx;
    }

    void Update()
    {
        motionDB.setDataToFrame(ref local_bone_positions, ref local_bone_rotations, frameIdx);
        playFrameIdx();
        frameIdx++;
    }

    private void playFrameIdx()
    {
        forward_kinematics_full();
        apply_global_pos_and_rot();
    }

    // Compute forward kinematics for all joints
    private void forward_kinematics_full(
        //in Vector3[] local_bone_positions,
        //in Quaternion[] local_bone_rotations,
        //in int[] bone_parents
        )
    {
        for (int i = 0; i < bone_parents.Length; i++)
        {
            // Assumes bones are always sorted from root onwards
            Assert.IsTrue(bone_parents[i] < i);

            if (bone_parents[i] == -1)
            {
                global_bone_positions[i] = local_bone_positions[i];
                global_bone_rotations[i] = local_bone_rotations[i];
            }
            else
            {
                Vector3 parent_position = global_bone_positions[bone_parents[i]];
                Quaternion parent_rotation = global_bone_rotations[bone_parents[i]];
                global_bone_positions[i] = quat_mul_vec3(parent_rotation, local_bone_positions[i]) + parent_position;
                global_bone_rotations[i] = quat_mul(parent_rotation, local_bone_rotations[i]);
            }
        }
    }

    private void apply_global_pos_and_rot()
    {
        //Debug.Log(motionDB.nbones()); // 23
        for(int i = 0; i < motionDB.nbones(); i++)
        {
            Transform t = boneToTransform[i];
            if (i < 2)
            {
                //Quaternion rot = local_bone_rotations[i];
                //Debug.Log($"Frame {frameIdx}: {rot.w}, {rot.x} , {rot.y} , {rot.z}");
                //t.localPosition = local_bone_positions[i];
                //t.position = global_bone_positions[i];
            }
            t.localRotation = local_bone_rotations[i];
            //t.localRotation = Utils.to_unity_rot(local_bone_rotations[i]);
            //t.position = global_bone_positions[i];
            //t.rotation = global_bone_rotations[i];
        }
    }

    private Vector3 quat_mul_vec3(Quaternion q, Vector3 v)
    {
        Vector3 t = 2.0f * Vector3.Cross(new Vector3(q.x, q.y, q.z), v);
        return v + q.w * t + Vector3.Cross(new Vector3(q.x, q.y, q.z), t);
    }

    private Quaternion quat_mul(Quaternion q, Quaternion v)
    {
        return q * v;
    }
}
