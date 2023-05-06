using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public static class ArtBodyUtils
{

    public static float geodesicBetweenTwoRotationMatrices(Matrix4x4 a, Matrix4x4 b, bool inDeg = false)
    {
        //Matrix4x4 kinRotation = Matrix4x4.Rotate(kinBone.localRotation);
        //Matrix4x4 simRotation = Matrix4x4.Rotate(simBone.localRotation);
        Matrix4x4 lossMat = (b * a.transpose);
        float trace = lossMat[0, 0] + lossMat[1, 1] + lossMat[2, 2];
        // Need clamping because Acos will throw NAN for values outside [-1, 1]
        float traceClamped = Mathf.Clamp((trace - 1) / 2, -1f, 1f);
        return Mathf.Acos(traceClamped) * (inDeg ? Mathf.Rad2Deg : 1f);
    }
    public static void deconstructScaledAngleAxis(this Vector3 angle_axis, out float angle, out Vector3 axis)
    {
        angle = angle_axis.magnitude;
        axis = angle_axis.normalized;
    }
    public static Matrix4x4 MatrixFrom6DRepresentation(Vector3 v1, Vector3 v2)
    {
        // Apply Gram-Schmidt process treating v1 - v2 as columns of new rotation matrix
        Vector3 e1 = v1.normalized;
        Vector3 u2 = v2 - (Vector3.Dot(e1, v2) * e1);
        Vector3 e2 = u2.normalized;
        Vector3 e3 = Vector3.Cross(e1, e2);
        //float qw = Mathf.Sqrt(1f + m00 + m11 + m22) / 2f;
        //float qx = (m21 - m12) / (4f * qw);
        //float qy = (m02 - m20) / (4f * qw);
        //float qz = (m10 - m01) / (4f * qw);

        Matrix4x4 rotMatrix = Matrix4x4.identity;
        rotMatrix[0,0] = e1.x;
        rotMatrix[1, 0] = e1.y;
        rotMatrix[2, 0] = e1.z;

        rotMatrix[0,1] = e2.x;
        rotMatrix[1, 1] = e2.y;
        rotMatrix[2, 1] = e2.z;

        rotMatrix[0, 2] = e3.x;
        rotMatrix[1, 2] = e3.y;
        rotMatrix[2, 2] = e3.z;
        return rotMatrix;
    }

    public static Quaternion From6DRepresentation(Vector3 v1, Vector3 v2, ref Matrix4x4 adjustmentMat, bool useAdjustmentMat)
    {
        // Apply Gram-Schmidt process treating v1 - v2 as columns of new rotation matrix
        Vector3 e1 = v1.normalized;
        Vector3 u2 = v2 - (Vector3.Dot(e1, v2) * e1);
        Vector3 e2 = u2.normalized;
        Vector3 e3 = Vector3.Cross(e1, e2);
        //float qw = Mathf.Sqrt(1f + m00 + m11 + m22) / 2f;
        //float qx = (m21 - m12) / (4f * qw);
        //float qy = (m02 - m20) / (4f * qw);
        //float qz = (m10 - m01) / (4f * qw);
        
        float qw, qx, qy, qz;
        Matrix4x4 mat = Matrix4x4.identity;
        mat[0, 0] = e1.x;
        mat[1, 0] = e1.y;
        mat[2, 0] = e1.z;

        mat[0, 1] = e2.x;
        mat[1, 1] = e2.y;
        mat[2, 1] = e2.z;
        //mlagents - learn Assets\config\Drecon_more_layers.yaml--env = "Builds\Drecon"--run - id = AxisAngleAngleRenormalized--time - scale 1--capture - frame - rate = 0--no - graphics--num - env = 12
        //mlagents - learn Assets\config\Drecon_6d.yaml--run - id = 6DTest--time - scale 1--capture - frame - rate = 0--force
        mat[0, 2] = e3.x;
        mat[1, 2] = e3.y;
        mat[2, 2] = e3.z;
        if (adjustmentMat != null && useAdjustmentMat)
        {
            //Debug.Log($"Geodesic between identity and unadjusted: {geodesicBetweenTwoRotationMatrices(Matrix4x4.identity, mat, true)} adjusted: {geodesicBetweenTwoRotationMatrices(Matrix4x4.identity, mat * adjustmentMat, true)} v1: {v1} v2: {v2} e1: {e1} e2: {e2} e3: {e3} ");
            mat = mat * adjustmentMat;
        }

        float m00 = mat[0, 0];
        float m10 = mat[1, 0];
        float m20 = mat[2, 0];

        float m01 = mat[0, 1];
        float m11 = mat[1, 1];
        float m21 = mat[2, 1];

        float m02 = mat[0, 2];
        float m12 = mat[1, 2];
        float m22 = mat[2, 2];
        float tr = m00 + m11 + m22;
        //Quaternion q;
        //float t;
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        if (tr > 0)
        {
            float S = Mathf.Sqrt(tr + 1.0f) * 2; // S=4*qw 
            qw = 0.25f * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S;
            qz = (m10 - m01) / S;
        }
        else if ((m00 > m11) & (m00 > m22))
        {
            float S = Mathf.Sqrt(1.0f + m00 - m11 - m22) * 2; // S=4*qx 
            qw = (m21 - m12) / S;
            qx = 0.25f * S;
            qy = (m01 + m10) / S;
            qz = (m02 + m20) / S;
        }
        else if (m11 > m22)
        {
            float S = Mathf.Sqrt(1.0f + m11 - m00 - m22) * 2; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S;
            qy = 0.25f * S;
            qz = (m12 + m21) / S;
        }
        else
        {
            float S = Mathf.Sqrt(1.0f + m22 - m00 - m11) * 2; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25f * S;
        }
        Quaternion q = new Quaternion(qx, qy, qz, qw);
        return q.normalized;
    }
    public static Matrix4x4 From6DRepresentation(Vector3 v1, Vector3 v2)
    {
        // Apply Gram-Schmidt process treating v1 - v2 as columns of new rotation matrix
        Vector3 e1 = v1.normalized;
        Vector3 u2 = v2 - (Vector3.Dot(e1, v2) * e1);
        Vector3 e2 = u2.normalized;
        Vector3 e3 = Vector3.Cross(e1, e2);
        //float qw = Mathf.Sqrt(1f + m00 + m11 + m22) / 2f;
        //float qx = (m21 - m12) / (4f * qw);
        //float qy = (m02 - m20) / (4f * qw);
        //float qz = (m10 - m01) / (4f * qw);

        //float qw, qx, qy, qz;
        Matrix4x4 mat = Matrix4x4.identity;
        mat[0, 0] = e1.x;
        mat[1, 0] = e1.y;
        mat[2, 0] = e1.z;

        mat[0, 1] = e2.x;
        mat[1, 1] = e2.y;
        mat[2, 1] = e2.z;
        //mlagents - learn Assets\config\Drecon_more_layers.yaml--env = "Builds\Drecon"--run - id = AxisAngleAngleRenormalized--time - scale 1--capture - frame - rate = 0--no - graphics--num - env = 12
        //mlagents - learn Assets\config\Drecon_6d.yaml--run - id = 6DTest--time - scale 1--capture - frame - rate = 0--force
        mat[0, 2] = e3.x;
        mat[1, 2] = e3.y;
        mat[2, 2] = e3.z;
        return mat;
    }
    public static void SetDriveTargetVelocity(this ArticulationBody body, Quaternion targetLocalVel, bool debug = false)
    {
        /* Angular velocity appears in scaled angle axis representation 
         * 
         */
        Vector3 target_vel = body.ToTargetRotationInReducedSpace(targetLocalVel, true);

        //body.angularVelocity = target_vel;
        //return;
        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.targetVelocity = target_vel.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.targetVelocity = target_vel.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.targetVelocity = target_vel.z;
        body.zDrive = zDrive;
    }

    public static void SetDriveTargetVelocity(this ArticulationBody body, Vector3 targetAngularVel)
    {
        /* Angular velocity appears in scaled angle axis representation 
         * 
         */
        //Vector3 target_vel = body.ToTargetRotationInReducedSpace(targetLocalVel, true);

        //body.angularVelocity = target_vel;
        //return;
        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.targetVelocity = targetAngularVel.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.targetVelocity = targetAngularVel.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.targetVelocity = targetAngularVel.z;
        body.zDrive = zDrive;
    }

    public static void SetDriveRotation(this ArticulationBody body, Quaternion targetLocalRotation, bool debug = false)
    {
        Vector3 target = body.ToTargetRotationInReducedSpace(targetLocalRotation, true);
        //if (debug)
        //Debug.Log($"{body.transform.name} Target rot: {target.ToString("f6")}");


        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.target = target.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.target = target.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.target = target.z;
        body.zDrive = zDrive;
    }

    /// <summary>
    /// Assigns articulation body joint drive target rotations for the entire hierarchy of bodies.
    /// </summary>
    /// <param name="bodies"> array of hierarchy of bodies to apply rotations to </param>
    /// <param name="targetTransforms"> array of transforms these art bodies try to mimic </param>
    /// <param name="startIndexes"> is obtained by calling articulationBody.GetDofStartIndices(startIndecies) </param> 
    /// <param name="driveTargets"> is obtained by calling articulationBody.GetDriveTargets(driveTargets) </param>
    public static void SetDriveRotations(ref ArticulationBody[] bodies, ref Transform[] targetTransforms, Quaternion[] startingRotations, ref List<int> startIndexes, ref List<float> driveTargets)
    {
        mm_v2.Bones[] testBones = { mm_v2.Bones.Bone_Hips, mm_v2.Bones.Bone_RightUpLeg };
        for (int i = 0; i < bodies.Length; i++)
        {
            if (bodies[i].isRoot )//|| !testBones.Contains((mm_v2.Bones)i)) 
                continue;
            int j = bodies[i].index;
            int index = startIndexes[j];
            //Debug.Log($"{bodies[i].transform.name} has index {index}");
            if (bodies[i].transform.name != targetTransforms[i].name)
                Debug.Log($"Warning: {bodies[i].transform.name} != {targetTransforms[i].name }");

            bool rotateX = bodies[i].twistLock != ArticulationDofLock.LockedMotion;
            bool rotateY = bodies[i].swingYLock != ArticulationDofLock.LockedMotion;
            bool rotateZ = bodies[i].swingZLock != ArticulationDofLock.LockedMotion;

            Vector3 targets = bodies[i].ToTargetRotationInReducedSpace(targetTransforms[i].localRotation, true);
            //Vector3 targets = bodies[i].ToTargetRotationFromStartingRotation(targetTransforms[i].localRotation, startingRotations[i]);
            if (testBones.Contains((mm_v2.Bones)i)) {
                Debug.Log($"{bodies[i].transform.name} target is: {targets} in radians: {targets * Mathf.Deg2Rad}");
            }
            //Vector3 targets = bodies[i].ToTargetRotationInReducedSpace_OLD_DO_NOT_USE(targetTransforms[i].localRotation);
            //Debug.Log($"Setting {bodies[i].transform.name} to: {targets * Mathf.Deg2Rad}");

            int dofIndex = 0;

            if (rotateX)
            {
                driveTargets[index + dofIndex] = targets.x * Mathf.Deg2Rad;
                dofIndex++;
            }
            if (rotateY)
            {
                driveTargets[index + dofIndex] = targets.y * Mathf.Deg2Rad;
                dofIndex++;
            }
            if (rotateZ)
            {
                driveTargets[index + dofIndex] = targets.z * Mathf.Deg2Rad;
                dofIndex++;
            }
         }

        bodies[0].SetDriveTargets(driveTargets);
    }


    /*
     * F = stiffness * (currentPosition - target) - damping * (currentVelocity - targetVelocity).
     * In this formula, currentPosition and currentVelocity are linear position and linear velocity in case of the linear drive. 
     * In case of the rotational drive, currentPosition and currentVelocity correspond to the angle and angular velocity respectively.
     */
    public static void SetAllDriveStiffness(this ArticulationBody body, float stiffness)
    {

       // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.stiffness = stiffness;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.stiffness = stiffness;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.stiffness = stiffness;
        body.zDrive = zDrive;
    }

    public static void SetAllDriveStiffness(this ArticulationBody body, Vector3 stiffness)
    {

        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.stiffness = stiffness.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.stiffness = stiffness.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.stiffness = stiffness.z;
        body.zDrive = zDrive;
    }
    public static void SetAllDriveDamping(this ArticulationBody body, float damping)
    {

        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.damping = damping;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.damping = damping;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.damping = damping;
        body.zDrive = zDrive;
    }
    public static void SetAllForceLimit(this ArticulationBody body, float forceLimit)
    {
        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.forceLimit = forceLimit;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.forceLimit = forceLimit;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.forceLimit = forceLimit;
        body.zDrive = zDrive;
    }
    /// <summary>
    /// Converts targetLocalRotation into reduced space of this articulation body.
    /// </summary>
    /// <param name="body"> ArticulationBody to apply rotation to </param>
    /// <param name="targetLocalRotation"> target's local rotation this articulation body is trying to mimic </param>
    /// <returns></returns>

    public static Vector3 ToTargetRotationInReducedSpace_OLD_DO_NOT_USE(this ArticulationBody body, Quaternion targetLocalRotation)
    {
        if (body.isRoot)
            return Vector3.zero;
        Vector3 axis;
        float angle;

        //Convert rotation to angle-axis representation (angles in degrees)
        targetLocalRotation.ToAngleAxis(out angle, out axis);

        //Debug.Log($"Re extracted angle: {angle} , axis: {axis.ToString("f6")}");
        // Converts into reduced coordinates and combines rotations (anchor rotation and target rotation)
        Vector3 rotInReducedSpace = Quaternion.Inverse(body.anchorRotation) * axis * angle;

        return rotInReducedSpace;
    }
    public static Vector3 ToTargetRotationFromStartingRotation(this ArticulationBody body, Quaternion targetLocalRotation, Quaternion startingRotation)
    {
        if (body.isRoot)
            return Vector3.zero;

        // Need C s.t. A * C = B
        Quaternion diff = startingRotation * Quaternion.Inverse(targetLocalRotation) ;
        return body.ToTargetRotationInReducedSpace(diff, true);
        //Vector3 axis;
        //float angle;

        ////Convert rotation to angle-axis representation (angles in degrees)
        //targetLocalRotation.ToAngleAxis(out angle, out axis);

        ////Debug.Log($"Re extracted angle: {angle} , axis: {axis.ToString("f6")}");
        //// Converts into reduced coordinates and combines rotations (anchor rotation and target rotation)
        //Vector3 rotInReducedSpace = Quaternion.Inverse(body.anchorRotation) * axis * angle;

        //return rotInReducedSpace;
    }


    public static Vector3 ToTargetRotationInReducedSpace(this ArticulationBody body, Quaternion targetLocalRotation, bool inDegrees)
    {
        if (body.isRoot)
            return Vector3.zero;
        Quaternion q = Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation;
        q.Normalize();
        Vector3 TargetRotationInJointSpace = -q.eulerAngles;
        //Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation).normalized.eulerAngles;
        //TargetRotationInJointSpace = targetLocalRotation.eulerAngles;
        TargetRotationInJointSpace = new Vector3(
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.x),
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.y),
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.z));
        return inDegrees ? TargetRotationInJointSpace : TargetRotationInJointSpace * Mathf.Deg2Rad;
    }

    public static Vector3 ToEulerAnglesInRange180(this Quaternion q)
    {
        return new Vector3()
        {
            x = WrapAngle(q.eulerAngles.x),
            y = WrapAngle(q.eulerAngles.y),
            z = WrapAngle(q.eulerAngles.z)
        };

        float WrapAngle(float angle)
        {
            angle %= 360;
            return angle > 180 ? angle - 360 : angle;
        }
    }
    public static void resetJointPosition(this ArticulationBody body, Vector3 newJointPositions, bool resetEverything = true)
    {
        if (body.jointType != ArticulationJointType.SphericalJoint)
            throw new System.Exception("Attempting to reset joint phyiscs with Vector3 on non spherical articulation body: " + body.gameObject.name);
        body.jointPosition = new ArticulationReducedSpace(newJointPositions.x, newJointPositions.y, newJointPositions.z);
        if (!resetEverything)
            return;
        body.jointAcceleration = new ArticulationReducedSpace(0f, 0f, 0f);
        body.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
        body.jointForce = new ArticulationReducedSpace(0f, 0f, 0f);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }
    public static void resetJointPosition(this ArticulationBody body, float newJointPosition, bool resetEverything = true)
    {
        if (body.dofCount != 1)
            throw new System.Exception($"Attempting to reset joint phyiscs with float on non articulation body with != 1 DOF: DOF: {body.dofCount} name: {body.gameObject.name}");
        body.jointPosition = new ArticulationReducedSpace(newJointPosition);
        if (!resetEverything)
            return;
        body.jointAcceleration = new ArticulationReducedSpace(0);
        body.jointVelocity = new ArticulationReducedSpace(0);
        body.jointForce = new ArticulationReducedSpace(0);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }

    public static void resetJointPhysics(this ArticulationBody body)
    {
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }

}
