using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class ArtBodyUtils
{

    public static void deconstructScaledAngleAxis(Vector3 angle_axis, out float angle, out Vector3 axis)
    {
        angle = angle_axis.magnitude;
        axis = angle_axis.normalized;
    }
    public static void SetDriveTargetVelocity(this ArticulationBody body, Vector3 target_vel, bool debug = false)
    {
        /* Angular velocity appears in scaled angle axis representation 
         * 
         */
        if (debug)
            Debug.Log($"Target rot velocity: {target_vel.ToString("f6")}");

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
    public static void SetDriveRotation(this ArticulationBody body, Quaternion targetLocalRotation, bool debug = false)
    {
        Vector3 target = body.ToTargetRotationInReducedSpace(targetLocalRotation, true);
        if (debug)
            Debug.Log($"Target rot: {target.ToString("f6")}");


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
    public static void SetAllForceLimit(this ArticulationBody body, float damping)
    {
        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.forceLimit = 200;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.forceLimit = 200;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.forceLimit = 200;
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

    public static Vector3 ToTargetRotationInReducedSpace(this ArticulationBody body, Quaternion targetLocalRotation, bool inDegrees)
    {
        if (body.isRoot)
            return Vector3.zero;
        Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation).eulerAngles;
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
    public static void resetJointPosition(this ArticulationBody body, Vector3 newJointPositions)
    {
        if (body.jointType != ArticulationJointType.SphericalJoint)
            throw new System.Exception("Attempting to reset joint phyiscs with Vector3 on non spherical articulation body: " + body.gameObject.name);
        body.jointPosition = new ArticulationReducedSpace(newJointPositions.x, newJointPositions.y, newJointPositions.z);
        body.jointAcceleration = new ArticulationReducedSpace(0f, 0f, 0f);
        body.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
        body.jointForce = new ArticulationReducedSpace(0f, 0f, 0f);
        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }
    public static void resetJointPosition(this ArticulationBody body, float newJointPosition)
    {
        if (body.dofCount != 1)
            throw new System.Exception($"Attempting to reset joint phyiscs with float on non articulation body with != 1 DOF: DOF: {body.dofCount} name: {body.gameObject.name}");
        body.jointPosition = new ArticulationReducedSpace(newJointPosition);
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
