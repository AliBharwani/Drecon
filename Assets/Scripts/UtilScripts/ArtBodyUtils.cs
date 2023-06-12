using UnityEngine;

public static class ArtBodyUtils
{

    public static void SetDriveTargetVelocity(this ArticulationBody body, Vector3 targetAngularVel, Quaternion localRotation)
    {
        /* Angular velocity appears in scaled angle axis representation 
         * 
         */
        Quaternion targetAngularVelocityQ = Quaternion.AngleAxis(targetAngularVel.magnitude * Mathf.Rad2Deg * Time.fixedDeltaTime  , targetAngularVel.normalized);

        Vector3 degsPerSecond = (targetAngularVelocityQ * localRotation).ToEulerAnglesInRange180() - localRotation.ToEulerAnglesInRange180();
        degsPerSecond /= Time.fixedDeltaTime;
        //if (body.name == "Model:LeftUpLeg")
        //    Debug.Log($"DegsPerSecond: {degsPerSecond}");

        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.targetVelocity = degsPerSecond.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.targetVelocity = degsPerSecond.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.targetVelocity = degsPerSecond.z;
        body.zDrive = zDrive;
    }

    public static void SetDriveRotation(this ArticulationBody body, Quaternion targetLocalRotation, bool debug = false)
    {
        Vector3 target = body.ToTargetRotationInReducedSpace(targetLocalRotation, true);
        //if (debug)
        //Debug.Log($"{body.transform.name} Target rot: {target.ToString("f6
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
    public static void SetAllDriveDamping(this ArticulationBody body, Vector3 damping)
    {

        // assign to the drive targets...
        ArticulationDrive xDrive = body.xDrive;
        xDrive.damping = damping.x;
        body.xDrive = xDrive;

        ArticulationDrive yDrive = body.yDrive;
        yDrive.damping = damping.y;
        body.yDrive = yDrive;

        ArticulationDrive zDrive = body.zDrive;
        zDrive.damping = damping.z;
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

    public static Vector3 ToTargetRotationInReducedSpace(this ArticulationBody body, Quaternion targetLocalRotation, bool inDegrees)
    {
        if (body.isRoot)
            return Vector3.zero;

        Quaternion q = Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation;
        q.Normalize();
        Vector3 TargetRotationInJointSpace = -q.eulerAngles;
        TargetRotationInJointSpace = new Vector3(
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.x),
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.y),
            Mathf.DeltaAngle(0, TargetRotationInJointSpace.z));
        return inDegrees ? TargetRotationInJointSpace : TargetRotationInJointSpace * Mathf.Deg2Rad;
    }
    public static Vector3 ToEulerAnglesInRange180(this Vector3 vec)
    {
        return new Vector3()
        {
            x = WrapAngle(vec.x),
            y = WrapAngle(vec.y),
            z = WrapAngle(vec.z)
        };

        float WrapAngle(float angle)
        {
            angle %= 360;
            return angle > 180 ? angle - 360 : angle;
        }
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
