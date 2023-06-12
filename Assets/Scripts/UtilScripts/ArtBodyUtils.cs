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

    // Apply Gram-Schmidt process treating v1 - v2 as columns of new rotation matrix
    public static void GramSchmidtNormalization(Vector3 v1, Vector3 v2, out Vector3 e1, out Vector3 e2, out Vector3 e3)
    {
        e1 = v1.normalized;
        Vector3 u2 = v2 - (Vector3.Dot(e1, v2) * e1);
        e2 = u2.normalized;
        e3 = Vector3.Cross(e1, e2);
    }
    public static Matrix4x4 MatrixFrom6DRepresentation(Vector3 v1, Vector3 v2)
    {
        GramSchmidtNormalization(v1, v2, out Vector3 e1, out Vector3 e2, out Vector3 e3);
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
    public static Quaternion RotateObjectWithOrthonormalVector(Vector3 v1, Vector3 v2)
    {
        GramSchmidtNormalization(v1, v2, out Vector3 e1, out _, out Vector3 e3);
        // Create quaternion from orthonormalized vectors
        Quaternion rotation = new Quaternion();
        rotation.SetFromToRotation(Vector3.right, e1);
        rotation *= Quaternion.FromToRotation(Vector3.up, e3);
        return rotation;
    }

    public static Quaternion QuatFrom6DRepresentation(Vector3 v1, Vector3 v2)
    {

        float qw, qx, qy, qz;
        Matrix4x4 mat = MatrixFrom6DRepresentation(v1, v2);
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
        // = Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation)  gives us the rotation that goes from the inverse of the targetLocalRot to the anchor rot
        // * body.parentAnchorRotation puts this rotation in the parent's joitn space
        // Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation --> applies the inverse of the targetLocalRotation to the parentAnchorRotation. Basically saying:
        // "What is the rotation that will take us from parentAnchorRotation to targetLocalRotation? 

        Quaternion q = Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation;
        //q = Quaternion.Inverse(targetLocalRotation);
        q.Normalize();
        Vector3 TargetRotationInJointSpace = -q.eulerAngles;
        //Vector3 TargetRotationInJointSpace = -(Quaternion.Inverse(body.anchorRotation) * Quaternion.Inverse(targetLocalRotation) * body.parentAnchorRotation).normalized.eulerAngles;
        //TargetRotationInJointSpace = Quaternion.Inverse(targetLocalRotation).eulerAngles;
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

    public static float GetYAngle(this Quaternion q, bool inDegrees = false)
    {
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
        return Mathf.Atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * Mathf.Pow(q.y,2) - 2 * Mathf.Pow(q.z,2)) * (inDegrees ? Mathf.Rad2Deg : 1);
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
