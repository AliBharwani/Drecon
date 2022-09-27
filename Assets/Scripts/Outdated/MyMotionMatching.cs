using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using UnityEngine.InputSystem;
using UnityEditor;

public class MyMotionMatching : MonoBehaviour
{
    [Header("==== SET ONCE AND FORGET ====")]
    public bool KeepSceneViewActive = true;
    public int targetFramerate = 30;
    public GameObject leftFoot;
    public GameObject rightFoot;
    public GameObject root;
    public GameObject hip;
    public bool walkOnly = true;
    public Transform toyPointer1, toyPointer2, toyPointer3;

    [Header("====    DEBUG    ====")]
    public bool applyMM = false;
    public bool drawGizmos = true;
    public bool drawAngles = false;
    public float gizmoSphereRad = .01f;
    public bool bruteforceSearch = false;
    public bool useGlobal = true;
    [Header("====   ANIMATION    ====")]
    public int updateEveryNFrame = 10;
    [Tooltip("# of frames in a transition between clips")]
    public float transitionTime = 3f;
    public int numNeigh = 1;
    public int animTransitionTolerance = 3;
    public bool useAnimTransforms = false;
    public bool lerpFromFristFrame = true;
    public float trajPenalty = 5f;
    public float velCombineFactor = .5f;
    public float a = .2f;
    public float b = .85f;
    public bool useInertializationBlending = true;
    [Header("====   PHYSICS    ====")]
    public bool useQuadraticVel = true;
    //public Vector3 acc;
    public float MoveSpeed = 2.0f;
    public float SprintSpeed = 5.335f;
    public float acceleration = 10.0f;
    public bool useSpringsForVel = true;
    public float halfLife = .25f;


    private Vector2 acc = new Vector2(5f, 5f);
    private Vector2 currentVel;

    private Vector3 hipRotOffset;
    private Vector3 lastLeftFootGlobalPos;
    private Vector3 lastRightFootGlobalPos;
    private Vector3 lastHipPos;
    private Quaternion lastHipQuat;
    private int searchVecLen;
    private KDTree motionDB;
    // private string pathToAnims = @"D:/Unity/Unity 2021 Editor Test/Assets/LAFLAN Data/Animations/";
    private float[] means;
    private float[] std_devs;
    private string[] prefixes;
    private string[] walkPrefixes = {
            "walk1_subject1",
            "walk1_subject2",
            "walk1_subject5",
            "walk3_subject1",
            "walk3_subject2",
        };
    private string[] allPrefixes = {
            "walk1_subject1",
            "walk1_subject2",
            "walk1_subject5",
            "walk3_subject1",
            "walk3_subject2",
            "run1_subject2",
            "run1_subject5",
            "run2_subject1",
            "sprint1_subject2",
        };
    private float frameTime = .03333f;
    private Gamepad gamepad;
    private float maxXVel; //= 4.92068f;
    private float maxZVel; // = 6.021712f;
    //private Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap;
    private int frameCounter = 0;
    private int nextFileIdx = -1;
    private int nextFrameIdx = -1;
    private int curFileIdx = -1;
    private int curFrameIdx = -1;
    private int curTransitionFrameNum = 1;
    private int lastMMFrameIdx = -1;
    private List<BVHParser.BVHBone>[] boneLists;
    Dictionary<string, Transform> nameToTransformMap;

    private bool firstFrame = true;
    // Debug stuff
    private Vector3 animDebugStart, animDebugEnd, inputDebugStart, inputDebugEnd, finalDebugStart, finalDebugEnd;
    private Vector3[] gizmoSpheres1;
    private Vector3[] gizmoSpheres2;
    private Vector3[] gizmoSpheres3;
    private string[] textLabels;

    private void loadBVHFiles()
    {
        //boneToTransformMap = new Dictionary<BVHParser.BVHBone, Transform>();
        boneLists = new List<BVHParser.BVHBone>[prefixes.Length];
        for (int i = 0; i < prefixes.Length; i++)
        {
            string prefix = prefixes[i];
            BVHParser bp = BVHUtils.parseFile(prefix + ".bvh");
            boneLists[i] = bp.boneList;
        }
        nameToTransformMap = BVHUtils.loadTransforms(transform);
    }
    private void ingestMotionMatchingDB()
    {
        // @ escapes the backslashes

        DateTime startTime = DateTime.Now;
        int counter = 0;
        string pathToData = @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/" + (useGlobal ? @"global/" : @"nonglobal/");
        pathToData += walkOnly ? @"walk_only/" : "" ;
        int j = 0;
        foreach (string line in File.ReadLines(pathToData + "stats.txt"))
        {
            if (j == 0)
            {
                if (!line.StartsWith("Means:"))
                {
                    throw new Exception("Parsing error at j " + j.ToString());
                }
            } else if (j == 1)
            {
                List<string> stringValues = line.Split(',').ToList();
                means = stringValues.Select(float.Parse).ToArray();
            } else if (j == 2)
            {
                if (!line.StartsWith("Std_Devs:"))
                {
                    throw new Exception("Parsing error at j " + j.ToString());
                }
            } else if (j == 3)
            {
                List<string> stringValues = line.Split(',').ToList();
                std_devs = stringValues.Select(float.Parse).ToArray();
            } else if (j == 4)
            {
                if (!line.StartsWith("Max X"))
                {
                    throw new Exception("Parsing error at j " + j.ToString());
                }
            } else if (j == 5)
            {
                List<string> stringValues = line.Split(',').ToList();
                maxXVel = float.Parse(stringValues[0]);
                maxZVel = float.Parse(stringValues[1]);
            }
            j++;
        }
        //Debug.Log("Means: " + string.Join(",", means));
        //Debug.Log("std_devs: " + string.Join(",", std_devs));
        //Debug.Log("maxXVel: " + maxXVel.ToString() + " maxZVel: " + maxZVel.ToString());
        for (int i = 0; i < prefixes.Length; i++)
        {
            string prefix = prefixes[i];
            foreach (string line in File.ReadLines(pathToData + prefix + "_normalized_outputs.txt"))
            {
                List<string> stringValues = line.Split(',').ToList();
                stringValues.Add(i.ToString());
                motionDB.Add(stringValues.Select(float.Parse).ToArray());
                counter++;
            }
        }
        DateTime ingestTime = DateTime.Now;
        Debug.Log("Counter: " + counter.ToString());
        motionDB.Build();
        DateTime buildTime = DateTime.Now;
        Debug.Log("Time to ingest: " + (ingestTime - startTime).Milliseconds);
        Debug.Log("Time to Build: " + (buildTime - ingestTime).Milliseconds);

    }

    private void normalizeVector(float[] vec)
    {
        for (int i = 0; i < searchVecLen; i++)
        {
            vec[i] = (vec[i] - means[i]) / std_devs[i];
        }
    }
    void OnDrawGizmos()
    {
        if (!drawGizmos || gizmoSpheres1 == null)
            return;
        // Draw a yellow sphere at the transform's position
        Gizmos.color = Color.blue;
        foreach (Vector3 spherePos in gizmoSpheres1)
        {
            Gizmos.DrawSphere(spherePos, gizmoSphereRad * 1.5f);
        }
        if (animDebugStart != null)
        {
            Gizmos.DrawLine(animDebugStart, animDebugEnd);
        }
        Gizmos.color = Color.red;
        foreach (Vector3 spherePos in gizmoSpheres2)
        {
            Gizmos.DrawSphere(spherePos, gizmoSphereRad);
        }
        if (inputDebugStart != null)
        {
            Gizmos.DrawLine(inputDebugStart, inputDebugEnd);
        }
        Gizmos.color = Color.green;
        if (finalDebugStart != null)
        {
            Gizmos.DrawLine(finalDebugStart, finalDebugEnd);
        }
        if (gizmoSpheres3.Length < 3)
        {
            return;
        }
        for (int i = 0; i < 3; i++)
        {
            Vector3 spherePos = gizmoSpheres3[i];
            Gizmos.DrawSphere(spherePos, gizmoSphereRad);
            if (drawAngles)
                Handles.Label(spherePos, textLabels[i]);
        }
    }
    private float[] getCurrentSearchVector()
    {
        if (firstFrame)
        {
            firstFrame = false;
            return new float[searchVecLen];
        }

        // get left and right foot local positions and global velocities
        // 2 pairs (left and right) of 2 vectors in r^3, 3 numbers
        Vector3 leftFootLocalPos = leftFoot.transform.position - hip.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - hip.transform.position;
        // velocity is the change in distance over time - if you went from 0m to 10m in 1 sec, your veloctiy is 10m/s 
        Vector3 leftFootGlobalVelocity = (leftFoot.transform.position - lastLeftFootGlobalPos) / frameTime;
        Vector3 rightFootGlobalVelocity = (rightFoot.transform.position - lastRightFootGlobalPos) / frameTime;

        // hip global velocity (one number in r3, 3 numbers)
        //Vector3 hipGlobalVelPerFrame = useAnimTransforms ? hip.transform.position - lastHipPos : currentVel * frameTime; // hip.transform.position - lastHipPos;

        // I should combine hipGlolabVel with user input
        //Vector3 hipGlobalVel = useAnimTransforms ? (hip.transform.position - lastHipPos) / frameTime : currentVel;// (hip.transform.position - lastHipPos) / frameTime;
        //Vector3 combinedHipGlobalVel = combineHipGlobalVel(hipGlobalVel);
        // based off bobsir's answer in https://forum.unity.com/threads/manually-calculate-angular-velocity-of-gameobject.289462/
        //Quaternion deltaRot = hip.transform.rotation * Quaternion.Inverse(lastHipQuat);
        //Vector3 eulerRot = new Vector3(Mathf.DeltaAngle(0, deltaRot.eulerAngles.x), Mathf.DeltaAngle(0, deltaRot.eulerAngles.y), Mathf.DeltaAngle(0, deltaRot.eulerAngles.z));

        //Vector3 hipAngularVelPerFrame = eulerRot;// / Time.fixedDeltaTime;
        //Vector3 hipAngularVel = (hip.transform.position - lastHipTransform.rotation.To) / frameTime;

        // (hip)trajectory positions and orientations  located at 20, 40, and 60 frames in the future which are projected onto the
        // groundplane(t-sub - i in R ^ 6, d - sub - i in R ^ 6, concatenating 3 xy pairs -> R ^ 6, total 12 numbers) 
        int additionalLen = 9;
        float[] hipFutureTrajAndOrientations = new float[additionalLen];
        int idx = 0;
        float[] userTraj = readUserInput();
        animDebugStart = hip.transform.position;

        Vector3 alternativeHipGlobalVel = Vector3.zero;
        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            //float futureXPos = hipGlobalVelPerFrame.x * frameNum;
            //float futureZPos = hipGlobalVelPerFrame.z * frameNum;
            float curAnimFutureXPos, curAnimFutureZPos;
            int fileIdxForTraj = nextFileIdx < 0 ? curFileIdx : nextFileIdx;
            int frameIdxForTraj = nextFrameIdx < 0 ? curFrameIdx : nextFrameIdx;
            BVHUtils.getTrajectoryNFramesFromNow(boneLists[fileIdxForTraj], frameIdxForTraj, frameNum ,out curAnimFutureXPos, out curAnimFutureZPos);
            if (i == 3)
            {
                animDebugEnd = new Vector3(curAnimFutureXPos, 0, curAnimFutureZPos) + hip.transform.position;
                // because 60 frames is 2 seconds from now, and we need vel per second
                float oneSecXPos, oneSecZPos;
                BVHUtils.getTrajectoryNFramesFromNow(boneLists[fileIdxForTraj], frameIdxForTraj, 30, out oneSecXPos, out oneSecZPos);
                alternativeHipGlobalVel = new Vector3(oneSecXPos, 0, oneSecZPos);
            }
            gizmoSpheres1[i - 1] = new Vector3(curAnimFutureXPos, 0, curAnimFutureZPos) + hip.transform.position;

            int startIdx = (i - 1) * 2;
            float futureXPos = combineCurTrajWithUser(curAnimFutureXPos, userTraj[startIdx], frameNum);
            float futureZPos = combineCurTrajWithUser(curAnimFutureZPos, userTraj[startIdx + 1], frameNum);
            gizmoSpheres3[i - 1] = new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;


            hipFutureTrajAndOrientations[idx] = futureXPos;
            hipFutureTrajAndOrientations[idx + 1] = futureZPos;

            //float futureYRot = hipAngularVelPerFrame.y * frameNum;
            float targetY = userInputTargetY();
            //float futureYRot = combineYRots(hip.transform.rotation.eulerAngles.y, targetY, frameNum);
            float futureYRot = useGlobal ? combineYRots(hip.transform.rotation.eulerAngles.y, targetY, frameNum) : BVHUtils.getDifferenceInYRots(hip.transform.rotation.eulerAngles.y, targetY);
            futureYRot *= i == 3 ? 1 : i == 2 ? b : a;
            if (drawAngles)
            {
                Transform toyPointer;
                if (i == 1) { toyPointer = toyPointer1; }
                else if (i == 2) { toyPointer = toyPointer2; }
                else { toyPointer = toyPointer3; }
                toyPointer.rotation = Quaternion.Euler(0, futureYRot, 270);
                toyPointer.position = hip.transform.position + new Vector3(futureXPos, 0f, futureZPos);

                textLabels[i - 1] = futureYRot.ToString();
            }

            hipFutureTrajAndOrientations[idx + 2] = futureYRot;

            idx += additionalLen / 3;
        }
        updateUserVelocity();
        Vector3 altCombinedHipGlobalVel = combineHipGlobalVel(alternativeHipGlobalVel);
        Vector3 velocitiesToUse = altCombinedHipGlobalVel;
        if (!useAnimTransforms)
        {
            velocitiesToUse = new Vector3(currentVel.x, 0f, currentVel.y);
        }
        float velocityMag;
        double velocityAngle;
        float angleBetweenVelHips = BVHUtils.getAngleBetweenVelocityAndHip(hip.transform, velocitiesToUse, out velocityMag, out velocityAngle);

        return useGlobal ? new float[] {
                leftFootLocalPos.x,
                leftFootLocalPos.y,
                leftFootLocalPos.z,
                rightFootLocalPos.x,
                rightFootLocalPos.y,
                rightFootLocalPos.z,
                leftFootGlobalVelocity.x,
                leftFootGlobalVelocity.y,
                leftFootGlobalVelocity.z,
                rightFootGlobalVelocity.x,
                rightFootGlobalVelocity.y,
                rightFootGlobalVelocity.z,
                velocitiesToUse.x,
                //velocitiesToUse.y,
                velocitiesToUse.z,
                 hip.transform.rotation.eulerAngles.y,
                hipFutureTrajAndOrientations[0],
                hipFutureTrajAndOrientations[1],
                hipFutureTrajAndOrientations[2],
                hipFutureTrajAndOrientations[3],
                hipFutureTrajAndOrientations[4],
                hipFutureTrajAndOrientations[5],
                hipFutureTrajAndOrientations[6],
                hipFutureTrajAndOrientations[7],
                hipFutureTrajAndOrientations[8],
        } : new float[] {
                leftFootLocalPos.x,
                leftFootLocalPos.y,
                leftFootLocalPos.z,
                rightFootLocalPos.x,
                rightFootLocalPos.y,
                rightFootLocalPos.z,
                leftFootGlobalVelocity.x,
                leftFootGlobalVelocity.y,
                leftFootGlobalVelocity.z,
                rightFootGlobalVelocity.x,
                rightFootGlobalVelocity.y,
                rightFootGlobalVelocity.z,
                velocityMag,
                angleBetweenVelHips,
                hipFutureTrajAndOrientations[0],
                hipFutureTrajAndOrientations[1],
                hipFutureTrajAndOrientations[2],
                hipFutureTrajAndOrientations[3],
                hipFutureTrajAndOrientations[4],
                hipFutureTrajAndOrientations[5],
                hipFutureTrajAndOrientations[6],
                hipFutureTrajAndOrientations[7],
                hipFutureTrajAndOrientations[8],
        };
    }


    private float combineCurTrajWithUser(float curTraj, float userTraj, int frameNum)
    {
        // values I like: (.2, .85); 

        //float a = 1;
        //float b = 1;
        if (frameNum == 60)
        {
            return userTraj;
        } else if (frameNum == 40)
        {
            return b * userTraj + (1 - b) * curTraj;
        } else if (frameNum == 20)
        {
            return a * userTraj + (1 - a) * curTraj;
        }
        throw new Exception("combineCurTrajWithUser called with invalid frameNum " + frameNum.ToString());
    }
    private float combineYRots(float curY, float userY, int frameNum)
    {
        // values I like: (.2, .85); 
        if (frameNum == 60)
        {
            return userY;
        }
        if (Mathf.Abs(curY - userY) <= 180)
        {
            float diff = curY - userY;
            if (frameNum == 40)
            {
                return curY - (diff * b);
            } else if (frameNum == 20)
            {
                return curY - (diff * a);
            }
        } else
        {
            // imagine curY points to 1oclock (60deg), and userY points to 4oclock (330deg) - want curY to drag back
            float diff;
            if (curY < userY)
            {
                diff = curY + (360 - userY);
                float val = frameNum == 40 ? curY - (diff * b) : curY - (diff * a);
                return val < 0 ? 360 + val : val;
            }
            else
            {
                diff = userY + (360 - curY);
                float val = frameNum == 40 ? curY + (diff * b) : curY + (diff * a);
                return val > 360 ? val - 360 : val;
            }
        }

        throw new Exception("combineCurTrajWithUser called with invalid frameNum " + frameNum.ToString());
    }

    private bool intsApproxSame(int a, int b)
    {
        return Math.Abs(a - b) <= animTransitionTolerance;
    }

    private float userInputTargetY()
    {
        Vector2 stickL = gamepad.leftStick.ReadValue();
        float angle = Mathf.Atan2(stickL.y, stickL.x) * Mathf.Rad2Deg * -1;
        angle = angle < 0f ? angle + 360 : angle;
        // Have to rotate 90 deg
        return angle;
        //double angle = BVHUtils.reverseAtan2ClockDirection(stickL.y, stickL.x);
        //return (float) angle;
    }
    private float[] readUserInput()
    {
        float[] userTraj;
        Vector2 desiredVel = getDesiredVelocity();
        inputDebugStart = hip.transform.position;
        if (useSpringsForVel)
        {
            Vector2[] px = new Vector2[3];
            Vector2[] pv = new Vector2[3];
            Vector2[] pa = new Vector2[3];
            SpringUtils.spring_character_predict(px, pv, pa, 3, Vector2.zero, currentVel, acc, desiredVel, halfLife, 20f * frameTime);
            userTraj = new float[6];
            for (int i = 0; i < 3; i++)
            {
                int startIdx = i * 2;
                userTraj[startIdx] = px[i].x;
                userTraj[startIdx + 1] =  px[i].y;
                gizmoSpheres2[i] = new Vector3(px[i].x, 0, px[i].y) + hip.transform.position;
            }
            inputDebugEnd = gizmoSpheres2[2];
            return userTraj;
        }

        Vector2 desiredVelDir = desiredVel - currentVel;
        if (desiredVelDir.magnitude > acceleration)
            desiredVelDir = desiredVelDir.normalized * acceleration;
        currentVel += desiredVelDir * Time.deltaTime;
        userTraj = new float[6];
        int idx = 0;
        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            float futureXPos = (currentVel.x * frameTime) * frameNum;
            float futureZPos = (currentVel.y * frameTime) * frameNum;
            userTraj[idx] = futureXPos;
            userTraj[idx + 1] = futureZPos;
            gizmoSpheres2[i - 1] = new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;
            if (i == 3)
            {
                inputDebugEnd = new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;
            }
            idx += 2;
        }
        return userTraj;
        //Debug.Log("Desired x vel: " + desiredXVel.ToString() + " Desierd z vel: " + desiredZVel.ToString());
    }

    private Vector2 getDesiredVelocity()
    {
        Vector2 stickL = gamepad.leftStick.ReadValue();
        float targetSpeed;
        if (stickL == Vector2.zero) targetSpeed = 0.0f;
        else
        {
            targetSpeed = stickL.magnitude * MoveSpeed;
        }
        Vector2 desiredVel = stickL.normalized * targetSpeed;
        return desiredVel;
    }
    // This function takes the current user velocity and updates it using a critically dampened spring based on the current user inputs
    private void updateUserVelocity()
    {
        Vector2 desiredVel = getDesiredVelocity();
        SpringUtils.spring_character_update(
               currentVel,
               acc,
               desiredVel,
               halfLife,
               Time.deltaTime,
               out currentVel,
               out acc);
        return;
    }
    private Vector3 combineHipGlobalVel(Vector3 hipGlobalVel)
    {
        if (useSpringsForVel) {
            return new Vector3(currentVel.x, hipGlobalVel.y, currentVel.y);
        }

        Vector2 desiredVel = getDesiredVelocity();
        Vector2 desiredVelDir = desiredVel - currentVel;
        if (desiredVelDir.magnitude > acceleration)
            desiredVelDir = desiredVelDir.normalized * acceleration;
        currentVel += desiredVelDir * Time.deltaTime;

        float newX = hipGlobalVel.x * (1 - velCombineFactor) + currentVel.x * velCombineFactor;
        float newZ = hipGlobalVel.z * (1 - velCombineFactor) + currentVel.y * velCombineFactor;
        return new Vector3(newX, hipGlobalVel.y, newZ);
    }
    /*
    To construct this feature vector, we take the joint positions and velocities from
    the current best matching animation being played, and compute
    the future trajectory points by extrapolating the user input with a
    critically damped spring damper, which essentially mixes the current
    character velocity with the desired user velocity
    */
    private void motionMatch()
    {
        float[] currentSearchVector = getCurrentSearchVector();
         normalizeVector(currentSearchVector);
        //Debug.Log("normalized Vector: " + string.Join(",", currentSearchVector));
        float[] bestMatchingAnimation = bruteforceSearch ? motionDB.bruteForceSearch(currentSearchVector) : motionDB.nnSearch(currentSearchVector);
        //Debug.Log("bestMatchingAnimation: " + string.Join(",", bestMatchingAnimation));

        int bestFrameIdx = (int)bestMatchingAnimation[searchVecLen];

        int bestFileIdx = (int)bestMatchingAnimation[searchVecLen + 1];
        bool cond_a = bestFileIdx == curFileIdx && (intsApproxSame(bestFrameIdx , curFrameIdx ) || intsApproxSame(bestFrameIdx, lastMMFrameIdx));
        bool cond_b = bestFileIdx == nextFileIdx && (intsApproxSame(bestFrameIdx, nextFrameIdx) || intsApproxSame(bestFrameIdx, lastMMFrameIdx));

        if (cond_a || cond_b)
        {
            // just let it play
            playNextFrame();
            Debug.Log("MM not transitioning because: " + (cond_a ? "cond_a" : "cond_b"));
            return;
        }
        //curFrameIdx = bestFrameIdx;
        //curFileIdx = bestFileIdx;
        if (curFileIdx == -1)
        {
            curFrameIdx = bestFrameIdx;
            curFileIdx = bestFileIdx;
        }
        else
        {
            nextFrameIdx = bestFrameIdx;
            nextFileIdx = bestFileIdx;
            curTransitionFrameNum = 1;
        }
        lastMMFrameIdx = bestFrameIdx;

        if (applyMM)
        {
            Debug.Log("MM  transitioning to: " + prefixes[bestFileIdx] + " Frame: " + bestFrameIdx.ToString());
            playNextFrame();
        }
    }

    private void playNextFrame()
    {
        //curFrameIdx++;
        if (nextFileIdx != -1 && curTransitionFrameNum <= transitionTime)
        {
            if (useInertializationBlending)
            {


            } else
            {
                int frameToLerpFrom = lerpFromFristFrame ? curFrameIdx - (curTransitionFrameNum - 1) : curFrameIdx;
                BVHUtils.lerp(frameToLerpFrom, boneLists[curFileIdx], nextFrameIdx, boneLists[nextFileIdx], nameToTransformMap, ((float)curTransitionFrameNum) / transitionTime, useAnimTransforms);
            }
            if (curTransitionFrameNum == transitionTime)
            {
                curFrameIdx = nextFrameIdx;
                curFileIdx = nextFileIdx;
                nextFileIdx = -1;
                nextFrameIdx = -1;
            }
            nextFrameIdx++;
            curTransitionFrameNum++;
        } else
        {
            BVHUtils.playFrame(curFrameIdx, boneLists[curFileIdx], nameToTransformMap, true, useAnimTransforms);
        }

        curFrameIdx++;
        //Debug.Log("Playing file: " + prefixes[curFileIdx] + " Frame: " + curFrameIdx.ToString());
    }

    private void updatePhysics()
    {
        float targetSpeed;
        // Get desired velocity 
        Vector2 stickL = gamepad.leftStick.ReadValue();
        // note: Vector2's == operator uses approximation so is not floating point error prone, and is cheaper than magnitude
        // if there is no input, set the target speed to 0
        if (stickL == Vector2.zero) targetSpeed = 0.0f;
        else
        {
            targetSpeed = stickL.magnitude * MoveSpeed;
        }
        Vector2 desiredVel = stickL.normalized * targetSpeed;
        Vector2 desiredVelDir = desiredVel - currentVel;
        if (desiredVelDir.magnitude > acceleration)
            desiredVelDir = desiredVelDir.normalized * acceleration ;
        currentVel += desiredVelDir * Time.deltaTime;
        Vector3 deltaPosition = new Vector3(currentVel.x, 0f, currentVel.y) * Time.deltaTime;
        hip.transform.position += deltaPosition;
    }




    private void inertialize_pose_transition(
        Vector3[] bone_offset_positions,
        Vector3[] bone_offset_velocities,
        Quaternion[] bone_offset_rotations,
        Vector3[] bone_offset_angular_velocities,
        ref Vector3 transition_src_position,
        ref Quaternion transition_src_rotation,
        ref Vector3 transition_dst_position,
        ref Quaternion transition_dst_rotation,
        in Vector3 root_position,
        in Vector3 root_velocity,
        in Quaternion root_rotation,
        in Vector3 root_angular_velocity,
        in Vector3[] bone_src_positions,
        in Vector3[] bone_src_velocities,
        in Quaternion[] bone_src_rotations,
        in Vector3[] bone_src_angular_velocities,
        in Vector3[] bone_dst_positions,
        in Vector3[] bone_dst_velocities,
        in Quaternion[] bone_dst_rotations,
        in Vector3[] bone_dst_angular_velocities)
    {

        // First we record the root position and rotation
        // in the animation data for the source and destination
        // animation
        transition_dst_position = root_position;
        transition_dst_rotation = root_rotation;
        transition_src_position = bone_dst_positions[0];
        transition_src_rotation = bone_dst_rotations[0];

        // We then find the velocities so we can transition the 
        // root inertiaizers

        Vector3 world_space_dst_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_velocities[0]));

        Vector3 world_space_dst_angular_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_dst_angular_velocities[0]));

        // Transition inertializers recording the offsets for 
        // the root joint
        SpringUtils.inertialize_transition(
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            root_position,
            root_velocity,
            root_position,
            world_space_dst_velocity);

        SpringUtils.inertialize_transition(
            ref bone_offset_rotations[0],
            ref bone_offset_angular_velocities[0],
            root_rotation,
            root_angular_velocity,
            root_rotation,
            world_space_dst_angular_velocity);

        // Transition all the inertializers for each other bone
        for (int i = 1; i < bone_offset_positions.Length; i++)
        {
            SpringUtils.inertialize_transition(
                ref bone_offset_positions[i],
                ref bone_offset_velocities[i],
                bone_src_positions[i],
                bone_src_velocities[i],
                bone_dst_positions[i],
                bone_dst_velocities[i]);

            SpringUtils.inertialize_transition(
                ref bone_offset_rotations[i],
                ref bone_offset_angular_velocities[i],
                bone_src_rotations[i],
                bone_src_angular_velocities[i],
                bone_dst_rotations[i],
                bone_dst_angular_velocities[i]);
        }
    }

    // This function updates the inertializer states. Here 
    // it outputs the smoothed animation (input plus offset) 
    // as well as updating the offsets themselves. It takes 
    // as input the current playing animation as well as the 
    // root transition locations, a halflife, and a dt
    private void inertialize_pose_update(
        Vector3[] bone_positions,
        Vector3[] bone_velocities,
        Quaternion[] bone_rotations,
        Vector3[] bone_angular_velocities,
        Vector3[] bone_offset_positions,
        Vector3[] bone_offset_velocities,
        Quaternion[] bone_offset_rotations,
        Vector3[] bone_offset_angular_velocities,
        in Vector3[] bone_input_positions,
        in Vector3[] bone_input_velocities,
        in Quaternion[] bone_input_rotations,
        in Vector3[] bone_input_angular_velocities,
        in Vector3 transition_src_position,
        in Quaternion transition_src_rotation,
        in Vector3 transition_dst_position,
        in Quaternion transition_dst_rotation,
        in float halflife,
        in float dt)
    {
        // First we find the next root position, velocity, rotation
        // and rotational velocity in the world space by transforming 
        // the input animation from it's animation space into the 
        // space of the currently playing animation.
        Vector3 world_space_position = Utils.quat_mul_vec3(transition_dst_rotation,
        Utils.quat_inv_mul_vec3(transition_src_rotation,
            bone_input_positions[0] - transition_src_position)) + transition_dst_position;

        Vector3 world_space_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
        Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities[0]));

        // Normalize here because quat inv mul can sometimes produce 
        // unstable returns when the two rotations are very close.
        Quaternion world_space_rotation = Quaternion.Normalize(transition_dst_rotation * 
            Utils.quat_inv_mul(transition_src_rotation, bone_input_rotations[0]));

        Vector3 world_space_angular_velocity = Utils.quat_mul_vec3(transition_dst_rotation,
            Utils.quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities[0]));

        // Then we update these two inertializers with these new world space inputs
        SpringUtils.inertialize_update(
            ref bone_positions[0],
            ref bone_velocities[0],
            ref bone_offset_positions[0],
            ref bone_offset_velocities[0],
            world_space_position,
            world_space_velocity,
            halflife,
            dt);

        SpringUtils.inertialize_update(
            ref bone_rotations[0],
            ref bone_angular_velocities[0],
            ref bone_offset_rotations[0],
            ref bone_offset_angular_velocities[0],
            world_space_rotation,
            world_space_angular_velocity,
            halflife,
            dt);        
    
        // Then we update the inertializers for the rest of the bones
        for (int i = 1; i < bone_positions.Length; i++)
        {
            SpringUtils.inertialize_update(
                ref bone_positions[i],
                ref bone_velocities[i],
                ref bone_offset_positions[i],
                ref bone_offset_velocities[i],
                bone_input_positions[i],
                bone_input_velocities[i],
                halflife,
                dt);

            SpringUtils.inertialize_update(
                ref bone_rotations[i],
                ref bone_angular_velocities[i],
                ref bone_offset_rotations[i],
                ref bone_offset_angular_velocities[i],
                bone_input_rotations[i],
                bone_input_angular_velocities[i],
                halflife,
                dt);
        }
    }

    void Start()
    {
        searchVecLen = useGlobal ? 24 : 23;
        prefixes = walkOnly ? walkPrefixes : allPrefixes;

        motionDB = new KDTree(searchVecLen, 2, numNeigh, trajPenalty);
        Application.targetFrameRate = targetFramerate;
        hipRotOffset = hip.transform.rotation.eulerAngles;
        if (this.KeepSceneViewActive && Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        ingestMotionMatchingDB();

        loadBVHFiles();
        gamepad = Gamepad.current;
    }

    // Update is called once per frame
    void Update()
    {
        gizmoSpheres1 = new Vector3[3];  // MUST BE EVEN LENGTH
        gizmoSpheres2 = new Vector3[3];
        gizmoSpheres3 = new Vector3[3];
        textLabels = new string[3];
        if (!applyMM)
        {
            getCurrentSearchVector();
            return;
        }

        if (frameCounter % updateEveryNFrame == 0)
        {
            motionMatch();
        } else
        {
            // hack to call draw gizmos
            getCurrentSearchVector();
            playNextFrame();
        }
        lastHipPos = hip.transform.position;
        lastHipQuat = hip.transform.rotation;
        lastLeftFootGlobalPos = leftFoot.transform.position;
        lastRightFootGlobalPos = rightFoot.transform.position;
        if (!useAnimTransforms)
        {
            updatePhysics();
        }
        frameCounter++;
    }

}
