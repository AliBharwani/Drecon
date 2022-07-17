using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using UnityEngine.InputSystem;
using UnityEditor;

public class MotionMatching : MonoBehaviour
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
    [Header("====   PHYSICS    ====")]
    public bool useQuadraticVel = true;
    //public float hackyMaxVelReducer = 5f;
    //public Vector3 acc;
    public float MoveSpeed = 2.0f;
    public float SprintSpeed = 5.335f;
    public float acceleration = 10.0f;

    private Vector3 velocity;
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
        string pathToData = @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/";
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
                motionDB.Add(stringValues.Select(double.Parse).ToArray());
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
        Vector3 hipGlobalVelPerFrame = useAnimTransforms ? hip.transform.position - lastHipPos : velocity * frameTime; // hip.transform.position - lastHipPos;

        // I should combine hipGlolabVel with user input
        Vector3 hipGlobalVel = useAnimTransforms ? (hip.transform.position - lastHipPos) / frameTime : velocity;// (hip.transform.position - lastHipPos) / frameTime;
        Vector3 combinedHipGlobalVel = combineHipGlobalVel(hipGlobalVel);
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
            float futureYRot = BVHUtils.getDifferenceInYRots(hip.transform.rotation.eulerAngles.y, targetY);
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
        Vector3 altCombinedHipGlobalVel = combineHipGlobalVel(alternativeHipGlobalVel);
        Vector3 velocitiesToUse = altCombinedHipGlobalVel;
        if (!useAnimTransforms)
        {
            velocitiesToUse = new Vector3(currentVel.x, 0f, currentVel.y);
        }
        float velocityMag;
        float angleBetweenVelHips = BVHUtils.getAngleBetweenVelocityAndHip(hip.transform, velocitiesToUse, out velocityMag);

        return new float[] {
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
                //velocitiesToUse.x,
                //velocitiesToUse.y,
                //velocitiesToUse.z,
                // hip.transform.rotation.eulerAngles.y,
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
                //hipFutureTrajAndOrientations[9],
                //hipFutureTrajAndOrientations[10],
                //hipFutureTrajAndOrientations[11],
                //hipFutureTrajAndOrientations[12],
                //hipFutureTrajAndOrientations[13],
                //hipFutureTrajAndOrientations[14],
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
        double angle = BVHUtils.reverseAtan2ClockDirection(stickL.y, stickL.x);
        return (float) angle;
    }
    private float[] readUserInput()
    {
        //Vector2 stickL = gamepad.leftStick.ReadValue();
        //if (Mathf.Approximately(stickL.x, 0) && Mathf.Approximately(stickL.y, 0))
        //{
        //    return new float[6];
        //}
        //float desiredXVel, desiredZVel;
        //if (useQuadraticVel)
        //{
        //    Vector2 normalized = stickL.normalized * stickL.sqrMagnitude;
        //    desiredXVel = normalized.x * maxXVel;
        //    desiredZVel = normalized.y * maxZVel;
        //}
        //else
        //{
        //    desiredXVel = stickL.x * maxXVel;
        //    desiredZVel = stickL.y * maxZVel;
        //}
        //float desiredSpeed = stickL.magnitude * MoveSpeed;
        //Vector2 desiredVel = stickL.normalized * desiredSpeed;
        //desiredXVel = desiredVel.x;
        //desiredZVel = desiredVel.y;
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
            desiredVelDir = desiredVelDir.normalized * acceleration;
        currentVel += desiredVelDir * Time.deltaTime;
        inputDebugStart = hip.transform.position;
        float[] userTraj = new float[6];
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
    private Vector3 combineHipGlobalVel(Vector3 hipGlobalVel)
    {
        Vector2 stickL = gamepad.leftStick.ReadValue();
        float targetSpeed;
        float desiredSpeed = stickL.magnitude * MoveSpeed;
        if (stickL == Vector2.zero) targetSpeed = 0.0f;
        else
        {
            targetSpeed = stickL.magnitude * MoveSpeed;
        }
        Vector2 desiredVel = stickL.normalized * targetSpeed;
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
        double[] bestMatchingAnimation = bruteforceSearch ? motionDB.bruteForceSearch(currentSearchVector) : motionDB.nnSearch(currentSearchVector);
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
            int frameToLerpFrom = lerpFromFristFrame ? curFrameIdx - (curTransitionFrameNum - 1) : curFrameIdx;
            BVHUtils.lerp(frameToLerpFrom, boneLists[curFileIdx], nextFrameIdx, boneLists[nextFileIdx], nameToTransformMap, ((float) curTransitionFrameNum) / transitionTime, true, useAnimTransforms);
            if (curTransitionFrameNum == transitionTime)
            {
                curFrameIdx = nextFrameIdx;
                curFileIdx = nextFileIdx;
                nextFileIdx = -1;
                nextFrameIdx = -1;
            } else
            {
                nextFrameIdx++;
            }
            curTransitionFrameNum++;
        } else
        {
            BVHUtils.playFrame(curFrameIdx, boneLists[curFileIdx], nameToTransformMap, true, useAnimTransforms);
        }

        curFrameIdx++;
        //Debug.Log("Playing file: " + prefixes[curFileIdx] + " Frame: " + curFrameIdx.ToString());
    }


    private void updatePhysics_OLD()
    {
        float targetSpeed = MoveSpeed; // _input.sprint ? SprintSpeed : MoveSpeed;
        Vector2 stickL = gamepad.leftStick.ReadValue();
        // note: Vector2's == operator uses approximation so is not floating point error prone, and is cheaper than magnitude
        // if there is no input, set the target speed to 0
        if (stickL == Vector2.zero) targetSpeed = 0.0f;
        float currentHorizontalSpeed = velocity.magnitude;

        float speedOffset = 0.1f;
        float inputMagnitude = stickL.magnitude;
        // accelerate or decelerate to target speed
        float _speed;
        if (currentHorizontalSpeed < targetSpeed - speedOffset || currentHorizontalSpeed > targetSpeed + speedOffset)
        {
            // creates curved result rather than a linear one giving a more organic speed change
            // note T in Lerp is clamped, so we don't need to clamp our speed
            _speed = Mathf.Lerp(currentHorizontalSpeed, targetSpeed * inputMagnitude, Time.deltaTime * acceleration);

            // round speed to 3 decimal places
            _speed = Mathf.Round(_speed * 1000f) / 1000f;
        }
        else
        {
            _speed = targetSpeed;
        }
        Vector2 normalizedStickL = stickL.normalized;
        Vector3 deltaPosition = new Vector3(normalizedStickL.x, 0f, normalizedStickL.y) * (_speed * Time.deltaTime);
        velocity = deltaPosition / Time.deltaTime;
        hip.transform.position += deltaPosition;
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
    void Start()
    {
        //maxXVel /= hackyMaxVelReducer;
        //maxZVel /= hackyMaxVelReducer;
        //useAnimTransforms = false;
        // + 2 for extra data 
        searchVecLen = 24;
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
