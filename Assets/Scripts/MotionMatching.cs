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
    public bool KeepSceneViewActive = true;
    public int targetFramerate = 30;
    public GameObject leftFoot;
    public GameObject rightFoot;
    public GameObject root;
    public GameObject hip;
    public int updateEveryNFrame = 10;
    public bool drawGizmos = true;
    public bool useQuadraticVel = true;
    public bool yrotonly = true;
    public float gizmoSphereRad = .01f;
    public float hackyMaxVelReducer = 5f;
    public bool applyMM = false;

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
    private string[] prefixes = {
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
    private Vector3 animDebugStart, animDebugEnd, inputDebugStart, inputDebugEnd, finalDebugStart, finalDebugEnd;
    private List<Vector3> gizmoSpheres1 = new List<Vector3>(); 
    private List<Vector3> gizmoSpheres2 = new List<Vector3>();
    private List<Vector3> gizmoSpheres3 = new List<Vector3>();
    private List<string> textLabels = new List<string>();
    private Dictionary<BVHParser.BVHBone, Transform>[] bvhMaps;

    private bool firstFrame = true;
    private void loadBVHFiles()
    {
        //boneToTransformMap = new Dictionary<BVHParser.BVHBone, Transform>();
        bvhMaps = new Dictionary<BVHParser.BVHBone, Transform>[prefixes.Length];
        for (int i = 0; i < prefixes.Length; i++)
        {
            string prefix = prefixes[i];
            BVHParser bp = BVHUtils.parseFile(prefix + ".bvh");
            bvhMaps[i] = new Dictionary<BVHParser.BVHBone, Transform>();
            BVHUtils.loadTransforms(bp, bvhMaps[i], transform);
        }
    }
    private void ingestMotionMatchingDB()
    {
        // @ escapes the backslashes

        DateTime startTime = DateTime.Now;
        int counter = 0;
        string pathToData = yrotonly ? @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs_yrotonly/" : @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/";
        int j = 0;
        foreach (string line in File.ReadLines(pathToData +  "stats.txt"))
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
        for (int i= 0; i < prefixes.Length; i++)
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
        for(int i = 0; i < searchVecLen; i ++)
        {
            vec[i] = (vec[i] - means[i]) / std_devs[i];
        }
    }
    void OnDrawGizmos()
    {
        if (!drawGizmos)
            return;
        // Draw a yellow sphere at the transform's position
        Gizmos.color = Color.blue;
        foreach(Vector3 spherePos in gizmoSpheres1)
        {
            Gizmos.DrawSphere(spherePos, gizmoSphereRad);
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
        for (int i = 0; i < gizmoSpheres3.Count; i++)
        {
            Vector3 spherePos = gizmoSpheres3[i];
            Gizmos.DrawSphere(spherePos, gizmoSphereRad);
            Handles.Label(spherePos, textLabels[i]);
        }
        //foreach (Vector3 spherePos in gizmoSpheres3)
        //{
        //    Gizmos.DrawSphere(spherePos, .1f);
        //}
        //foreach (string label in textLabels)
        //{
        //    Handles.Label(transform.position, "Text");
        //}
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
        Vector3 leftFootLocalPos = leftFoot.transform.position - root.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - root.transform.position;
        // velocity is the change in distance over time - if you went from 0m to 10m in 1 sec, your veloctiy is 10m/s 
        Vector3 leftFootGlobalVelocity = (leftFoot.transform.position - lastLeftFootGlobalPos) / frameTime;
        Vector3 rightFootGlobalVelocity = (rightFoot.transform.position - lastRightFootGlobalPos) / frameTime;

        // hip global velocity (one number in r3, 3 numbers)
        Vector3 hipGlobalVelPerFrame = hip.transform.position - lastHipPos;

        Vector3 hipGlobalVel = (hip.transform.position - lastHipPos) / frameTime;

        // based off bobsir's answer in https://forum.unity.com/threads/manually-calculate-angular-velocity-of-gameobject.289462/
        Quaternion deltaRot = hip.transform.rotation * Quaternion.Inverse(lastHipQuat);
        Vector3 eulerRot = new Vector3(Mathf.DeltaAngle(0, deltaRot.eulerAngles.x), Mathf.DeltaAngle(0, deltaRot.eulerAngles.y), Mathf.DeltaAngle(0, deltaRot.eulerAngles.z));

        Vector3 hipAngularVelPerFrame = eulerRot;// / Time.fixedDeltaTime;
        //Vector3 hipAngularVel = (hip.transform.position - lastHipTransform.rotation.To) / frameTime;

        // (hip)trajectory positions and orientations  located at 20, 40, and 60 frames in the future which are projected onto the
        // groundplane(t-sub - i in R ^ 6, d - sub - i in R ^ 6, concatenating 3 xy pairs -> R ^ 6, total 12 numbers) 
        int additionalLen = yrotonly ? 9 : 15;
        float[] hipFutureTrajAndOrientations = new float[additionalLen];
        int idx = 0;
        float[] userTraj = readUserInput();
        animDebugStart = hip.transform.position;

        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            float futureXPos = hipGlobalVelPerFrame.x * frameNum;
            float futureZPos = hipGlobalVelPerFrame.z * frameNum;
            if (i == 3)
            {
                animDebugEnd = new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;
            }
            gizmoSpheres1.Add(new Vector3(futureXPos, 0, futureZPos) + hip.transform.position);
            //if (userInputtingLeftStick())
            //{
                int startIdx = (i - 1) * 2;
                futureXPos = combineCurTrajWithUser(futureXPos, userTraj[startIdx], frameNum);
                futureZPos = combineCurTrajWithUser(futureZPos, userTraj[startIdx + 1], frameNum);
                gizmoSpheres3.Add(new Vector3(futureXPos, 0, futureZPos) + hip.transform.position);
            //}

            hipFutureTrajAndOrientations[idx] = futureXPos;
            hipFutureTrajAndOrientations[idx + 1] = futureZPos;

            // Combine future Y rotation and current Y rotation
            //float currentEulerY = hip.transform.rotation.eulerAngles.y;
            //float difference = hip.transform.rotation.eulerAngles.y - targetY;
            float futureYRot = hipAngularVelPerFrame.y * frameNum;
            //if (userInputtingLeftStick())
            //{
                float targetY = userInputTargetY();
                //Debug.Log("Targety: " + targetY.ToString() + " Outputed Y : " + hip.transform.rotation.eulerAngles.y.ToString());

                futureYRot = combineYRots(hip.transform.rotation.eulerAngles.y, targetY, frameNum);
                textLabels.Add(futureYRot.ToString());
            //}
            hipFutureTrajAndOrientations[idx + 2] = futureYRot;
            //float futureXRot = hipAngularVelPerFrame.x * frameNum;
            //float futureYRot = hipAngularVelPerFrame.y * frameNum;
            //float futureZRot = hipAngularVelPerFrame.z * frameNum;
            //hipFutureTrajAndOrientations[idx + 2] = futureXRot;
            //hipFutureTrajAndOrientations[idx + 3] = futureYRot;
            //hipFutureTrajAndOrientations[idx + 4] = futureZRot;
            idx += additionalLen / 3;
        }
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
                hipGlobalVel.x,
                hipGlobalVel.y,
                hipGlobalVel.z,
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
        float a = .2f;
        float b = .85f;
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
        float a = .2f;
        float b = .85f;
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

    private bool userInputtingLeftStick()
    {
        if (gamepad == null)
            gamepad = Gamepad.current;
        Vector2 stickL = gamepad.leftStick.ReadValue();
        return  !(Mathf.Approximately(stickL.x, 0) && Mathf.Approximately(stickL.y, 0)) ;
    }

    private float userInputTargetY()
    {
        Vector2 stickL = gamepad.leftStick.ReadValue();
        float angle = Mathf.Atan2(stickL.y, stickL.x) * Mathf.Rad2Deg * -1;
        angle = angle < 0f ? angle + 360 : angle;
        // Have to rotate 90 deg
        return angle;
    }
    private float[] readUserInput()
    {
        Vector2 stickL = gamepad.leftStick.ReadValue();
        //if (Mathf.Approximately(stickL.x, 0) && Mathf.Approximately(stickL.y, 0))
        //{
        //    return new float[6];
        //}
        float desiredXVel, desiredZVel;
        if (useQuadraticVel)
        {
            Vector2 normalized = stickL.normalized * stickL.sqrMagnitude;
            desiredXVel = normalized.x * maxXVel;
            desiredZVel = normalized.y * maxZVel;
        } else
        {
            desiredXVel = stickL.x * maxXVel;
            desiredZVel = stickL.y * maxZVel;
        }
        inputDebugStart = hip.transform.position;
        float[] userTraj = new float[6];
        int idx = 0;
        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            float futureXPos = (desiredXVel * frameTime) * frameNum;
            float futureZPos = (desiredZVel  * frameTime) * frameNum;
            userTraj[idx] = futureXPos;
            userTraj[idx + 1] = futureZPos;
            gizmoSpheres2.Add(new Vector3(futureXPos, 0, futureZPos) + hip.transform.position);
            if (i == 3)
            {
                inputDebugEnd =new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;
            }
            idx += 2;
        }
        return userTraj;
        //Debug.Log("Desired x vel: " + desiredXVel.ToString() + " Desierd z vel: " + desiredZVel.ToString());
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
        double[] bestMatchingAnimation = motionDB.bruteForceSearch(currentSearchVector);
        int frameIdx = (int)bestMatchingAnimation[searchVecLen];
        int fileIdx = (int)bestMatchingAnimation[searchVecLen + 1];
        Debug.Log("Playing file: " + prefixes[fileIdx] + " Frame: " + frameIdx.ToString());
        if (applyMM) { 
        BVHUtils.playFrame(frameIdx, bvhMaps[fileIdx]);
        }
    }
    void Start()
    {
        // + 2 for extra data 
        searchVecLen = yrotonly ? 24 : 30;
        motionDB = new KDTree(searchVecLen, 2);
        Application.targetFrameRate = targetFramerate;
        hipRotOffset = hip.transform.rotation.eulerAngles;
        if (this.KeepSceneViewActive && Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        ingestMotionMatchingDB();
        maxXVel /= hackyMaxVelReducer;
        maxZVel /= hackyMaxVelReducer;
        loadBVHFiles();
        gamepad = Gamepad.current;
    }

    // Update is called once per frame
    void Update()
    {
        gizmoSpheres1 = new List<Vector3>();  // MUST BE EVEN LENGTH
        gizmoSpheres2 = new List<Vector3>();
        gizmoSpheres3 = new List<Vector3>();
        textLabels = new List<string>();

        //if (frameCounter % updateEveryNFrame == 0)
        //{
        //}
        motionMatch();
        lastHipPos = hip.transform.position;
        lastHipQuat = hip.transform.rotation;
        lastLeftFootGlobalPos = leftFoot.transform.position;
        lastRightFootGlobalPos = rightFoot.transform.position;
        frameCounter++;
    }

}
