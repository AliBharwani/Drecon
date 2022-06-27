using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using UnityEngine.InputSystem;

public class MotionMatching : MonoBehaviour
{
    public bool KeepSceneViewActive = true;
    public int targetFramerate = 30;
    public GameObject leftFoot;
    public GameObject rightFoot;
    public GameObject root;
    public GameObject hip;
    public int updateEveryNFrame = 10;
    private int frameCounter = 0;
    public bool drawGizmos = true;
    public bool useQuadraticVel = true;

    private Vector3 lastLeftFootGlobalPos;
    private Vector3 lastRightFootGlobalPos;
    private Vector3 lastHipPos;
    private Quaternion lastHipQuat;
    private KDTree motionDB = new KDTree();
    private string pathToAnims = @"D:/Unity/Unity 2021 Editor Test/Assets/LAFLAN Data/Animations/";
    private double[] means = new double[] { -0.0458957508171, 0.148889939955, -1.98212023408, -0.0303706152439, 0.147394518158, -2.00615341094, -0.00105541224217, 0.000328890938174, 0.0011302420994, -0.00187898262142, 3.8678468464e-05, 0.00122720085238, -0.00165934029416, -0.000206693790398, 0.00128557976763, -0.00163039570779, 0.00114093651462, 182.212187285, 188.200992116, 267.64185031, -0.00379681699947, 0.00228497923495, 182.354338485, 188.366045526, 267.646277596, -0.0064437902479, 0.00327497249638, 182.299978261, 188.581469405, 267.647131713 };
    private double[] std_devs = new double[] { 2.75672803451, 0.0851475658313, 5.39294074082, 2.76605976978, 0.0867144467634, 5.404615269, 1.50215804108, 0.671604269949, 1.99188466153, 1.52159098328, 0.687993202854, 2.0142738406, 1.02034196113, 0.29502827464, 1.43598251437, 0.653119557334, 0.933524741409, 175.25471024, 107.414065461, 8.47444261664, 1.20173555218, 1.77276238065, 175.25317976, 107.38595772, 8.47702118087, 1.60681077673, 2.47236803325, 175.261603812, 107.328262296, 8.47962818468 };
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
    private float maxXVel = 4.92068f;
    private float maxZVel = 6.021712f;

    private Vector3 animDebugStart, animDebugEnd, inputDebugStart, inputDebugEnd, finalDebugStart, finalDebugEnd;
    private List<Vector3> gizmoSpheres1 = new List<Vector3>();  // MUST BE EVEN LENGTH
    private List<Vector3> gizmoSpheres2 = new List<Vector3>();
    private bool firstFrame = true;

    private void ingestMotionMatchingDB()
    {
        // @ escapes the backslashes

        DateTime startTime = DateTime.Now;
        int counter = 0;
        string pathToData = @"D:/Unity/Unity 2021 Editor Test/Python/pyoutputs/";

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

    private void normalizeVector(double[] vec)
    {
        for(int i = 0; i < 30; i ++)
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
            Gizmos.DrawSphere(spherePos, .1f);
        }
        if (animDebugStart != null)
        {
            Gizmos.DrawLine(animDebugStart, animDebugEnd);
        }
        Gizmos.color = Color.red;
        foreach (Vector3 spherePos in gizmoSpheres2)
        {
            Gizmos.DrawSphere(spherePos, .1f);
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
    }
    private float[] getCurrentSearchVector()
    {
        if (firstFrame)
        {
            firstFrame = false;
            return new float[30];
        }

        // get left and right foot local positions and global velocities
        // 2 pairs (left and right) of 2 vectors in r^3, 3 numbers
        Vector3 leftFootLocalPos = leftFoot.transform.position - root.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - root.transform.position;
        // velocity is the change in distance over time - if you went from 0m to 10m in 1 sec, your veloctiy is 10m/s 
        Vector3 leftFootGlobalVelocity = (leftFoot.transform.position - lastLeftFootGlobalPos) / frameTime;
        Vector3 rightFootGlobalVelocity = (rightFoot.transform.position - lastRightFootGlobalPos) / frameTime;

        // hip global velocity (one number in r3, 3 numbers)
        //Debug.Log("hip.transform.position: " + hip.transform.position.ToString("F6") + "  lastHipTransform.position: " + lastHipPos.ToString("F6"));

        Vector3 hipGlobalVelPerFrame = hip.transform.position - lastHipPos;

        Vector3 hipGlobalVel = (hip.transform.position - lastHipPos) / frameTime;

        // based off bobsir's answer in https://forum.unity.com/threads/manually-calculate-angular-velocity-of-gameobject.289462/
        Quaternion deltaRot = hip.transform.rotation * Quaternion.Inverse(lastHipQuat);
        Vector3 eulerRot = new Vector3(Mathf.DeltaAngle(0, deltaRot.eulerAngles.x), Mathf.DeltaAngle(0, deltaRot.eulerAngles.y), Mathf.DeltaAngle(0, deltaRot.eulerAngles.z));

        Vector3 hipAngularVelPerFrame = eulerRot;// / Time.fixedDeltaTime;
        //Vector3 hipAngularVel = (hip.transform.position - lastHipTransform.rotation.To) / frameTime;

        // (hip)trajectory positions and orientations  located at 20, 40, and 60 frames in the future which are projected onto the
        // groundplane(t-sub - i in R ^ 6, d - sub - i in R ^ 6, concatenating 3 xy pairs -> R ^ 6, total 12 numbers) 
        // hack - for right now just list trajectory position and orientation, then in cleanup we map the features 
        //Vector2 curTrajectoryOntoGroundPlane = new Vector2(hip.transform.position.x, hip.transform.position.z);
        //Vector2 curOrientationOntoGroundPlane = new Vector2(hip.transform.rotation.eulerAngles.x, hip.transform.rotation.eulerAngles.z);
        float[] hipFutureTrajAndOrientations = new float[15];
        int idx = 0;
        Debug.Log(hipGlobalVelPerFrame.ToString("F6"));
        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            float futureXPos = hipGlobalVelPerFrame.x * frameNum;
            float futureZPos = hipGlobalVelPerFrame.z * frameNum;
            gizmoSpheres1.Add(new Vector3(futureXPos, 0, futureZPos) + hip.transform.position);
            hipFutureTrajAndOrientations[idx] = futureXPos;
            hipFutureTrajAndOrientations[idx + 1] = futureZPos;
            float futureXRot = hipAngularVelPerFrame.x * frameNum;
            float futureYRot = hipAngularVelPerFrame.y * frameNum;
            float futureZRot = hipAngularVelPerFrame.z * frameNum;
            hipFutureTrajAndOrientations[idx + 2] = futureXRot;
            hipFutureTrajAndOrientations[idx + 3] = futureYRot;
            hipFutureTrajAndOrientations[idx + 4] = futureZRot;
            idx += 5;
        }
        animDebugStart = hip.transform.position;
        animDebugEnd = new Vector3(hipFutureTrajAndOrientations[10], 0, hipFutureTrajAndOrientations[11]) + hip.transform.position;
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
                hipFutureTrajAndOrientations[9],
                hipFutureTrajAndOrientations[10],
                hipFutureTrajAndOrientations[11],
                hipFutureTrajAndOrientations[12],
                hipFutureTrajAndOrientations[13],
                hipFutureTrajAndOrientations[14],
        };
    }
    private void readUserInput()
    {
        if (gamepad == null)
            gamepad = Gamepad.current;
        Vector2 stickL = gamepad.leftStick.ReadValue();
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
        for (int i = 1; i < 4; i++)
        {
            int frameNum = i * 20;
            float futureXPos = (desiredXVel * frameTime) * frameNum;
            float futureZPos = (desiredZVel  * frameTime) * frameNum;
            gizmoSpheres2.Add(new Vector3(futureXPos, 0, futureZPos) + hip.transform.position);
            if (i == 3)
            {
                inputDebugEnd =new Vector3(futureXPos, 0, futureZPos) + hip.transform.position;
            }
        }
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
        readUserInput();
        //normalizeVector(currentSearchVector);
        //double[] userInputSomething = magic();
        //double[] combined = combined(currentSearchVector, userInputSomething);
        //double[] bestMatchingAnimation = motionDB.nnSearch(searchVector);
        //string filePath = pathToAnims + prefixes[(int)bestMatchingAnimation[31]] + ".bvh";
        //lerp(bestMatchingAnimation[30], filePath);
    }
    void Start()
    {
        Application.targetFrameRate = targetFramerate;
        if (this.KeepSceneViewActive && Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        ingestMotionMatchingDB();
        gamepad = Gamepad.current;
    }

    // Update is called once per frame
    void Update()
    {
        gizmoSpheres1 = new List<Vector3>();  // MUST BE EVEN LENGTH
        gizmoSpheres2 = new List<Vector3>();
        if (frameCounter % updateEveryNFrame == 0)
        {
            //motionMatch();
            //var gamepad = Gamepad.current;
            //if (gamepad == null)
            //{
            //    Debug.Log("Gamepad null");
            //    return;  // No gamepad connected.
            //}
            //Vector2 move = gamepad.leftStick.ReadValue();
            //Debug.Log("Frame: " + frameCounter.ToString() + " Left stick value: " + move.ToString());
        }
        motionMatch();
        lastHipPos = hip.transform.position;
        lastHipQuat = hip.transform.rotation;
        lastLeftFootGlobalPos = leftFoot.transform.position;
        lastRightFootGlobalPos = rightFoot.transform.position;
        frameCounter++;
    }

}
