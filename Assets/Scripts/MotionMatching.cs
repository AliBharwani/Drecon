using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using System.IO;

public class MotionMatching : MonoBehaviour
{
    public GameObject leftFoot;
    public GameObject rightFoot;
    public GameObject root;
    public GameObject hip;
    //public GameObject leftLeg;
    //// Constraints: Z rotates from 0-150
    //public GameObject rightLeg;

    //public GameObject leftUpperLeg;
    //public GameObject rightUpperLeg;
    //private Animation anim;

    private Animator animator;
    AnimatorClipInfo[] m_AnimatorClipInfo;


    private Vector3 lastLeftFootGlobalPos;
    private Vector3 lastRightFootGlobalPos;
    private Vector3 lastHipGlobalPos;

    private float _rightKneeRotation;
    private string animPrefix = "walk1_";

    private string currentStateName = "walk1_0";
    public int startAtIdx = -1;
    public int targetFramerate = 24;
    public bool clearFileNameOnStart = true;
    private int frameIdx = 0;
    private int overallIdx = 0;
    private int animClipIdx = 0;
    public static int numTotalFrames = 7800;
    private float[ ][]  data = new float[numTotalFrames][];
    private float[] dataInsertionTracker = new float[numTotalFrames];
    private void Awake()
    {
        Application.targetFrameRate = targetFramerate;
        if (startAtIdx > 0)
        {
            currentStateName = animPrefix + startAtIdx.ToString();
            overallIdx = startAtIdx * 100;
            animClipIdx = startAtIdx;
        }

    }
    void Start()
    {
        // GET CHILDREN 
        //anim = gameObject.GetComponent<Animation>();
        animator = GetComponent<Animator>();
    }

    // Update is called once per frame
    void Update()
    {
        //m_AnimatorClipInfo = animator.GetCurrentAnimatorClipInfo(0);
        //Output the name of the starting clip
        //Debug.Log("Starting clip : " + m_AnimatorClipInfo[0].clip);
        if (overallIdx < numTotalFrames)
        {
            if (frameIdx == 100)
            {
                frameIdx = 0;
                animClipIdx++;
                currentStateName = animPrefix + animClipIdx.ToString();
                Debug.Log("Now playing " + currentStateName);
            }
            animator.Play(currentStateName, 0, (.01f * frameIdx));
        } else if (overallIdx == numTotalFrames)
        {
            printData();
            overallIdx++;
        }
        //Debug.Log(leftFootLocalPos.ToString() + " , "  + rightFootLocalPos.ToString() + " , " + leftFootGlobalVelocity.ToString() + " , " + rightFootGlobalVelocity.ToString() + " , " + hipGlobalVel.ToString() + " , " + curTrajectoryOntoGroundPlane.ToString() + " , " + curOrientationOntoGroundPlane.ToString() + " , ");
    }

    private void printData()
    {
        string filename = Application.dataPath + "/" + animPrefix + "output.txt";
        if (File.Exists(filename) && clearFileNameOnStart)
        {
            File.Delete(filename);
        }
        TextWriter tw = new StreamWriter(filename, true);
        tw.WriteLine("[" + System.DateTime.Now + "]");
        int idx = 0;
        foreach (float[] row in data)
        {
            if (row == null)
            {
                continue;
            }
            StringBuilder sb = new StringBuilder();
            sb.Append("[ ");
            foreach (float val in row)
            {
                sb.Append(val);
                sb.Append(" , ");
            }
            sb.Append("]");
            //Debug.Log(sb.ToString());
            int printIdx = startAtIdx > 0 ? idx + startAtIdx*100 : idx;
            tw.WriteLine("Idx " + printIdx.ToString() + " : " + sb.ToString());
            idx++; 
        }
        tw.Close();
    }

    private void LateUpdate()
    {
        if (overallIdx < numTotalFrames)
        {
            data[overallIdx] = getCurrentSearchVector();
            frameIdx++;
            overallIdx++;
            //dataInsertionTracker = animator.GetCurrentAnimatorStateInfo().normalizedTime;
        }

    }

    private float[] getCurrentSearchVector()
    {
        if (lastLeftFootGlobalPos == null)
        {
            lastLeftFootGlobalPos = leftFoot.transform.position;
            lastRightFootGlobalPos = rightFoot.transform.position;
            lastHipGlobalPos = hip.transform.position;
            return new float[19];
        }

        // get left and right foot local positions and global velocities
        // 2 pairs (left and right) of 2 vectors in r^3, 3 numbers
        Vector3 leftFootLocalPos = leftFoot.transform.position - root.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - root.transform.position;
        // velocity is the change in distance over time - if you went from 0m to 10m in 1 sec, your veloctiy is 10m/s 
        Vector3 leftFootGlobalVelocity = (leftFoot.transform.position - lastLeftFootGlobalPos) / Time.deltaTime;
        Vector3 rightFootGlobalVelocity = (rightFoot.transform.position - lastRightFootGlobalPos) / Time.deltaTime;
        lastLeftFootGlobalPos = leftFoot.transform.position;
        lastRightFootGlobalPos = rightFoot.transform.position;

        // hip global velocity (one number in r3, 3 numbers)
        Vector3 hipGlobalVel = (hip.transform.position - lastHipGlobalPos) / Time.deltaTime;
        lastHipGlobalPos = hip.transform.position;

        // (hip)trajectory positions and orientations  located at 20, 40, and 60 frames in the future which are projected onto the
        // groundplane(t-sub - i in R ^ 6, d - sub - i in R ^ 6, concatenating 3 xy pairs -> R ^ 6, total 12 numbers) 
        // hack - for right now just list trajectory position and orientation, then in cleanup we map the features 
        //Vector2 curTrajectoryOntoGroundPlane = new Vector2(hip.transform.position.x, hip.transform.position.z);
        //Vector2 curOrientationOntoGroundPlane = new Vector2(hip.transform.rotation.eulerAngles.x, hip.transform.rotation.eulerAngles.z);
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
                hip.transform.position.x, hip.transform.position.z,
                hip.transform.rotation.eulerAngles.x, hip.transform.rotation.eulerAngles.y,
        };
    }
    private void updateCurrentRotations()
    {
        //_rightKneeRotation = rightLeg.transform.rotation.z;
    }
}
