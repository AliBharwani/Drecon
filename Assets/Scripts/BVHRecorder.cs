using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

public class BVHRecorder: MonoBehaviour
{

    [Tooltip("This is the path to the BVH file that should be loaded. Bone offsets are currently being ignored by this loader.")]
    public string filename;

    public bool KeepSceneViewActive = true;
    //public float scale = .01f;
    public int overrideFrames = -1;
    public int targetFramerate = 30;
    public GameObject leftFoot;
    public GameObject rightFoot;
    public GameObject root;
    public GameObject hip;
    public bool clearFileNameOnStart = true;
    public bool dryRun = true;
    private float[][] motionDB;

    public bool blender = false;
    public int startAtFrame = 0;

    private BVHParser bp = null;
    private int currentFrame = 0;
    // the frametime from the BVH file 
    private float frameTime;
    private Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap;
    private bool firstFrame = true;
    private Vector3 lastLeftFootGlobalPos;
    private Vector3 lastRightFootGlobalPos;
    private Vector3 lastHipGlobalPos;

    private float[] getCurrentSearchVector()
    {
        if (firstFrame)
        {
            firstFrame = false;
            lastLeftFootGlobalPos = leftFoot.transform.position;
            lastRightFootGlobalPos = rightFoot.transform.position;
            lastHipGlobalPos = hip.transform.position;
            return new float[20];
        }

        // get left and right foot local positions and global velocities
        // 2 pairs (left and right) of 2 vectors in r^3, 3 numbers
        Vector3 leftFootLocalPos = leftFoot.transform.position - hip.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - hip.transform.position;
        // velocity is the change in distance over time - if you went from 0m to 10m in 1 sec, your veloctiy is 10m/s 
        Vector3 leftFootGlobalVelocity = (leftFoot.transform.position - lastLeftFootGlobalPos) / frameTime;
        Vector3 rightFootGlobalVelocity = (rightFoot.transform.position - lastRightFootGlobalPos) / frameTime;
        lastLeftFootGlobalPos = leftFoot.transform.position;
        lastRightFootGlobalPos = rightFoot.transform.position;

        // hip global velocity (one number in r3, 3 numbers)
        Vector3 hipGlobalVel = (hip.transform.position - lastHipGlobalPos) / frameTime;
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
                hip.transform.position.x, 
                hip.transform.position.z,
                hip.transform.rotation.eulerAngles.x,
                hip.transform.rotation.eulerAngles.y,
                hip.transform.rotation.eulerAngles.z,
        };
    }
    private void outputMotionDBToFile()
    {
        string filenameWithoutExtension = filename.Substring(0, filename.Length - 4);
        string newFilename = Application.dataPath + "/outputs/" + filenameWithoutExtension + "_output.txt";
        Debug.Log("Ouputting to : " + newFilename);
        if (File.Exists(newFilename) && clearFileNameOnStart)
        {
            File.Delete(newFilename);
        }
        TextWriter tw = new StreamWriter(newFilename, true);
        tw.WriteLine("[" + System.DateTime.Now + "]");
        int idx = 0;
        foreach (float[] row in motionDB)
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
            tw.WriteLine("Idx " + idx.ToString() + " : " + sb.ToString());
            idx++;
        }
        tw.Close();
    }

    void Start()
    {
        Application.targetFrameRate = targetFramerate;
        if (this.KeepSceneViewActive && Application.isEditor)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
        bp = BVHUtils.parseFile(filename, overrideFrames);
        frameTime = bp.frameTime;
        motionDB = new float[bp.frames][];
        currentFrame = startAtFrame;
        boneToTransformMap = new Dictionary<BVHParser.BVHBone, Transform>();
        BVHUtils.loadTransforms(bp, boneToTransformMap, transform);
    }

    // Update is called once per frame
    void Update()
    {
        if (currentFrame < bp.frames)
        {
            BVHUtils.playFrame(currentFrame, boneToTransformMap, blender);
            motionDB[currentFrame] = getCurrentSearchVector();
            currentFrame++;
        } else if (currentFrame == bp.frames)
        {
            if (!dryRun)
            {
                outputMotionDBToFile();
            }
            currentFrame++;
        }
    }

}
