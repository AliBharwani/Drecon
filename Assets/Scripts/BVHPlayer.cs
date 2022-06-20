using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

public class BVHPlayer : MonoBehaviour
{

    [Tooltip("This is the path to the BVH file that should be loaded. Bone offsets are currently being ignored by this loader.")]
    public string filename;

    public bool KeepSceneViewActive = true;
    public float scale = .01f;
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

    private BVHParser bp = null;
    private int currentFrame = 0;
    // the frametime from the BVH file 
    private float frameTime;
    private Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap;

    private Vector3 lastLeftFootGlobalPos;
    private Vector3 lastRightFootGlobalPos;
    private Vector3 lastHipGlobalPos;

    enum Channel
    {
        XPos,
        YPos,
        ZPos,
        XRot,
        YRot,
        ZRot
    }

    // BVH to Unity
    private Quaternion fromEulerZXY(Vector3 euler)
    {
        return Quaternion.AngleAxis(euler.z, Vector3.forward)  * Quaternion.AngleAxis(euler.y, Vector3.up) * Quaternion.AngleAxis(euler.x, Vector3.right);
    }

    private float wrapAngle(float a)
    {
        if (a > 180f)
        {
            return a - 360f;
        }
        if (a < -180f)
        {
            return 360f + a;
        }
        return a;
    }

    // 0 = Xpos, 1 = Ypos, 2 = Zpos, 3 = Xrot, 4 = Yrot, 5 = Zrot
    private float getAtFrame(BVHParser.BVHBone bone, Channel c, int frame)
    {
        switch(c)
        {
            case Channel.XPos:
                return bone.channels[0].values[frame] * scale;
            case Channel.YPos:
                return bone.channels[1].values[frame] * scale;
            case Channel.ZPos:
                return bone.channels[2].values[frame] * scale;
            case Channel.XRot:
                return bone.channels[3].values[frame];
            case Channel.YRot:
                return bone.channels[4].values[frame];
            case Channel.ZRot:
                return bone.channels[5].values[frame];
        }
        throw new InvalidOperationException("getAtCurrentFrame called with invalid params: " + bone.ToString() + " , Channel: " + c);
    }

    private float getAtCurrentFrame(BVHParser.BVHBone bone, Channel c)
    {
        return getAtFrame(bone, c, currentFrame);
    }

    private float getAtLastFrame(BVHParser.BVHBone bone, Channel c)
    {
        return getAtFrame(bone, c, currentFrame - 1);
    }

    private Vector3 getDifferenceInPosition(BVHParser.BVHBone bone)
    {
        float xPos = -(getAtCurrentFrame(bone, Channel.XPos) - getAtLastFrame(bone, Channel.XPos));
        float yPos = getAtCurrentFrame(bone, Channel.YPos) - getAtLastFrame(bone, Channel.YPos);
        float zPos = getAtCurrentFrame(bone, Channel.ZPos) - getAtLastFrame(bone, Channel.ZPos);
        return new Vector3(xPos, yPos, zPos);
    }
    private void playCurrentFrame()
    {
        //Debug.Log("Playing frame: " + currentFrame);
        bool first = false;
        foreach (KeyValuePair<BVHParser.BVHBone, Transform> kvp in boneToTransformMap) {
           BVHParser.BVHBone bone = kvp.Key;
           Transform curTransform = kvp.Value;
           first = bone.channels[0].enabled;
           // cheating here - we know that only hips will have pos data
           if (first && currentFrame > 0) // update position
           {
                if (blender) {
                    curTransform.position += getDifferenceInPosition(bone);
                }
                else
                {
                    Vector3 bvhPosition = curTransform.parent.InverseTransformPoint(new Vector3(-getAtCurrentFrame(bone, Channel.XPos), getAtCurrentFrame(bone, Channel.YPos), getAtCurrentFrame(bone, Channel.ZPos)) + curTransform.parent.position);
                    curTransform.localPosition = bvhPosition;
                }

           }
           // Update rotation
           Vector3 eulerBVH = new Vector3(wrapAngle(getAtCurrentFrame(bone, Channel.XRot)), wrapAngle(getAtCurrentFrame(bone, Channel.YRot)), wrapAngle(getAtCurrentFrame(bone, Channel.ZRot)));
           Quaternion rot = fromEulerZXY(eulerBVH);
           curTransform.localRotation = new Quaternion(rot.x, -rot.y, -rot.z, rot.w);
        }
         
    }


    private void loadTransforms()
    {
        if (bp == null)
        {
            throw new InvalidOperationException("No BVH file has been parsed.");
        }
        Dictionary<string, Transform> nameToTransformMap = new Dictionary<string, Transform>();
        boneToTransformMap = new Dictionary<BVHParser.BVHBone, Transform>();
        Queue<Transform> transforms = new Queue<Transform>();

        transforms.Enqueue(transform);
        while (transforms.Any())
        {
            Transform transform = transforms.Dequeue();
            nameToTransformMap.Add(transform.name, transform);
            for (int i = 0; i < transform.childCount; i++)
            {
                transforms.Enqueue(transform.GetChild(i));
            }
        }
        // 
        foreach (BVHParser.BVHBone bone in bp.boneList)
        {
            boneToTransformMap.Add(bone, nameToTransformMap[bone.name]);
        }
    }

    // This function doesn't call any Unity API functions and should be safe to call from another thread
    public void parseFile()
    {
        bp = new BVHParser(File.ReadAllText(Application.dataPath + "/LAFLAN Data/Animations/" +  filename), overrideFrames);
        frameTime = bp.frameTime;
        Debug.Log("Frame time: " + frameTime);
    }

    private static void printDictionary<T1, T2>(Dictionary<T1, T2> dict)
    {
           foreach (KeyValuePair<T1, T2> kvp in dict)
        {
            Debug.Log(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
        }
    }

    private float[] getCurrentSearchVector()
    {
        if (lastLeftFootGlobalPos == null)
        {
            lastLeftFootGlobalPos = leftFoot.transform.position;
            lastRightFootGlobalPos = rightFoot.transform.position;
            lastHipGlobalPos = hip.transform.position;
            return new float[20];
        }

        // get left and right foot local positions and global velocities
        // 2 pairs (left and right) of 2 vectors in r^3, 3 numbers
        Vector3 leftFootLocalPos = leftFoot.transform.position - root.transform.position;
        Vector3 rightFootLocalPos = rightFoot.transform.position - root.transform.position;
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
        parseFile();
        motionDB = new float[bp.frames][];
        loadTransforms();
    }

    // Update is called once per frame
    void Update()
    {
        if (currentFrame < bp.frames)
        {
            playCurrentFrame();
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
