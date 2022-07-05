using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System;

public static class BVHUtils
    {
    // BVH to Unity

    public static Quaternion fromEulerZXY(Vector3 euler)
    {
        return Quaternion.AngleAxis(euler.z, Vector3.forward) * Quaternion.AngleAxis(euler.y, Vector3.up) * Quaternion.AngleAxis(euler.x, Vector3.right);
    }

    public static float wrapAngle(float a)
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

    public static void printDictionary<T1, T2>(Dictionary<T1, T2> dict)
    {
        foreach (KeyValuePair<T1, T2> kvp in dict)
        {
            Debug.Log(string.Format("Key = {0}, Value = {1}", kvp.Key, kvp.Value));
        }
    }

    enum Channel
    {
        XPos,
        YPos,
        ZPos,
        XRot,
        YRot,
        ZRot
    }

    // 0 = Xpos, 1 = Ypos, 2 = Zpos, 3 = Xrot, 4 = Yrot, 5 = Zrot
    private static float getAtFrame(BVHParser.BVHBone bone, Channel c, int frame)
    {
        switch (c)
        {
            case Channel.XPos:
                return bone.channels[0].values[frame] * .01f;
            case Channel.YPos:
                return bone.channels[1].values[frame] * .01f;
            case Channel.ZPos:
                return bone.channels[2].values[frame] * .01f;
            case Channel.XRot:
                return bone.channels[3].values[frame];
            case Channel.YRot:
                return bone.channels[4].values[frame];
            case Channel.ZRot:
                return bone.channels[5].values[frame];
        }
        throw new InvalidOperationException("getAtCurrentFrame called with invalid params: " + bone.ToString() + " , Channel: " + c);
    }

    private static Vector3 getDifferenceInPosition(BVHParser.BVHBone bone, int frame)
    {
        float xPos = -(getAtFrame(bone, Channel.XPos, frame) - getAtFrame(bone, Channel.XPos, frame - 1));
        float yPos = 0;// getAtFrame(bone, Channel.YPos, frame) - getAtFrame(bone, Channel.YPos, frame - 1);
        float zPos = getAtFrame(bone, Channel.ZPos, frame) - getAtFrame(bone, Channel.ZPos, frame - 1);
        return new Vector3(xPos, yPos, zPos);
    }
    public static void playFrame(int frame, Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap, bool blender =true, bool applyMotion =true)
    {
        //Debug.Log("Playing frame: " + currentFrame);
        bool first = false;
        foreach (KeyValuePair<BVHParser.BVHBone, Transform> kvp in boneToTransformMap)
        {
            BVHParser.BVHBone bone = kvp.Key;
            Transform curTransform = kvp.Value;
            first = bone.channels[0].enabled;
            // cheating here - we know that only hips will have pos data
            if (applyMotion && first && frame > 0) // update position
            {
                if (blender)
                {
                    curTransform.position += getDifferenceInPosition(bone, frame);
                }
                else
                {
                    Vector3 bonePos = new Vector3(-getAtFrame(bone, Channel.XPos, frame), getAtFrame(bone, Channel.YPos, frame), getAtFrame(bone, Channel.ZPos, frame));
                    Vector3 bvhPosition = curTransform.parent.InverseTransformPoint(bonePos + curTransform.parent.position);
                    curTransform.localPosition = bvhPosition;
                }

            }
            // Update rotation
            float xRot = wrapAngle(getAtFrame(bone, Channel.XRot, frame));
            float yRot = wrapAngle(getAtFrame(bone, Channel.YRot, frame));
            float zRot = wrapAngle(getAtFrame(bone, Channel.ZRot, frame));
            Vector3 eulerBVH = new Vector3(xRot, yRot, zRot);
            Quaternion rot = fromEulerZXY(eulerBVH);
            curTransform.localRotation = new Quaternion(rot.x, -rot.y, -rot.z, rot.w);
        }

    }

    // This function doesn't call any Unity API functions and should be safe to call from another thread
    public static BVHParser parseFile(string filename, int overrideFrames = -1)
    {
        BVHParser bp = new BVHParser(File.ReadAllText(Application.dataPath + "/LAFLAN Data/Animations/" + filename), overrideFrames);
        return bp;
    }

    public static void loadTransforms(BVHParser bp, Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap, Transform transform)
    {
        Dictionary<string, Transform> nameToTransformMap = new Dictionary<string, Transform>();
        Queue<Transform> transforms = new Queue<Transform>();

        transforms.Enqueue(transform);
        while (transforms.Any())
        {
            Transform curTransform = transforms.Dequeue();
            nameToTransformMap.Add(curTransform.name, curTransform);
            for (int i = 0; i < curTransform.childCount; i++)
            {
                transforms.Enqueue(curTransform.GetChild(i));
            }
        }
        
        foreach (BVHParser.BVHBone bone in bp.boneList)
        {
            boneToTransformMap.Add(bone, nameToTransformMap[bone.name]);
        }
    }

    public static void lerp(BVHParser bp, Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap, Transform transform, int frameIdx)
    {

    }

    public static void interializationBlend(BVHParser bp, Dictionary<BVHParser.BVHBone, Transform> boneToTransformMap, Transform transform, int frameIdx)
    {

    }

    public static void debugArray<T>(T[] data, string name)
    {
        Debug.Log(name + string.Join(",", data));
    }
}
