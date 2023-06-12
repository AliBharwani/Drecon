using UnityEngine;

public class AngleLossTracker : MonoBehaviour
{
    public GameObject target;
    Transform[] targetRots;
    public int numFramesToReport = 300;
    Transform[] self;
    int frameIdx = 1;
    float totalAngleDiff = 0f;
    void Start()
    {
        targetRots = target.GetComponent<SimCharController>().boneToTransform;
        self = gameObject.GetComponent<SimCharController>().boneToTransform;
    }

    void FixedUpdate()
    {
        float angleDiffForThisFrame = 0f;
        for(int i = 1; i < 23; i++)
        {
            angleDiffForThisFrame += Quaternion.Angle(self[i].localRotation, targetRots[i].localRotation);
        }
        totalAngleDiff += angleDiffForThisFrame / (float)numFramesToReport;
        if (frameIdx % numFramesToReport == 0)
        {
            Debug.Log($"Total Angle Diff: {totalAngleDiff}");
            totalAngleDiff = 0f;
        }
        frameIdx++;
    }
}
