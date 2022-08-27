using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class debugScript : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    private void OnDrawGizmos()
    {
        string mean = "0.647868  ,  1.44229543, -0.03082059";
        Vector3 meanPoint = BVHUtils.convertToVec(mean);
        string test = "-0.99862385, -0.01221036,  0.05100307";
        string test2 = "-0.04758921, -0.19768843, -0.97910906";
        Vector3 testVec = BVHUtils.convertToVec(test).normalized;
        Vector3 testVec2 = BVHUtils.convertToVec(test2).normalized;

        Gizmos.color = Color.blue;
        Vector3 start = meanPoint;//Vector3.zero; // transform.TransformPoint(Vector3.zero);
        Vector3 end = transform.position + testVec2; // transform.TransformPoint(testVec);
        //Gizmos.DrawLine(start, meanPoint + testVec);
        //Gizmos.DrawLine(start, meanPoint + testVec2);

        Gizmos.DrawSphere(start, .01f);


        Gizmos.color = Color.red;
        //Gizmos.DrawLine(transform.position, transform.position + Vector3.right);
        //Gizmos.DrawLine(start, transform.TransformPoint(Vector3.up));
    }
    public string rotString = "0.0, 0.03721865792682488, 0.6616028889689546, -0.7489301628390053";
    public bool abTest = false;
    //[ContextMenu("Create bounding capsule")]
    private void createBoundingCapsule()
    {
        /* Read in height, mean, radius, direction, and orientation
         * 
         */
        float height = 0.26763651690777457f;
        float rad = 0.04052660788170391f;
        //Quaternion rot2 = BVHUtils.convertToQuat("-0.0, 0.03721865792682488, 0.6616028889689546, -0.7489301628390053");
        //Quaternion rot = BVHUtils.convertToQuat("-0.0, 0.08182367261118306, 1.4545064545863615, 0.5519666023099086");
        Quaternion rot = BVHUtils.convertToQuat(rotString);
        //Vector3 lookAt = BVHUtils.convertToVec("-0.99862384, -0.01221104,  0.05100308") + BVHUtils.convertToVec("0.647868  ,  1.44229541, -0.0308206 ");
        //rot = Quaternion.LookRotation(lookAt, Vector3.up);
        if (abTest)
        {
            rot.y *= -1;
            rot.z *= -1;
        }
        Vector3 mean = BVHUtils.convertToVec("0.647868  ,  1.44229541, -0.0308206 ");
        GameObject capsuleObject = new GameObject();
        capsuleObject.transform.parent = transform;
        capsuleObject.transform.position = mean;
        //capsuleObject.transform.localPosition = Vector3.zero;
        //capsuleObject.transform.rotation = Quaternion.identity; 
        capsuleObject.transform.rotation = rot;
        //capsuleObject.transform.rotation = rot;
        capsuleObject.transform.localScale = Vector3.one;

        CapsuleCollider capsule = capsuleObject.AddComponent<CapsuleCollider>() as CapsuleCollider;
        //capsule.center = transform.InverseTransformPoint(mean);
        capsule.height = height;
        capsule.direction = 0;
        capsule.radius = rad;
    }
    private void createBoundingCapsule_2(float height, float rad, Quaternion orientation, Vector3 center)
    {
        GameObject capsuleObject = new GameObject();
        capsuleObject.transform.parent = transform;
        capsuleObject.transform.position = center;
        capsuleObject.transform.rotation = orientation;
        CapsuleCollider capsule = capsuleObject.AddComponent<CapsuleCollider>();
        //capsule.center = transform.InverseTransformPoint(mean);
        capsule.height = height;
        capsule.direction = 0;
        capsule.radius = rad;

    }


    [ContextMenu("CALCULATE bounding capsule")]
    public void calculateBoundingCapsule(Vector3[] verts)
    {
        int n = verts.Length;
        //Vector3[] verts = new Vector3[n];
        //for (int i = 0; i < n; i++)
        //    verts[i] = transform.position + local_verts[i];
        Vector3 mean = GeoUtils.calculateMean(verts);
        //Debug.Log("Mean: " + mean.ToString("f6"));
        double[,] covar = GeoUtils.calculateCovarMat(verts);
        //Debug.Log("covar: " + matToString(covar));
        double[] eigenvalues = GeoUtils.getEigenvalues(covar);
        BVHUtils.debugArray(eigenvalues, "Eigenvalues: ");
        Vector3 center = Vector3.zero;
        Vector3 largest_eigen = GeoUtils.getEigenvectorFromValue(covar, eigenvalues[0]).normalized;
        //Debug.Log("largest_eigen : " + largest_eigen.ToString("f6"));
        //return;
        Vector3[] proj_verts = GeoUtils.projectVertsOntoAxis(verts, mean, mean + largest_eigen);

        // center should be at middle of the two furthest points
        float height = GeoUtils.getMaxDistApart(proj_verts, ref center);
        // Calculate radius using mean
        //double dist_from_main_axis_sum = 0;
        //foreach (Vector3 v in verts)
        //    dist_from_main_axis_sum += (v - GeoUtils.closestPointOnLine(mean, mean + largest_eigen, v)).magnitude;
        //float radius = (float) dist_from_main_axis_sum / n;
        double max_dist_from_main_axis = 0;
        foreach (Vector3 v in verts)
            max_dist_from_main_axis = Math.Max(max_dist_from_main_axis , (v - GeoUtils.closestPointOnLine(mean, mean + largest_eigen, v)).magnitude);
        float radius = (float)max_dist_from_main_axis;


        createBoundingCapsule_2(height, radius, GeoUtils.getRotationBetween(Vector3.right, largest_eigen), center);
    }    
}

/*
 0 hip
1 spine
2 spine1
3 spine2
4 neck (contains all head verts basically)
5 head
6 right shoulder
7 right arm
8 right forearm
9 right hand
[10, 32] right fingers
33 left shoulder
34 left arm 
35 left forearm
36 left hand
[37, 59] left fingers
60 right up leg
61 right leg 
62 right foot
63 right toe
64 right toe end
65 left up leg
66 left leg 
67 left foot
68 left toe 
69 left toe end

 
 */