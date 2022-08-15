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
        Gizmos.DrawLine(start, meanPoint + testVec);
        Gizmos.DrawLine(start, meanPoint + testVec2);

        Gizmos.DrawSphere(start, .01f);


        Gizmos.color = Color.red;
        Gizmos.DrawLine(start, transform.TransformPoint(Vector3.up));
    }
    public string rotString = "0.0, 0.03721865792682488, 0.6616028889689546, -0.7489301628390053";
    public bool abTest = false;
    [ContextMenu("Create bounding capsule")]
    private void createBoundingCapsule()
    {
        /* Read in height, mean, radius, direction, and orientation
         * 
         */
        float height = 0.236745082931f;
        float rad = 0.05830464461985159f;
        //Quaternion rot2 = BVHUtils.convertToQuat("-0.0, 0.03721865792682488, 0.6616028889689546, -0.7489301628390053");
        //Quaternion rot = BVHUtils.convertToQuat("-0.0, 0.08182367261118306, 1.4545064545863615, 0.5519666023099086");
        Quaternion rot = BVHUtils.convertToQuat(rotString);
        if (abTest)
        {
            rot.y *= -1;
            rot.z *= -1;
        }
        Vector3 mean = BVHUtils.convertToVec("0.647868  ,  1.44229541, -0.0308206 ");
        GameObject capsuleObject = new GameObject();
        capsuleObject.transform.parent = transform;

        capsuleObject.transform.localPosition = Vector3.zero;
        //capsuleObject.transform.localRotation = Quaternion.identity;
        capsuleObject.transform.rotation = rot;
        capsuleObject.transform.localScale = Vector3.one;

        CapsuleCollider capsule = capsuleObject.AddComponent<CapsuleCollider>() as CapsuleCollider;
        capsule.center = transform.InverseTransformPoint(mean);
        capsule.height = height;
        capsule.direction = 0;
        capsule.radius = rad;
    }
}
