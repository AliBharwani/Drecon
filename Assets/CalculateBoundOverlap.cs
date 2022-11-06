using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static mm_v2.Bones;
public class CalculateBoundOverlap : MonoBehaviour
{

    public float INC = .001f;
    public BoxCollider[] boxes;
    [ContextMenu("Calculate bounding overlaps")]
    void calculateBoundingOverlaps()
    {
        int [][] bone_to_points = new int[23][];

        // idx 0 = total points,1 = points only in this bone, 2 = points shared bt 1 bones, 3 = points shared bt 2 bones
        for (int i = 0; i < bone_to_points.Length; i++)
            bone_to_points[i] = new int[4];
        
        foreach(BoxCollider box in boxes)
        {
            //float min_x, max_x, min_y, max_y, min_z, max_z;
            Vector3 mins = transform.position + box.center - box.size;
            Vector3 maxes = transform.position + box.center + box.size;
            for (float x = mins.x; x < maxes.x; x += INC)
                for (float y = mins.y; x < maxes.y; x += INC)
                    for (float z = mins.z; x < maxes.z; x += INC)
                    {
                        Vector3 point = new Vector3(x, y, z);
                        int num_collisions = 0;
                        for (int i = 0; i < bone_to_points.Length; i++)
                            num_collisions += check_collision(point, (mm_v2.Bones)i) ? 1 : 0;
                        if (num_collisions > 0)
                            for (int i = 0; i < bone_to_points.Length; i++)
                                bone_to_points[i][num_collisions] += check_collision(point, (mm_v2.Bones)i) ? 1 : 0;

                    }   
        } 


    }

    private bool check_collision(Vector3 point, mm_v2.Bones bone)
    {
        /*
         Get cardinal line for capsule collider 
         */
        return false;
    }


}

public static class CapsuleColliderUtils
{
    public static void collision_check(this CapsuleCollider cap, Vector3 point)
    {
        Vector3 center = cap.center;
        float halfHeight = cap.height / 2 - cap.radius;
        var top = cap.transform.TransformPoint(center + halfHeight * Vector3.right);
        var bottom = cap.transform.TransformPoint(center - halfHeight * Vector3.right);
         GeoUtils.
    }
} 