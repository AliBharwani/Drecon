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
        Gizmos.DrawLine(start, meanPoint + testVec);
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
    private void createBoundingCapsule_2(float height, float rad, Quaternion orientation)
    {

    }

    // Returns Quat Q such that v1 * q = v2
    private Quaternion getRotationBetween(Vector3 v1, Vector3 v2)
    {
        Vector3 dir_vec = v2.normalized;
        Vector3 a = Vector3.Cross(v1, dir_vec);
        float w = Mathf.Sqrt(dir_vec.sqrMagnitude * v1.sqrMagnitude) + Vector3.Dot(dir_vec, v1);
        Quaternion q = new Quaternion(a.x, a.y, a.z, w);
        return q.normalized;
    }
    private Vector3 calculateMean(Vector3[] verts)
    {
        int n = verts.Length;
        Vector3 mean = new Vector3();
        foreach (Vector3 vert in verts)
        {
            mean.x += vert.x / n;
            mean.y += vert.y / n;
            mean.z += vert.z / n;
        }
        return mean;
    }
    private string matToString(double[,] mat)
    {
        string s = "";
        for (int i = 0; i < mat.GetLength(0); i++) {
            s += "[ ";
            for (int j = 0; j < mat.GetLength(1); j++)
            {
                s += mat[i, j].ToString();
                if (j != mat.GetLength(1) - 1)
                    s += " , ";
            }
            s += " ]";
        }
        return s;
    }
    //[ContextMenu("CALCULATE bounding capsule")]
    public void calculateBoundingCapsule(Vector3[] verts)
    {
        int n = verts.Length;
        Vector3 mean = calculateMean(verts);
        Debug.Log("Mean: " + mean.ToString("f6"));
        double[,] covar = calculateCovarMat(verts);
        Debug.Log("covar: " + matToString(covar));
        double[] eigenvalues = getEigenvalues(covar);
        BVHUtils.debugArray(eigenvalues, "Eigenvalues: ");
        Vector3 largest_eigen = getEigenvectorFromValue(covar, eigenvalues[0]).normalized;
        Debug.Log("largest_eigen : " + largest_eigen.ToString("f6"));
        Vector3 second_eigen = getEigenvectorFromValue(covar, eigenvalues[1]);
        Vector3[] proj_verts = projectVertsOntoAxis(verts, mean, mean + largest_eigen);
        float height = getMaxDistApart(proj_verts);
        return;
        // Calculate radius using iterative refinement
        double dist_from_main_axis_sum = 0;
        foreach (Vector3 v in verts)
            dist_from_main_axis_sum += (v - closestPointOnLine(mean, mean + largest_eigen, v)).magnitude;
        float radius = (float) dist_from_main_axis_sum / n;
        createBoundingCapsule_2(height, radius, getRotationBetween(Vector3.right, largest_eigen));
    }

    private float getMaxDistApart(Vector3[] verts)
    {
        // O(n) method to test: since we know they'll be on a line determined by a and b,
        // we can set a to go through the origin and shift all other points by subtracting a 
        // from them. Then, we know that that the min and max magnitudes represent the furthest
        // apart points
        float ans = float.NegativeInfinity;
        for (int i = 0; i < verts.Length; i++)
            for (int j = i + 1; j < verts.Length; j++)
                ans = Math.Max(ans, (verts[i] - verts[j]).magnitude);
        return ans;

    }

    private Vector3[] projectVertsOntoAxis(Vector3[] verts, Vector3 point_a, Vector3 point_b)
    {
        Vector3[] proj_verts = new Vector3[verts.Length];
        for(int i = 0; i< verts.Length; i++)
        {
            proj_verts[i] = closestPointOnLine(point_a, point_b, verts[i]);
        }
        return proj_verts;
    }
    // Projects point b onto line defined by a and b
    private Vector3 closestPointOnLine(Vector3 a, Vector3 b, Vector3 p)
    {
        Vector3 ap = p - a;
        Vector3 ab = b - a;
        Vector3 result = a + Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab) * ab;
        return result;
    }

    private double[] doubleCross(double[,] mat, int col_a_idx, int col_b_idx, bool _nomralize = false)
    {
        double[] a = new double[] { mat[0, col_a_idx], mat[1, col_a_idx], mat[2, col_a_idx] };
        double[] b = new double[] { mat[0, col_b_idx], mat[1, col_b_idx], mat[2, col_b_idx] };
        var product = new double[] {a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0] };
        if (_nomralize)
            normalize(product);
        return product;
    }

    private void normalize(double[] x)
    {
        double sqrMag = x.Aggregate(0d, (sum, cur) => sum + cur * cur);
        double mag = Math.Sqrt(sqrMag);
        for (int i = 0; i < x.Length; i++)
            x[i] /= mag;
    }

    // mat must be normal (for real mat this means symmetric) 
    /*
     */
    private Vector3 getEigenvectorFromValue(double[,] mat, double eigenvalue)
    {
        double[,] x = subtractNTimesIdentity(mat, eigenvalue);
        // The cross product of two ind. col of x will be in the null space
        // that is, it will be an eigenvector associated with the eigenvalue
        if (areIndCols(x, 0, 1))
        {
            Debug.Log("0 and 1 are ind cols");
            //Debug.Log($"Column 0 is " + convertToVector(x, 0).ToString("f6"));
            //Debug.Log($"Column 1 is " + convertToVector(x, 1).ToString("f6"));
            double[] t = doubleCross(x, 0, 1, true);
            Debug.Log($"Double Cross results: {t[0]} , {t[1]}, {t[2]}");
            return new Vector3((float) t[0], (float)t[1], (float) t[2]);
            //return Vector3.Cross(convertToVector(x, 0), convertToVector(x, 1));
        } else if (areIndCols(x, 0, 2))
        {
            Debug.Log("0 and 2 are ind cols");

            return Vector3.Cross(convertToVector(x, 0), convertToVector(x, 2));
        } else if (areIndCols(x, 1, 2))
        {
            Debug.Log("1 and 2 are ind cols");

            return Vector3.Cross(convertToVector(x, 1), convertToVector(x, 2));
        }
        if (matIsZero(x))
            return Vector3.zero;
        int col_idx = colIsZero(x, 0) ? (colIsZero(x, 1) ? 2 : 1) : 0;
        // Suppose v is a non zero column of x
        // Choose an arbitrary vector u not parallel to v
        // Then v x u will be perpinduclar to v and thus will be eigenvectors of the eigenvalue
        Vector3 v = convertToVector(x, col_idx);
        Vector3 u = new Vector3(v.x, v.y, v.z) + Vector3.forward;
        return Vector3.Cross(v, u);
    }

    private bool colIsZero(double[,] mat, int col_idx)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            if (!Mathf.Approximately((float)mat[i, col_idx], 0))
                return false;
        Debug.Log($"Column {col_idx} is zero");
        return true;
    }
    private bool matIsZero(double[,] mat)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            for (int j = 0; j < mat.GetLength(1); j++)
                if (!Mathf.Approximately((float)mat[i, j], 0))
                    return false;
        return true;
    }
    private Vector3 convertToVector(double[,] arr, int col_idx)
    {
        return new Vector3((float)arr[0, col_idx], (float)arr[1, col_idx], (float)arr[2, col_idx]);
    }

    private bool areIndCols(double[,] mat, int col_a_idx, int col_b_idx)
    {
        double[] col_a = new double[] { mat[0, col_a_idx], mat[1, col_a_idx], mat[2, col_a_idx] };
        double[] col_b = new double[] { mat[0, col_b_idx], mat[1, col_b_idx], mat[2, col_b_idx] };
        if (colIsZero(mat, col_a_idx) || colIsZero(mat, col_b_idx))
        {
            return false;
        }
        double factor = col_b[0] / col_a[0];
        for (int i = 1; i < 3; i++)
            if (!Mathf.Approximately((float) ( col_a[i] * factor), (float) col_b[i]))
                return true;
        Debug.Log($"Col idx {col_a_idx} and {col_b_idx} have factor {factor}");
        return false;
    }

    // Given a real symmetric 3x3 matrix A, compute the eigenvalues
    // Note that acos and cos operate on angles in radians
    private double[] getEigenvalues(double[,] mat)
    {
        double[] eigenValues = new double[3];
        double p1 = Math.Pow(mat[0, 1], 2) + Math.Pow(mat[0, 2], 2) + Math.Pow(mat[1, 2], 2);
        if (Mathf.Approximately((float)p1, 0))
            return new double[] { mat[0, 0], mat[1, 1], mat[2, 2] };
        double q = trace(mat) / 3;
        double p2 = Math.Pow(mat[0, 0] - q, 2) + Math.Pow(mat[1, 1] - q, 2) + Math.Pow(mat[2, 2] - q, 2) + 2 * p1;
        double p = Math.Sqrt(p2 / 6);
        double[,] B = matrixScalarMult(subtractNTimesIdentity(mat, q), 1 / p);
        double r = det(B) / 2;
        // In exact arithmetic for a symmetric matrix - 1 <= r <= 1
        //but computation error can leave it slightly outside this range.
        double phi;
        if (r <= -1)
            phi = Math.PI / 3;
        else if (r >= 1)
            phi = 0;
        else
            phi = Math.Acos(r) / 3;
        // the eigenvalues satisfy eig3 <= eig2 <= eig1
        double eig1 = q + 2 * p * Math.Cos(phi);
        double eig3 = q + 2 * p * Math.Cos(phi + (2 * Math.PI / 3));
        double eig2 = 3 * q - eig1 - eig3; // since trace(A) = eig1 + eig2 + eig
        eigenValues[0] = eig1;
        eigenValues[1] = eig2;
        eigenValues[2] = eig3;
        return eigenValues;
    }

    private double det(double[,] x)
    {
        return x[0, 0] * x[1, 1] * x[2, 2] +
               x[0, 1] * x[1, 2] * x[2, 0] +
               x[0, 2] * x[1, 0] * x[2, 1] -
               x[0, 2] * x[1, 1] * x[2, 0] -
               x[0, 1] * x[1, 0] * x[2, 2] -
               x[0, 0] * x[1, 2] * x[2, 1];
    }

    private double[,] matrixScalarMult(double[,] mat, double n)
    {
        double[,] ans = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans[i, j] = mat[i, j] * n;
        return ans;
    }

    // equivalent to A - nI where I is the identity matrix
    private double[,] subtractNTimesIdentity(double[,] mat, double n = 1) {
        double[,] ans = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (i == j)
                    ans[i, j] = mat[i, j] - n;
                else
                    ans[i, j] = mat[i, j];
        return ans;
    }
    // trace(A) is the sum of all diagnol values
    private double trace(double[,] mat)
    {
        return mat[0, 0] + mat[1, 1] + mat[2, 2];
    }
    private double[,] calculateCovarMat(Vector3[] verts)
    {
        Vector3 mean = calculateMean(verts);
        int n = verts.Length;
        double[,] covar = new double[3, 3];
        // [ covar(x,x) , covar(x,y), covar(x,z) ]
        // [ covar(y,x) , covar(y,y), covar(y,z) ]
        // [ covar(z,x) , covar(z,y), covar(z,z) ]
        foreach (Vector3 vert in verts)
        {
            double x_var = vert.x - mean.x;
            double y_var = vert.y - mean.y;
            double z_var = vert.z - mean.z;
            // variances go along diagnols
            covar[0, 0] += Math.Pow(x_var, 2) / n;
            covar[1, 1] += Math.Pow(y_var, 2) / n;
            covar[2, 2] += Math.Pow(z_var, 2) / n;
            // do one side of the symmetric matrix
            covar[0, 1] += x_var * y_var / n;
            covar[0, 2] += x_var * z_var / n;
            covar[1, 2] += y_var * z_var / n;
        }
        // copy over symmetric values
        covar[1, 0] = covar[0, 1];
        covar[2, 0] = covar[0, 2];
        covar[2, 1] = covar[1, 2];
        return covar;
    }
}
