using System;
using System.Linq;
using UnityEngine;

public static class GeoUtils
{
    // Returns Quat Q such that v1 * q = v2
    public static Quaternion getRotationBetween(Vector3 v1, Vector3 v2)
    {
        Vector3 dir_vec = v2.normalized;
        Vector3 a = Vector3.Cross(v1, dir_vec);
        float w = Mathf.Sqrt(dir_vec.sqrMagnitude * v1.sqrMagnitude) + Vector3.Dot(dir_vec, v1);
        Quaternion q = new Quaternion(a.x, a.y, a.z, w);
        return q.normalized;
    }
    public static Vector3 calculateMean(Vector3[] verts)
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
    public static string matToString(double[,] mat)
    {
        string s = "";
        for (int i = 0; i < mat.GetLength(0); i++)
        {
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

    public static float getMaxDistApart(Vector3[] verts,  ref Vector3 center)
    {
        // O(n) method to test: since we know they'll be on a line determined by a and b,
        // we can set a to go through the origin and shift all other points by subtracting a 
        // from them. Then, we know that that the min and max magnitudes represent the furthest
        // apart points
        float ans = float.NegativeInfinity;
        //center = Vector3.zero;
        for (int i = 0; i < verts.Length; i++)
            for (int j = i + 1; j < verts.Length; j++)
                if ((verts[i] - verts[j]).magnitude > ans) {
                    ans = (verts[i] - verts[j]).magnitude;
                    center = (verts[i] + verts[j]) / 2;
                }
        //ans = Math.Max(ans, (verts[i] - verts[j]).magnitude);
        //Debug.Log("center" + center.ToString());
        return ans;

    }
    public static Vector3[] projectVertsOntoAxis(Vector3[] verts, Vector3 point_a, Vector3 point_b)
    {
        Vector3[] proj_verts = new Vector3[verts.Length];
        for (int i = 0; i < verts.Length; i++)
        {
            proj_verts[i] = closestPointOnLine(point_a, point_b, verts[i]);
        }
        return proj_verts;
    }
    // Projects point b onto line defined by a and b
    public static Vector3 closestPointOnLine(Vector3 a, Vector3 b, Vector3 p)
    {
        Vector3 ap = p - a;
        Vector3 ab = b - a;
        Vector3 result = a + Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab) * ab;
        return result;
    }

    public static double[] doubleCross(double[,] mat, int col_a_idx, int col_b_idx, bool _nomralize = false)
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

    public static void normalize(double[] x)
    {
        double sqrMag = x.Aggregate(0d, (sum, cur) => sum + cur * cur);
        double mag = Math.Sqrt(sqrMag);
        for (int i = 0; i < x.Length; i++)
            x[i] /= mag;
    }

    // mat must be normal (for real mat this means symmetric) 
    /*
     */
    public static Vector3 getEigenvectorFromValue(double[,] mat, double eigenvalue)
    {
        double[,] x = subtractNTimesIdentity(mat, eigenvalue);
        // The cross product of two ind. col of x will be in the null space
        // that is, it will be an eigenvector associated with the eigenvalue
        if (areIndCols(x, 0, 1))
        {
            double[] t = doubleCross(x, 0, 1, true);
            return new Vector3((float)t[0], (float)t[1], (float)t[2]);
            //return Vector3.Cross(convertToVector(x, 0), convertToVector(x, 1));
        }
        else if (areIndCols(x, 0, 2))
        {
            return Vector3.Cross(convertToVector(x, 0), convertToVector(x, 2));
        }
        else if (areIndCols(x, 1, 2))
        {
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

    public static bool colIsZero(double[,] mat, int col_idx)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            if (!Mathf.Approximately((float)mat[i, col_idx], 0))
                return false;
        Debug.Log($"Column {col_idx} is zero");
        return true;
    }
    public static bool matIsZero(double[,] mat)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            for (int j = 0; j < mat.GetLength(1); j++)
                if (!Mathf.Approximately((float)mat[i, j], 0))
                    return false;
        return true;
    }
    public static Vector3 convertToVector(double[,] arr, int col_idx)
    {
        return new Vector3((float)arr[0, col_idx], (float)arr[1, col_idx], (float)arr[2, col_idx]);
    }

    public static bool areIndCols(double[,] mat, int col_a_idx, int col_b_idx)
    {
        double[] col_a = new double[] { mat[0, col_a_idx], mat[1, col_a_idx], mat[2, col_a_idx] };
        double[] col_b = new double[] { mat[0, col_b_idx], mat[1, col_b_idx], mat[2, col_b_idx] };
        if (colIsZero(mat, col_a_idx) || colIsZero(mat, col_b_idx))
        {
            return false;
        }
        double factor = col_b[0] / col_a[0];
        for (int i = 1; i < 3; i++)
            if (!Mathf.Approximately((float)(col_a[i] * factor), (float)col_b[i]))
                return true;
        Debug.Log($"Col idx {col_a_idx} and {col_b_idx} have factor {factor}");
        return false;
    }

    // Given a real symmetric 3x3 matrix A, compute the eigenvalues
    // Note that acos and cos operate on angles in radians
    public static double[] getEigenvalues(double[,] mat)
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

    public static double det(double[,] x)
    {
        return x[0, 0] * x[1, 1] * x[2, 2] +
               x[0, 1] * x[1, 2] * x[2, 0] +
               x[0, 2] * x[1, 0] * x[2, 1] -
               x[0, 2] * x[1, 1] * x[2, 0] -
               x[0, 1] * x[1, 0] * x[2, 2] -
               x[0, 0] * x[1, 2] * x[2, 1];
    }

    public static double[,] matrixScalarMult(double[,] mat, double n)
    {
        double[,] ans = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans[i, j] = mat[i, j] * n;
        return ans;
    }

    // equivalent to A - nI where I is the identity matrix
    public static double[,] subtractNTimesIdentity(double[,] mat, double n = 1)
    {
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
    public static double trace(double[,] mat)
    {
        return mat[0, 0] + mat[1, 1] + mat[2, 2];
    }
    public static double[,] calculateCovarMat(Vector3[] verts)
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
