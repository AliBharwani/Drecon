using System;
using System.Linq;
using UnityEngine;

public static class GeoUtils
{
    // Returns Quat Q such that v1 * q = v2
    public static Quaternion get_rot_between(Vector3 v1, Vector3 v2)
    {
        Vector3 dir_vec = v2.normalized;
        Vector3 a = Vector3.Cross(v1, dir_vec);
        float w = Mathf.Sqrt(dir_vec.sqrMagnitude * v1.sqrMagnitude) + Vector3.Dot(dir_vec, v1);
        Quaternion q = new Quaternion(a.x, a.y, a.z, w);
        return q.normalized;
    }

    public static Vector3 calc_mean(Vector3[] verts)
    {
        Vector3 mean = Vector3.zero;
        foreach (Vector3 vert in verts)
            mean += vert / verts.Length;
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

    public static float get_max_dist_apart(Vector3[] verts,  ref Vector3 center)
    {
        // O(n) method to test: since we know they'll be on a line determined by a and b,
        // we can set a to go through the origin and shift all other points by subtracting a 
        // from them. Then, we know that that the min and max magnitudes represent the furthest
        // apart points
        float ans = float.NegativeInfinity;
        for (int i = 0; i < verts.Length; i++)
            for (int j = i + 1; j < verts.Length; j++)
                if ((verts[i] - verts[j]).magnitude > ans) {
                    ans = (verts[i] - verts[j]).magnitude;
                    center = (verts[i] + verts[j]) / 2;
                }
        return ans;

    }
    public static Vector3[] proj_verts_onto_axis(Vector3[] verts, Vector3 point_a, Vector3 point_b)
    {
        Vector3[] proj_verts = new Vector3[verts.Length];
        for (int i = 0; i < verts.Length; i++)
        {
            proj_verts[i] = closest_point_on_line(point_a, point_b, verts[i]);
        }
        return proj_verts;
    }
    // Projects point p onto line defined by a and b
    public static Vector3 closest_point_on_line(Vector3 a, Vector3 b, Vector3 p)
    {
        Vector3 ap = p - a;
        Vector3 ab = b - a;
        Vector3 result = a + Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab) * ab;
        return result;
    }

    public static double[] cross(double[,] mat, int col_a_idx, int col_b_idx, bool _normalize = false)
    {
        double[] a = new double[] { mat[0, col_a_idx], mat[1, col_a_idx], mat[2, col_a_idx] };
        double[] b = new double[] { mat[0, col_b_idx], mat[1, col_b_idx], mat[2, col_b_idx] };
        var product = new double[] {a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0] };
        if (_normalize)
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
    public static Vector3 get_eigenvector_from_value(double[,] mat, double eigenvalue)
    {
        double[,] x = subtract_n_times_identity(mat, eigenvalue);
        // The cross product of two ind. col of x will be in the null space
        // that is, it will be an eigenvector associated with the eigenvalue
        if (are_ind_cols(x, 0, 1))
        {
            double[] t = cross(x, 0, 1, true);
            return new Vector3((float)t[0], (float)t[1], (float)t[2]);
            //return Vector3.Cross(convertToVector(x, 0), convertToVector(x, 1));
        }
        else if (are_ind_cols(x, 0, 2))
        {
            return Vector3.Cross(col_to_vec(x, 0), col_to_vec(x, 2));
        }
        else if (are_ind_cols(x, 1, 2))
        {
            return Vector3.Cross(col_to_vec(x, 1), col_to_vec(x, 2));
        }
        if (mat_is_zero(x))
            return Vector3.zero;
        int col_idx = col_is_zero(x, 0) ? (col_is_zero(x, 1) ? 2 : 1) : 0;
        // Suppose v is a non zero column of x
        // Choose an arbitrary vector u not parallel to v
        // Then v x u will be perpinduclar to v and thus will be eigenvectors of the eigenvalue
        Vector3 v = col_to_vec(x, col_idx);
        Vector3 u = new Vector3(v.x, v.y, v.z) + Vector3.forward;
        return Vector3.Cross(v, u);
    }

    public static bool col_is_zero(double[,] mat, int col_idx)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            if (!Mathf.Approximately((float)mat[i, col_idx], 0))
                return false;
        Debug.Log($"Column {col_idx} is zero");
        return true;
    }
    public static bool mat_is_zero(double[,] mat)
    {
        for (int i = 0; i < mat.GetLength(0); i++)
            for (int j = 0; j < mat.GetLength(1); j++)
                if (!Mathf.Approximately((float)mat[i, j], 0))
                    return false;
        return true;
    }
    public static Vector3 col_to_vec(double[,] arr, int col_idx)
    {
        return new Vector3((float)arr[0, col_idx], (float)arr[1, col_idx], (float)arr[2, col_idx]);
    }

    public static bool are_ind_cols(double[,] mat, int col_a_idx, int col_b_idx)
    {
        double[] col_a = new double[] { mat[0, col_a_idx], mat[1, col_a_idx], mat[2, col_a_idx] };
        double[] col_b = new double[] { mat[0, col_b_idx], mat[1, col_b_idx], mat[2, col_b_idx] };
        if (col_is_zero(mat, col_a_idx) || col_is_zero(mat, col_b_idx))
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
    public static double[] get_eigenvalues(double[,] mat)
    {
        double[] eigenvalues = new double[3];
        double p1 = Math.Pow(mat[0, 1], 2) + Math.Pow(mat[0, 2], 2) + Math.Pow(mat[1, 2], 2);
        if (Mathf.Approximately((float)p1, 0))
            return new double[] { mat[0, 0], mat[1, 1], mat[2, 2] };
        double q = trace(mat) / 3;
        double p2 = Math.Pow(mat[0, 0] - q, 2) + Math.Pow(mat[1, 1] - q, 2) + Math.Pow(mat[2, 2] - q, 2) + 2 * p1;
        double p = Math.Sqrt(p2 / 6);
        double[,] B = matrix_scalar_mult(subtract_n_times_identity(mat, q), 1 / p);
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
        eigenvalues[0] = eig1;
        eigenvalues[1] = eig2;
        eigenvalues[2] = eig3;
        return eigenvalues;
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

    public static double[,] matrix_scalar_mult(double[,] mat, double n)
    {
        double[,] ans = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans[i, j] = mat[i, j] * n;
        return ans;
    }

    // equivalent to A - nI where I is the identity matrix
    public static double[,] subtract_n_times_identity(double[,] mat, double n = 1)
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

    public static double[,] calc_discrete_covar(Vector3[] verts)
    {
        Vector3 mean = calc_mean(verts);
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

    public static double[,] calculate_continous_covar(Mesh mesh)
    {
        int[] triangles = mesh.triangles;
        Vector3[] verts = mesh.vertices;
        double[,] covar = new double[3, 3];
        float total_area = 0f;
        Vector3 mesh_centroid = Vector3.zero;
        Vector3 a, b, c;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            a = verts[triangles[i]];
            b = verts[triangles[i + 1]];
            c = verts[triangles[i + 2]];
            float area = triangle_area(a, b, c);
            total_area += area;
            mesh_centroid += triangle_centroid(a, b, c) * area;
        }
        mesh_centroid /= total_area;

        float get_vector_idx(Vector3 v, int idx)
        {
            if (idx == 0)
                return v.x;
            else if (idx == 1)
                return v.y;
            else if (idx == 2)
                return v.z;
            throw new Exception($"Interal function incorrectly accesssed - get_centroid_idx: {idx}" );
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < i + 1; j++)
            {
                float covar_ij = 0;
                for (int k = 0; k < triangles.Length; k += 3)
                {
                    a = verts[triangles[k]]; b = verts[triangles[k + 1]]; c = verts[triangles[k + 2]];
                    float area = triangle_area(a, b, c);
                    Vector3 centroid = triangle_centroid(a, b, c);
                    float term_1 = 9 * get_vector_idx(centroid, i) * get_vector_idx(centroid, j);
                    float term_2 = get_vector_idx(a, i) * get_vector_idx(a, j) + get_vector_idx(b, i) * get_vector_idx(b, j) + get_vector_idx(c, i) * get_vector_idx(c, j);
                    covar_ij +=  (area / 12f) * (term_1 + term_2);
                }
                covar_ij /= total_area;
                covar_ij -= get_vector_idx(mesh_centroid, i) * get_vector_idx(mesh_centroid, j);
                covar[i, j] = covar_ij;
                covar[j, i] = covar_ij;
            }
        }

        return covar;
    }

    public static float triangle_area(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ba = b - a;
        Vector3 ca = c - a;
        Vector3 cross = Vector3.Cross(ba, ca);
        return cross.magnitude / 2;
    }

    public static Vector3 triangle_centroid(Vector3 a, Vector3 b, Vector3 c)
    {
        return (a + b + c) / 3;
    }
    // if we have a set's Principal Components, the mean can be the point on the plane and the normal can be the axis 
    public static Vector3 project_point_onto_plane(Vector3 point, Vector3 point_on_plane, Vector3 plane_normal)
    {
        // The projection of a point q = (x, y, z) onto a plane given by a point p = (a, b, c) and a normal n = (d, e, f) is
        // q_proj = q - dot(q - p, n) * n
        // This calculation assumes that n is a unit vector.
        // Since our plane is perpindcular to the axis 
        Vector3 proj_point = point - Vector3.Dot(point - point_on_plane, plane_normal.normalized) * plane_normal.normalized;
        return proj_point;
    }
    public static float GetCapsuleVolume(CapsuleCollider cap)
    {
        // check if it's a sphere
        if (cap.height <= 2 * cap.radius)
            return (4f/3f) * (float)Math.PI * (float)Math.Pow(cap.radius, 3);
        float a = cap.height - 2 * cap.radius;
        float volume = Mathf.PI * cap.radius * cap.radius * ((4f / 3f) * cap.radius + a);
        return volume;
    }
    public static double wrap_angle(double angle)
    {
        angle %= 360;
        return angle > 180 ? angle - 360 : angle;
    }

    public static float wrap_radians(float rads)
    {
        rads %= Mathf.PI;
        return rads > Mathf.PI / 2 ? rads - Mathf.PI : rads;
    }
    public static float wrap_anglef (float angle) // f for float
    {
        angle %= 360;
        return angle > 180 ? angle - 360 : angle;
    }

    // Computes distance between Point C and line segment created by Point A and B 
    // Implementation from realtimecollision textbook 
    public static float dist_bt_point_and_seg(Vector3 c, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a, ac = c - a, bc = c - b;
        float e = Vector3.Dot(ac, ab);
        // Handle cases where closest point is a or b
        if (e <= 0f) return ac.magnitude;
        float f = ab.sqrMagnitude;
        if (e >= f) return bc.magnitude;
        // C projects onto ab
        return Mathf.Sqrt(ac.sqrMagnitude - e * e / f);
    }
}
