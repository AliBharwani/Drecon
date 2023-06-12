using UnityEngine;
using System;
using System.IO;
using System.Text;
using static MotionMatchingAnimator.Bones;
public class MocapDB : MonoBehaviour
{
    public bool use60fps = true;
    [HideInInspector] // Hides var below
    public Vector3[][] bone_positions;
    [HideInInspector]
    public Vector3[][] bone_velocities;
    [HideInInspector]
    public Quaternion[][] bone_rotations;
    [HideInInspector]
    public Vector3[][] bone_angular_velocities;
    [HideInInspector]
    public bool[][] contact_states;
    internal int[] bone_parents;
    internal int numframes;

    internal int[] range_starts;
    internal int[] range_stops;

    internal float[][] features;
    internal float[]  features_offset;
    internal float[] features_scale;
    int offset;
    int num_neigh;
    KDTree tree;
    private Quaternion inv_sim_rot(int frame)
    {
        return MathUtils.quat_inv(bone_rotations[frame][0]);
    }

    public int motionMatch(float[] query)
    {
        // Assume query is already normalized
        float[] bestEntry = tree.nnSearch(query);
        int best_idx = (int) bestEntry[bestEntry.Length - 1];
        return best_idx;
    }

    public int database_trajectory_index_clamp(int frame, int _offset)
    {
        for (int i = 0; i < nranges(); i++)
        {
            if (frame >= range_starts[i] && frame < range_stops[i])
            {
                return Mathf.Clamp(frame + _offset, range_starts[i], range_stops[i] - 1);
            }
        }
        throw new Exception($"Index clamp out of range frame {frame} offset {_offset}");
    }
    int frame_increments;
    int ignore_range_end = 20;
    int ignore_surrounding;

    private static MocapDB _instance;
    public static MocapDB Instance { get { return _instance; } }
    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            init_database(getDatabaseFilename());
            _instance = this;
        }
    }
    private string getDatabaseFilename()
    {
        return Application.dataPath + (use60fps ? @"/outputs/database60fps.bin" : @"/outputs/database.bin");
    }
    public MocapDB(string filename = null )
    {
        filename = filename == null ? getDatabaseFilename() : filename;
        init_database(filename);
    }

    public void init_database(string filename, int _num_neigh = 1)
    {
        Debug.Log($"Creating motion database");
        num_neigh = _num_neigh;
        load_db(filename);
        ConfigManager _config = ConfigManager.Instance;
        if (!_config.useCapsuleFeet)
            offset_foot_rots();
    }

    // The motion data foot rotations have the feet at a slight angle when touching the ground, so the heel is slightly off the ground
    // This does not match the training clips that the original Drecon paper used, and also makes it hard for the sim character to balance
    // This should improve balance by offsetting them 20 deg towards the ground 
    // Note: this needs to be called before database_build_matching_features() and will also skew angular velocity  
    public void offset_foot_rots(float num_deg = 20)
    {
        Quaternion offset = Quaternion.AngleAxis(-20f, Vector3.forward);
        for(int i = 0; i < numframes; i++)
        {
            bone_rotations[i][(int)Bone_LeftFoot] *= offset;
            bone_rotations[i][(int)Bone_RightFoot] *= offset;
        }
    }
    public void database_build_matching_features(
        in float feature_weight_foot_position,
        in float feature_weight_foot_velocity,
        in float feature_weight_hip_velocity,
        in float feature_weight_trajectory_positions,
        in float feature_weight_trajectory_directions,
        int _frame_increments,
        int _ignore_surrounding)
    {
        frame_increments = _frame_increments;
        ignore_surrounding = _ignore_surrounding;
        int nfeatures =
            3 + // Left Foot Position
            3 + // Right Foot Position 
            3 + // Left Foot Velocity
            3 + // Right Foot Velocity
            3 + // Hip Velocity
            6 + // Trajectory Positions 2D
            6;  // Trajectory Directions 2D

        //db.features.resize(db.numframes , nfeatures);
        features = new float[numframes][];
        for (int i = 0; i < numframes; i++)
            features[i] = new float[nfeatures];

        features_offset = new float[nfeatures];
        features_scale = new float[nfeatures];

        offset = 0;
        compute_bone_position_feature( (int)Bone_LeftFoot, feature_weight_foot_position);
        compute_bone_position_feature( (int)Bone_RightFoot, feature_weight_foot_position);
        compute_bone_velocity_feature( (int)Bone_LeftFoot, feature_weight_foot_velocity);
        compute_bone_velocity_feature( (int)Bone_RightFoot, feature_weight_foot_velocity);
        compute_bone_velocity_feature( (int)Bone_Hips, feature_weight_hip_velocity);
        compute_trajectory_position_feature( feature_weight_trajectory_positions);
        compute_trajectory_direction_feature(feature_weight_trajectory_directions);

        UnityEngine.Assertions.Assert.IsTrue(offset == nfeatures);

        tree = new KDTree(nfeatures, 1, num_neigh, ignore_surrounding);
        for (int i = 0; i < numframes; i++)
        {
            bool is_stop = false;
            foreach (int stop in range_stops)
                if ( i <= stop && i > stop - ignore_range_end)
                    is_stop = true;
            if (is_stop)
                continue;
            float[] entry = new float[nfeatures + 1];
            for (int j = 0; j < nfeatures; j++)
            {
                entry[j] = features[i][j];
            }
            entry[nfeatures] = i;
            tree.Add(entry);
        }

        tree.Build();
    }
    // Compute the trajectory at 20, 40, and 60 frames in the future
    void compute_trajectory_position_feature(float weight = 1.0f)
    {
        for (int i = 0; i < numframes; i++)
        {
            int t0 = database_trajectory_index_clamp(i, frame_increments);
            int t1 = database_trajectory_index_clamp(i, frame_increments * 2);
            int t2 = database_trajectory_index_clamp(i, frame_increments * 3);

            Vector3 trajectory_pos0 = MathUtils.quat_mul_vec3(inv_sim_rot(i), bone_positions[t0][0] - bone_positions[i][0]);
            Vector3 trajectory_pos1 = MathUtils.quat_mul_vec3(inv_sim_rot(i), bone_positions[t1][0] - bone_positions[i][0]);
            Vector3 trajectory_pos2 = MathUtils.quat_mul_vec3(inv_sim_rot(i), bone_positions[t2][0] - bone_positions[i][0]);

            features[i][offset + 0] = trajectory_pos0.x;
            features[i][offset + 1] = trajectory_pos0.z;
            features[i][offset + 2] = trajectory_pos1.x;
            features[i][offset + 3] = trajectory_pos1.z;
            features[i][offset + 4] = trajectory_pos2.x;
            features[i][offset + 5] = trajectory_pos2.z;
        }

        normalize_feature(6, weight);
        offset += 6;
    }
    // Same for direction
    void compute_trajectory_direction_feature(float weight = 1.0f)
    {
        for (int i = 0; i < numframes; i++)
        {
            int t0 = database_trajectory_index_clamp(i, frame_increments);
            int t1 = database_trajectory_index_clamp(i, frame_increments * 2);
            int t2 = database_trajectory_index_clamp(i, frame_increments * 3);

            Vector3 trajectory_pos0 = MathUtils.quat_mul_vec3(inv_sim_rot(i), MathUtils.quat_mul_vec3(bone_rotations[t0][0], Vector3.forward));
            Vector3 trajectory_pos1 = MathUtils.quat_mul_vec3(inv_sim_rot(i), MathUtils.quat_mul_vec3(bone_rotations[t1][0], Vector3.forward));
            Vector3 trajectory_pos2 = MathUtils.quat_mul_vec3(inv_sim_rot(i), MathUtils.quat_mul_vec3(bone_rotations[t2][0], Vector3.forward));

            features[i][offset + 0] = trajectory_pos0.x;
            features[i][offset + 1] = trajectory_pos0.z;
            features[i][offset + 2] = trajectory_pos1.x;
            features[i][offset + 3] = trajectory_pos1.z;
            features[i][offset + 4] = trajectory_pos2.x;
            features[i][offset + 5] = trajectory_pos2.z;
        }

        normalize_feature(6, weight);
        offset += 6;
    }

    private void compute_bone_velocity_feature(int bone, float weight) {
        for (int i = 0; i < numframes; i++)
        {
            Vector3 bone_position = Vector3.zero;
            Vector3 bone_velocity = Vector3.zero;
            Quaternion bone_rotation = Quaternion.identity;
            Vector3 bone_angular_velocity = Vector3.zero;

            forward_kinematics_velocity(
                ref bone_position,
                ref bone_velocity,
                ref bone_rotation,
                ref bone_angular_velocity,
                bone_positions[i],
                bone_velocities[i],
                bone_rotations[i],
                bone_angular_velocities[i],
                bone);

            bone_velocity = MathUtils.quat_mul_vec3(inv_sim_rot(i), bone_velocity);

            features[i][offset + 0] = bone_velocity.x;
            features[i][offset + 1] = bone_velocity.y;
            features[i][offset + 2] = bone_velocity.z;
        }

        normalize_feature(3, weight);

        offset += 3;
    }


    // Compute a feature for the position of a bone relative to the simulation/root bone
    private void compute_bone_position_feature(int bone, float weight)
    {
        for (int i = 0; i < numframes ; i++)
        {
            Vector3 bone_position = Vector3.zero;
            Quaternion bone_rotation = Quaternion.identity;

            forward_kinematics(
                ref bone_position,
                ref bone_rotation,
                bone_positions[i],
                bone_rotations[i],
                bone);

            // multiply by inverse of simulation bone 
            bone_position = MathUtils.quat_mul_vec3(inv_sim_rot(i), bone_position - bone_positions[i][0]);

            features[i][offset + 0] = bone_position.x;
            features[i][offset + 1] = bone_position.y;
            features[i][offset + 2] = bone_position.z;
        }
        normalize_feature(3, weight);

        offset += 3;
    }

    public void forward_kinematics_velocity(
        ref Vector3 bone_position,
        ref Vector3 bone_velocity,
        ref Quaternion bone_rotation,
        ref Vector3 bone_angular_velocity,
        Vector3[]  bone_positions,
        Vector3[]  bone_velocities,
        Quaternion[]  bone_rotations,
        Vector3[] bone_angular_velocities,
        in int bone)
    {
        
        if (bone_parents[bone] != -1)
        {
            Vector3 parent_position = Vector3.zero;
            Vector3 parent_velocity = Vector3.zero;
            Quaternion parent_rotation = Quaternion.identity;
            Vector3 parent_angular_velocity = Vector3.zero;

            forward_kinematics_velocity(
            ref parent_position,
            ref parent_velocity,
            ref parent_rotation,
            ref parent_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_parents[bone]);

            bone_position = MathUtils.quat_mul_vec3(parent_rotation, bone_positions[bone]) + parent_position;
            bone_velocity = 
                parent_velocity +
                MathUtils.quat_mul_vec3(parent_rotation, bone_velocities[bone]) + 
                Vector3.Cross(parent_angular_velocity, MathUtils.quat_mul_vec3(parent_rotation, bone_positions[bone]));
            bone_rotation = parent_rotation * bone_rotations[bone];
            bone_angular_velocity = MathUtils.quat_mul_vec3(parent_rotation, bone_angular_velocities[bone] + parent_angular_velocity);
        }
        else
        {
            bone_position = bone_positions[bone];
            bone_velocity = bone_velocities[bone];
            bone_rotation = bone_rotations[bone];
            bone_angular_velocity = bone_angular_velocities[bone];
        }
    }

    private void forward_kinematics(ref Vector3 bone_position, ref Quaternion bone_rotation, Vector3[] bone_positions, Quaternion[] bone_rotations, int bone)
    {
        if (bone_parents[bone] != -1)
        {
            Vector3 parent_position = Vector3.zero;
            Quaternion parent_rotation = Quaternion.identity;

            forward_kinematics(
                ref parent_position,
                ref parent_rotation,
                bone_positions,
                bone_rotations,
                bone_parents[bone]);

            bone_position = MathUtils.quat_mul_vec3(parent_rotation, bone_positions[bone]) + parent_position;
            bone_rotation = parent_rotation * bone_rotations[bone];
        }
        else
        {
            bone_position = bone_positions[bone];
            bone_rotation = bone_rotations[bone];
        }
    }

    internal void forward_kinematics_partial(Vector3[] global_bone_positions, Quaternion[] global_bone_rotations, bool[] global_bone_computed, Vector3[] local_bone_positions,
        Quaternion[] local_bone_rotations, int bone)
    {
        if (bone_parents[bone] == -1)
        {
            global_bone_positions[bone] = local_bone_positions[bone];
            global_bone_rotations[bone] = local_bone_rotations[bone];
            global_bone_computed[bone] = true;
            return;
        }

        if (!global_bone_computed[bone_parents[bone]])
        {
            forward_kinematics_partial(
                global_bone_positions,
                global_bone_rotations,
                global_bone_computed,
                local_bone_positions,
                local_bone_rotations,
                bone_parents[bone]);
        }

        Vector3 parent_position = global_bone_positions[bone_parents[bone]];
        Quaternion parent_rotation = global_bone_rotations[bone_parents[bone]];
        global_bone_positions[bone] = MathUtils.quat_mul_vec3(parent_rotation, local_bone_positions[bone]) + parent_position;
        global_bone_rotations[bone] = parent_rotation * local_bone_rotations[bone];
        global_bone_computed[bone] = true;
    }
    void normalize_feature(
            in int size,
            in float weight = 1.0f)
    {
        // First compute what is essentially the mean 
        // value for each feature dimension
        for (int j = 0; j < size; j++)
            features_offset[offset + j] = 0.0f;    

        for (int i = 0; i < numframes; i++)
            for (int j = 0; j < size; j++)
                features_offset[offset + j] += features[i][ offset + j] / numframes;

        // Now compute the variance of each feature dimension
        float[] vars = new float[size];
        for (int i = 0; i < numframes; i++)
            for (int j = 0; j < size; j++)
                vars[j] += Mathf.Pow(features[i][offset + j] - features_offset[offset + j], 2) / numframes;


        // We compute the overall std of the feature as the average
        // std across all dimensions
        float std = 0.0f;
        for (int j = 0; j < size; j++)
            std += Mathf.Sqrt(vars[j]) / size;

        // Features with no variation can have zero std which is
        // almost always a bug.
        UnityEngine.Assertions.Assert.IsTrue(std > 0.0);

        // The scale of a feature is just the std divided by the weight
        for (int j = 0; j < size; j++)
            features_scale[offset + j] = std / weight;

        // Using the offset and scale we can then normalize the features
        for (int i = 0; i < numframes; i++)
            for (int j = 0; j < size; j++)
                features[i][ offset + j ] = (features[i][offset + j] - features_offset[offset + j]) / features_scale[offset + j];
    }
    public void setDataToFrame(ref Vector3[] local_bone_positions, ref Quaternion[] local_bone_rotations, int frameIdx)
    {
        local_bone_positions = bone_positions[frameIdx];
        local_bone_rotations = bone_rotations[frameIdx];
    }
    public int nframes() { return bone_positions.Length;  }
    public int nbones() { return bone_positions[0].Length; }
    public int nranges() { return range_starts.Length; }
    public int nfeatures() { return features[0].Length; }

    private void load2Darray(BinaryReader reader, ref Vector3[][] arr, bool invertY = false)
    {
        // rows = num_frames, cols = num_bones
        uint rows, cols;
        rows = reader.ReadUInt32();
        cols = reader.ReadUInt32();
        arr = new Vector3[rows][];
        for (int i = 0; i < rows; i++)
        {
            arr[i] = new Vector3[cols];
            for (int j = 0; j < cols; j++)
            {
                float x, y, z;
                x = reader.ReadSingle();
                y = reader.ReadSingle();
                y *= invertY ? -1 : 1;
                z = reader.ReadSingle();
                arr[i][j] = new Vector3(x, y, z);
            }
        }
    }


    private void load2Darray(BinaryReader reader, ref Quaternion[][] arr)
    {
        // rows = num_frames, cols = num_bones
        uint rows, cols;
        rows = reader.ReadUInt32();
        cols = reader.ReadUInt32();
        arr = new Quaternion[rows][];
        for (int i = 0; i < rows; i++)
        {
            arr[i] = new Quaternion[cols];
            for (int j = 0; j < cols; j++)
            {
                float w, x, y, z;
                w = reader.ReadSingle();
                x = reader.ReadSingle();
                y = reader.ReadSingle();
                z = reader.ReadSingle();
                arr[i][j] = new Quaternion(x, y, z , w);
            }

        }
    }

    private void load2Darray(BinaryReader reader, ref bool[][] arr)
    {
        // rows = num_frames, cols = num_bones
        uint rows, cols;
        rows = reader.ReadUInt32();
        cols = reader.ReadUInt32();
        //Debug.Log($"Rows: {rows} , Cols: {cols}");
        arr = new bool[rows][];
        for (int i = 0; i < rows; i++)
        {
            arr[i] = new bool[cols];
            for (int j = 0; j < cols; j++)
            {
                arr[i][j] = reader.ReadBoolean();
            }

        }
    }

    private void loadArray(BinaryReader reader, ref int[] arr)
    {
        uint size;
        size = reader.ReadUInt32();
        arr = new int[size];
        for (int i = 0; i < size; i++)
        {
            uint val = reader.ReadUInt32();
            arr[i] = (int) val;
        }
    }

    private void load_db(string filename)
    {
        Stream stream;
#if UNITY_EDITOR
        stream = File.Open(filename, FileMode.Open);
#else
        var textAsset = Resources.Load<TextAsset>("database60fps");
        stream = new MemoryStream(textAsset.bytes);
#endif
        using (stream)
        {
            using (var reader = new BinaryReader(stream, Encoding.UTF8))
            {
                load2Darray(reader, ref bone_positions, false);
                load2Darray(reader, ref bone_velocities);
                load2Darray(reader, ref bone_rotations);
                load2Darray(reader, ref bone_angular_velocities);
                loadArray(reader, ref bone_parents);
                // in the original generate database code, bone_parents is written in binary
                // as an unsigned int, need to convert it
                loadArray(reader, ref  range_starts);
                loadArray(reader, ref range_stops);
                load2Darray(reader, ref contact_states);

            }
        }
        bone_parents[0] = -1;
        numframes = nframes();
    }



}
