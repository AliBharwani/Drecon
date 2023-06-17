using System.Collections.Generic;
using UnityEngine;
using System.IO;
public enum SixDRotationMethod
{
    TRSTimesMatrix,
    RotateObjectWithOrthonormalVector,
    MatrixToQuat,
}

// The type of rotation the action outputs are formatted as
public enum ActionRotationType
{
    Euler,
    AxisAngle,
    SixD,
    Exp,
}
public class ConfigManager : MonoBehaviour
{
    [Header("CONFIG SETTINGS")]
    public string writeToFilePath = "";
    public string filename = "";
    public string loadFromFilePath = "";

    [Header("INFERENCE SETTINGS")]
    public bool clampKinCharToSim;
    public bool userControl;
    public float clampingMaxDistance = .5f;
    public bool useSkinnedMesh = false;
    public bool doNotRenderKinChar = false;

    [Header("PHYSICAL CHARACTER SETTINGS")]
    public int solverIterations = 32;
    public bool addOrientationDataToObsState = false;
    public bool setDriveTargetVelocities;
    public bool noStrafing = false;
    public bool setRotsDirectly = false;
    public bool outputIsBase = false;
    public ActionRotationType actionRotType;
    public float alphaForExpMap = 120f;
    public SixDRotationMethod sixDRotMethod;
    public bool networkControlsAllJoints = false;
    public bool fullRangeEulerOutputs;
    public bool resolveSimReferenceFrameWithSimRotation = false;
    public bool applyActionOverMultipleTimeSteps = false;
    [Header("TRAINING HYPERPARAM SETTINGS")]
    public int EVALUATE_EVERY_K_STEPS = 2;
    public int N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT = 4;
    public float EPISODE_END_REWARD = -.5f;
    public int MAX_EPISODE_LENGTH_SECONDS = 20;
    public float ACTION_STIFFNESS_HYPERPARAM = .2f;
    [Header("KINEMATIC CHARACTER SETTINGS")]
    public float prob_to_change_inputs = .001f;
    public bool useCustomInputGenerator = false;
    public float inputGeneratorHalflife = .5f;
    public float simulationVelocityHalflife = .27f;
    public float simulation_rotation_halflife = .27f;
    public bool walkOnly = false;
    public float searchTime = .2f;
    public float MAX_WANDERING_RADIUS = 50f;
    // PROJECTILE HYPER PARAMS
    [Header("PROJECTILE SETTINGS")]
    public bool projectileTraining = true;
    public int maxNumProjectiles = 10;
    public float LAUNCH_FREQUENCY = 1f;
    public  float LAUNCH_RADIUS = .66f;
    public  float LAUNCH_SPEED = 5f;
    public float PROJECTILE_MAX_WEIGHT = 10f;
    public float PROJECTILE_MIN_WEIGHT = 1f;
    public float PROJECTILE_SCALE = 1f;
    [Header("ARTICULATION BODY SETTINGS")]
    // Art body params
    public string[] boneToNames = new string[23];
    public  float[] boneToStiffness = new float[23];
    public List<MusclePower> MusclePowers;
    public float forceLimit;
    public float damping;
    public bool dampingScalesWithStiffness;
    public float pGainMultiplier = 1f;
    public bool rewardsInGUI;
    [Header("UNTOUCHED / LEGACY")]
    public bool useCapsuleFeet;
    public bool resetKinCharOnEpisodeEnd = false;
    public bool selfCollision;

    [System.Serializable]
    public class MusclePower
    {
        public MotionMatchingAnimator.Bones Bone;
        public Vector3 PowerVector;
    }

    private static ConfigManager _instance;

    public static ConfigManager Instance { get { return _instance; } }
    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            Application.targetFrameRate = 60;
            _instance = this;
        }
    }

    [ContextMenu("Multiply all stiffness values by pGainMultiplier")]
    public void multiplyAllStiffnessValues()
    {
        for (int i = 0; i < 23; i++)
            boneToStiffness[i] *= pGainMultiplier;
    }

    [ContextMenu("Multiply all muscle power values by pGainMultiplier")]
    public void multiplyAllMusclePowerValues()
    {
        foreach (var mp in MusclePowers)
            mp.PowerVector *= pGainMultiplier;
    }

    [ContextMenu("Multiply all Lower Body stiffness values by pGainMultiplier")]
    public void multiplyAllLowerBodyStiffnessValues()
    {
        for (int i = 0; i < 10; i++)
            boneToStiffness[i] *= pGainMultiplier;
    }


    [ContextMenu("Init bone to string")]
    public void initBoneToString()
    {
        for (int i = 0; i < 23; i++)
            boneToNames[i]  = ((MotionMatchingAnimator.Bones)i).ToString();
    }

    [ContextMenu("Write out config file to current config name ")]
    public void writeCurrentConfig()
    {
        string folderpath = Application.dataPath + @"/" + writeToFilePath;
        Debug.Log($"folderpath: {folderpath}");
        string filepath = folderpath + @"/" + filename;
        // Check if the folder exists
        if (!Directory.Exists(folderpath))
        {
            // Create the folder
            Directory.CreateDirectory(folderpath);
            Debug.Log("Folder created.");
        }
        string json = JsonUtility.ToJson(this);
        File.WriteAllText(filepath, json);
    }

    [ContextMenu("Load config file")]
    public void loadConfig()
    {
        string filepath = Application.dataPath + @"/" + loadFromFilePath + @"/" + filename;
        string json = File.ReadAllText(filepath);
        JsonUtility.FromJsonOverwrite(json, this);
    }

    [ContextMenu("Reset current config")]
    public void resetConfig()
    {
        string filepath = Application.dataPath + @"/" + writeToFilePath + @"/" + filename;
        string json = File.ReadAllText(filepath);
        JsonUtility.FromJsonOverwrite(json, this);
    }
}
