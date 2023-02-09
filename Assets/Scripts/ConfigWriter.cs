using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.IO;

public class ConfigWriter : MonoBehaviour
{
    public string writeToFilePath = "";
    public string filename = "";
    public string loadFromFilePath = "";

    // HYPER PARAMS
    public int solverIterations = 32;
    public bool actionsAreEulerRotations = true;
    public bool actionsAre6DRotations = false;
    public bool normalizeObservations = true;
    public bool normalizeLimitedDOFOutputs = true;
    public bool normalizeRewardComponents = false;
    public bool networkControlsAllJoints = false;

    public bool useGeodesicForAngleDiff = false;
    public float poseRewardMultiplier = 1f / 10f;
    public bool resetKinCharOnEpisodeEnd = false;
    public int EVALUATE_EVERY_K_STEPS = 2;
    public int N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT = 4;
    public float EPISODE_END_REWARD = -.5f;
    public int MAX_EPISODE_LENGTH_SECONDS = 20;
    public float ACTION_STIFFNESS_HYPERPARAM = .2f;
    public float simulationVelocityHalflife = .27f;
    public bool walkOnly = false;
    public bool useHandmadeColliders = false;
    // PROJECTILE HYPER PARAMS
    public bool projectileTraining = true;
    public float LAUNCH_FREQUENCY = 1f;
    public  float LAUNCH_RADIUS = .66f;
    public  float LAUNCH_SPEED = 5f;
    // Art body params
    public string[] boneToNames = new string[23];
    public  float[] boneToStiffness = new float[23];
    [System.Serializable]
    public class MusclePower
    {
        public mm_v2.Bones Bone;
        public Vector3 PowerVector;
    }

    public List<MusclePower> MusclePowers;
    public float forceLimit;
    public  float damping;

    private static ConfigWriter _instance;
    public static ConfigWriter Instance { get { return _instance; } }
    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            _instance = this;
        }
    }

    public float pGainMultiplier = 1f;
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
            boneToNames[i]  = ((mm_v2.Bones)i).ToString();
    }
    [ContextMenu("Write out config file to current config name ")]
    public void writeCurrentConfig()
    {
        string filepath = Application.dataPath + @"/" +  writeToFilePath + @"/" +  filename;
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
}
