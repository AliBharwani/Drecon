using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Policies;
using UnityEngine;

public class MultiMLAgentsDirector : MonoBehaviour
{
    public int numAgents = 5;
    public GameObject modelDirector;
    public GameObject model6DDirector;
    public GameObject modelAllJointsDirector;
    public GameObject modelAllJoints3DDirector;

    private MLAgentsDirector[] directors;
    public int reportMeanRewardEveryNSteps = 10000;
    private int curStep = 0;
    public int targetFrameRate = -1;
    private float meanReward;
    public int fps = 60;
    private ConfigWriter _config;


    // Start is called before the first frame update
    void Awake()
    {
        if (modelDirector == null)
            return;
        _config = ConfigWriter.Instance;

        Time.fixedDeltaTime = (1f / (float)fps);
        Physics.defaultSolverIterations = _config.solverIterations;
        Physics.defaultSolverVelocityIterations = _config.solverIterations;
        directors = new MLAgentsDirector[numAgents];
        for (int i = 0; i < numAgents; i++)
        {
            directors[i] = createMLAgent();
            directors[i].AssignLayer(LayerMask.NameToLayer($"model_{i + 1}"));

        }
        Application.targetFrameRate = targetFrameRate;
        //UnityEngine.Rendering.DebugManager.instance.enableRuntimeUI = false;
    }

    private void Start()
    {
    }
    private MLAgentsDirector createMLAgent()
    {
        GameObject obj;
        //obj = _config.networkControlsAllJoints ? Instantiate(modelAllJointsDirector) : _config.actionsAre6DRotations ? Instantiate(model6DDirector) : Instantiate(modelDirector);
        if (_config.networkControlsAllJoints)
            obj = _config.actionsAre6DRotations ? Instantiate(modelAllJointsDirector) : Instantiate(modelAllJoints3DDirector);
        else
            obj = _config.actionsAre6DRotations ? Instantiate(model6DDirector) : Instantiate(modelDirector);
        MLAgentsDirector director = obj.GetComponent<MLAgentsDirector>();
        director.EVALUATE_EVERY_K_STEPS = _config.EVALUATE_EVERY_K_STEPS; 
        director.normalizeObservations = _config.normalizeObservations;
        director.resetKinCharOnEpisodeEnd = _config.resetKinCharOnEpisodeEnd; 
        director.actionsAreEulerRotations = _config.actionsAreEulerRotations;
        director.normalizeLimitedDOFOutputs = _config.normalizeLimitedDOFOutputs;
        director.useGeodesicForAngleDiff = _config.useGeodesicForAngleDiff;
        director.poseRewardMultiplier = _config.poseRewardMultiplier; 
        director.normalizeRewardComponents = _config.normalizeRewardComponents;
        director.networkControlsAllJoints = _config.networkControlsAllJoints;
        //if (_config.actionsAre6DRotations)
        //{
        //int numActions = MLAgentsDirector.fullDOFBones.Length * 6; // Should be 7 3-DOF bones, so 42 Actions
        //numActions += MLAgentsDirector.limitedDOFBones.Length;
        //obj.GetComponent<BehaviorParameters>().BrainParameters.ActionSpec = new Unity.MLAgents.Actuators.ActionSpec(numActions);
        //obj.GetComponent<BehaviorParameters>().BrainParameters.VectorObservationSize = 110 + 21;
        //director.actionsAre6DRotations = true;
        //}
        director.actionsAre6DRotations = _config.actionsAre6DRotations; 

        director.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT = _config.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT;
        director.EPISODE_END_REWARD = _config.EPISODE_END_REWARD;
        director.MAX_EPISODE_LENGTH_SECONDS = _config.MAX_EPISODE_LENGTH_SECONDS;
        director.ACTION_STIFFNESS_HYPERPARAM = _config.ACTION_STIFFNESS_HYPERPARAM;
        director.simulationVelocityHalflife = _config.simulationVelocityHalflife;
        director.walkOnly = _config.walkOnly;
        director.useHandmadeColliders = _config.useHandmadeColliders;

        director.LAUNCH_FREQUENCY = _config.LAUNCH_FREQUENCY;
        director.LAUNCH_RADIUS = _config.LAUNCH_RADIUS;
        director.LAUNCH_SPEED = _config.LAUNCH_SPEED;
        director.projectileTraining = _config.projectileTraining;
        director.solverIterations = _config.solverIterations;
        obj.SetActive(true);
        return director;
    }
    void FixedUpdate()
    {
        curStep++;
        if (curStep % reportMeanRewardEveryNSteps == 0)
        {
            Debug.Log($"Step {curStep} mean reward last {reportMeanRewardEveryNSteps} is: {meanReward}");
            meanReward = 0f;
        }
        float curStepReward = 0f;
        foreach (var director in directors)
            curStepReward += director.finalReward / (float) directors.Length;
        meanReward += (curStepReward / (float)reportMeanRewardEveryNSteps);
    }
}
