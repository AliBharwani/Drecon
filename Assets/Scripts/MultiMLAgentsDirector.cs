using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultiMLAgentsDirector : MonoBehaviour
{
    public int numAgents = 5;
    public GameObject modelDirector;
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
            directors[i] = createMLAgent();
        Application.targetFrameRate = targetFrameRate;
        Physics.autoSimulation = false;
        //UnityEngine.Rendering.DebugManager.instance.enableRuntimeUI = false;
    }

    private void Start()
    {
        for(int i = 0; i < numAgents; i++) 
            directors[i].AssignLayer(LayerMask.NameToLayer($"model_{i + 1}"));
        Physics.autoSimulation = true;
    }
    private MLAgentsDirector createMLAgent()
    {
        GameObject obj =  Instantiate(modelDirector);
        obj.SetActive(true);
        MLAgentsDirector director = obj.GetComponent<MLAgentsDirector>();
        director.EVALUATE_EVERY_K_STEPS = _config.EVALUATE_EVERY_K_STEPS;
        director.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT = _config.N_FRAMES_TO_NOT_COUNT_REWARD_AFTER_TELEPORT;
        director.EPISODE_END_REWARD = _config.EPISODE_END_REWARD;
        director.MAX_EPISODE_LENGTH_SECONDS = _config.MAX_EPISODE_LENGTH_SECONDS;
        director.ACTION_STIFFNESS_HYPERPARAM = _config.ACTION_STIFFNESS_HYPERPARAM;

        director.LAUNCH_FREQUENCY = _config.LAUNCH_FREQUENCY;
        director.LAUNCH_RADIUS = _config.LAUNCH_RADIUS;
        director.LAUNCH_SPEED = _config.LAUNCH_SPEED;
        director.projectileTraining = _config.projectileTraining;
        director.solverIterations = _config.solverIterations;
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
