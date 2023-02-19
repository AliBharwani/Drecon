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
            if (_config.selfCollision)
                directors[i].AssignLayer(LayerMask.NameToLayer($"model_{i + 1}"), LayerMask.NameToLayer($"model_{i + 1}"));
            else
                directors[i].AssignLayer(LayerMask.NameToLayer($"test_model"), LayerMask.NameToLayer($"Default"));

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
        obj.SetActive(true);
        return director;
    }
    void FixedUpdate()
    {
        //Debug.Log($"{Time.frameCount} : MultiMLAgentsDirector updating");

        foreach (var director in directors)
            director.calcAndSetRewards();
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
