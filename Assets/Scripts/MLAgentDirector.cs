using UnityEngine;

public class MLAgentDirector : MonoBehaviour
{
    public int numAgents = 5;
    public GameObject modelAgent;
    public GameObject modelAgentWithOrientationData;
    public GameObject model6DAgent;
    public GameObject modelAllJoints6DAgent;
    public GameObject modelAllJoints3DAgent;
    public GameObject modelAgentAllJointsWOrientData;
    private MLAgent[] agents;
    public int reportMeanRewardEveryNSteps = 10000;
    private int curStep = 0;
    public int targetFrameRate = -1;
    private float meanReward;
    public int fps = 60;
    private ConfigManager _config;


    // Start is called before the first frame update
    void Awake()
    {
        if (modelAgent == null)
            return;
        _config = ConfigManager.Instance;

        Time.fixedDeltaTime = (1f / (float)fps);
        Physics.defaultSolverIterations = _config.solverIterations;
        Physics.defaultSolverVelocityIterations = _config.solverIterations;
        agents = new MLAgent[numAgents];
        for (int i = 0; i < numAgents; i++)
        {
            agents[i] = createMLAgent();
            if (_config.selfCollision)
                agents[i].AssignLayer(LayerMask.NameToLayer($"model_{i + 1}"));
            else
                agents[i].AssignLayer(LayerMask.NameToLayer($"test_model"));

        }
        Application.targetFrameRate = targetFrameRate;
    }

    private MLAgent createMLAgent()
    {
        GameObject obj;
        bool actionsAre6D = _config.actionRotType == ActionRotationType.SixD;
        if (_config.networkControlsAllJoints)
            obj = actionsAre6D ? Instantiate(modelAllJoints6DAgent) : _config.addOrientationDataToObsState ? Instantiate(modelAgentAllJointsWOrientData) : Instantiate(modelAllJoints3DAgent);
        else
            obj = actionsAre6D ? Instantiate(model6DAgent) : _config.addOrientationDataToObsState ? Instantiate(modelAgentWithOrientationData) : Instantiate(modelAgent);
        MLAgent agent = obj.GetComponent<MLAgent>();
        obj.SetActive(true);
        return agent;
    }
    void FixedUpdate()
    {
        foreach (var agent in agents)
            agent.LateFixedUpdate();
        curStep++;
        if (curStep % reportMeanRewardEveryNSteps == 0)
        {
            Debug.Log($"Step {curStep} mean reward last {reportMeanRewardEveryNSteps} is: {meanReward}");
            meanReward = 0f;
        }
        float curStepReward = 0f;
        foreach (var agent in agents)
            curStepReward += agent.finalReward / (float) agents.Length;
        meanReward += (curStepReward / (float)reportMeanRewardEveryNSteps);
    }
}
