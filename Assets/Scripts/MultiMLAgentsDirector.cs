using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultiMLAgentsDirector : MonoBehaviour
{
    public int numAgents = 5;
    public GameObject modelDirector;
    private MLAgentsDirector[] directors;
    // Start is called before the first frame update
    void Awake()
    {
        if (modelDirector == null)
            return;
        directors = new MLAgentsDirector[numAgents];
        for (int i = 0; i < numAgents; i++)
            directors[i] = createMLAgent();
        Physics.autoSimulation = false;
    }

    private void Start()
    {
        for(int i = 0; i < numAgents; i++)
            for(int j = + 1; j < numAgents; j++)
            {
                var colliders1 = directors[i].simulated_char.GetComponentsInChildren<Collider>();
                var colliders2 = directors[j].simulated_char.GetComponentsInChildren<Collider>();
                foreach (var c1 in colliders1)
                    foreach (var c2 in colliders2)
                        Physics.IgnoreCollision(c1, c2);

            }
        Physics.autoSimulation = true;
    }
    private MLAgentsDirector createMLAgent()
    {
        GameObject obj =  Instantiate(modelDirector);
        obj.SetActive(true);
        return obj.GetComponent<MLAgentsDirector>();
    }
    //void Update()
    //{
    //    if (modelDirector == null)
    //        return;
    //}
}
