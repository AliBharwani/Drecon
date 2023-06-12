using UnityEngine;

public class CollisionReporter : MonoBehaviour
{
    public MLAgent agent;
    private void OnCollisionEnter(Collision collision)
    {
        agent.processCollision(collision);
    }
}
