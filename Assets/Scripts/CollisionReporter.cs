using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionReporter : MonoBehaviour
{
    public MLAgent director;
    private void OnCollisionEnter(Collision collision)
    {
        director.processCollision(collision);
    }
}
