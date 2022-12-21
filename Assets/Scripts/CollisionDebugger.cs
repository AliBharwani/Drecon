using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDebugger : MonoBehaviour
{
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log($"{gameObject.name} collided with {collision.gameObject.name}");
    }
}
