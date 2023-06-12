using UnityEngine;

public class InputGenerator : MonoBehaviour
{
    public Vector2 currentPosition;
    Vector2 targetPosition;
    Vector2 velocity;
    public float inputChangeHalflife = .5f;
    void Start()
    {
        targetPosition = Random.insideUnitCircle;    
    }

    void FixedUpdate()
    {
        // tick spring update
        SpringUtils.spring_character_update(currentPosition, velocity, targetPosition, inputChangeHalflife, Time.fixedDeltaTime, out currentPosition, out velocity);
    }

    public void changeDirection()
    {
        targetPosition = Random.value < .01f ?  Vector2.zero : Random.insideUnitCircle;
    }
}
