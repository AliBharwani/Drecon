using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MimicUserInput : MonoBehaviour
{

	// Start is called before the first frame update
	void Start()
    {
		vel = new Vector3();
		cur = new Vector3();
	}

    // Update is called once per frame
    void Update()
    {
        
    }

	float frametime = 1f / 30f;
	float max_move_time = .5f;
	float max_halflife = .2f;
	float halflife;
	Vector3 cur;
	Vector3 vel;
	bool reset = true;
	Vector3 target;
	float time_to_target;
	float time_elapsed;

	public Vector3 nextInput(float deltatime)
	{
		if (reset)
		{
			cur = target;
			target = Random.insideUnitCircle;
			time_elapsed = deltatime;
			time_to_target = Mathf.Max(time_elapsed, Random.Range(0f, max_move_time));
			// halflife = rand.NextSingle() * max_halflife;
			reset = false;
		}
		else
		{
			time_elapsed += deltatime;
			if (time_elapsed > time_to_target)
			{
				reset = true;
				return target;
			}
		}
		// float new_x = simple_spring_damper_implicit(cur.x, vel.x, target.x, halflife, deltatime)
		float new_x = Mathf.Lerp(cur.x, target.x, time_elapsed / time_to_target);
		float new_y = Mathf.Lerp(cur.y, target.y, time_elapsed / time_to_target);
		return new Vector3(new_x, new_y);
	}
}
