using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// GameObject.FixedUpdate example.
//
// Measure frame rate comparing FixedUpdate against Update.
// Show the rates every second.

public class FixedUpdateCounter : MonoBehaviour
{
    private float updateCount = 0;
    private float fixedUpdateCount = 0;
    private float timeSinceLastCount;

    private float timeOfLastCount;

    private float updateUpdateCountPerSecond;
    private float updateFixedUpdateCountPerSecond;

    void Awake()
    {
        // Uncommenting this will cause framerate to drop to 10 frames per second.
        // This will mean that FixedUpdate is called more often than Update.
        //Application.targetFrameRate = 10;
        StartCoroutine(Loop());
    }

    // Increase the number of calls to Update.
    void Update()
    {
        updateCount += 1;
    }

    // Increase the number of calls to FixedUpdate.
    void FixedUpdate()
    {
        fixedUpdateCount += 1;
    }

    // Show the number of calls to both messages.
    void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 24;
        GUI.Label(new Rect(100, 100, 200, 50), "Update: " + updateUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 150, 200, 50), "FixedUpdate: " + updateFixedUpdateCountPerSecond.ToString(), fontSize);
        GUI.Label(new Rect(100, 200, 600, 100), "timeSinceLastCount: " + timeSinceLastCount.ToString(), fontSize);

    }

    // Update both CountsPerSecond values every second.
    IEnumerator Loop()
    {
        while (true)
        {
            yield return new WaitForSeconds(1 * Time.timeScale);
            updateUpdateCountPerSecond = updateCount;
            updateFixedUpdateCountPerSecond = fixedUpdateCount;
            timeSinceLastCount = Time.realtimeSinceStartup - timeOfLastCount;
            updateCount = 0;
            fixedUpdateCount = 0;
            timeOfLastCount = Time.realtimeSinceStartup;
        }
    }
}