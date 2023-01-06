 using UnityEditor;
 using UnityEngine;
 
 public class ScenePhysicsEditorWindow : EditorWindow
{
    const float physicsFrameTime = 1f / 60f;
    float physicsRunTime = physicsFrameTime;

    private void OnGUI()
    {
        GUILayout.Label("Base Settings", EditorStyles.boldLabel);
        physicsRunTime = EditorGUILayout.Slider("Physics Run Time", physicsRunTime, physicsFrameTime, 10);

        if (GUILayout.Button("Run Physics"))
        {
            StepPhysics();
        }
    }

    private void StepPhysics()
    {
        Physics.autoSimulation = false;
        float timeSimulated = 0f;
        while (timeSimulated < physicsRunTime) { 
            Physics.Simulate(physicsFrameTime);
            timeSimulated += physicsFrameTime;
        }
        Physics.autoSimulation = true;
    }

    [MenuItem("Tools/Scene Physics")]
    private static void OpenWindow()
    {
        GetWindow<ScenePhysicsEditorWindow>(false, "Physics", true);
    }
}