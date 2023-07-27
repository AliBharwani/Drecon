using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SixtyFPSSyncOracle : MonoBehaviour
{
    private static SixtyFPSSyncOracle _instance;
    float timeSinceLastUpdate = 0f;
    public bool isSyncFrame = false;
    float period = 1f / 60f;
    public static SixtyFPSSyncOracle Instance { get { return _instance; } }
    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(this.gameObject);
        }
        else
        {
            _instance = this;
        }
    }

    void FixedUpdate()
    {
        isSyncFrame = false;
        timeSinceLastUpdate += Time.fixedDeltaTime;
        if (timeSinceLastUpdate < period)
            return;
        timeSinceLastUpdate -= period;
        isSyncFrame = true;
    }
}
