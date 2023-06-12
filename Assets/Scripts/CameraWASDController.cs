using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class CameraWASDController : MonoBehaviour
{
    //public float velocityPerSecond = 2f;
    public float horizontalVelPerSecond = 2f;
    public float verticalVelPerSec = 2f;
    Keyboard kb;
    KeyControl forwardKey, backwardKey, rightKey, leftKey;
    public float xSens = 400f;
    public float ySens = 100f;
    public float screenSensitivity = 300f;
    float xRot, yRot;

    Vector2 screenVel, screenAcc;

    public float screenRotHalflife= .1f;
    void Start()
    {
        //Cursor.lockState = CursorLockMode.Locked;
        kb = Keyboard.current;
        forwardKey = kb.wKey;
        backwardKey = kb.sKey;
        leftKey = kb.aKey;
        rightKey = kb.dKey;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 positionDelta = transform.forward * (forwardKey.isPressed ? 1f: 0f);
        positionDelta += -transform.forward * (backwardKey.isPressed ? 1f : 0f);
        positionDelta += transform.right * (rightKey.isPressed ? 1f : 0f);
        positionDelta += -transform.right * (leftKey.isPressed ? 1f : 0f);
        positionDelta = positionDelta.normalized * (horizontalVelPerSecond * Time.deltaTime);

        float yDelta = kb.spaceKey.isPressed ? 1f : 0f;
        yDelta += kb.ctrlKey.isPressed ? -1f : 0f;
        positionDelta.y = yDelta * verticalVelPerSec * Time.deltaTime;

        transform.position += positionDelta;
        if (kb.capsLockKey.isPressed)
            transform.position = new Vector3(0f, 1f, 1f);

        if (Cursor.lockState == CursorLockMode.None)
            return;
        SpringUtils.spring_character_update(screenVel, screenAcc, Mouse.current.delta.ReadValue() * screenSensitivity * Time.deltaTime, screenRotHalflife, Time.deltaTime , out screenVel, out screenAcc);
        Vector2 mouseDelta = screenVel;
        yRot += mouseDelta.x;
        xRot -= mouseDelta.y;
        xRot = Mathf.Clamp(xRot, -90f, 90f);
        transform.rotation = Quaternion.Euler(xRot, yRot, 0f);
    }
}
