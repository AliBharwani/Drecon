using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class CameraWASDController : MonoBehaviour
{
    // Start is called before the first frame update
    bool isStrafing = true;
    //public float velocityPerSecond = 2f;
    public float horizontalVelPerSecond = 2f;
    public float verticalVelPerSec = 2f;
    Keyboard kb;
    KeyControl upKey, downKey, rightKey, leftKey;
    public float xSens = 400f;
    public float ySens = 100f;
    float xRot, yRot;

    Vector2 screenRot, targetScreenRot, screenRotVel;
    public float screenRotHalflife= .1f;
    void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;
        kb = Keyboard.current;
        upKey = kb.wKey;
        downKey = kb.sKey;
        leftKey = kb.aKey;
        rightKey = kb.dKey;
    }

    // Update is called once per frame
    void Update()
    {

        Vector3 positionDelta = transform.forward * (upKey.isPressed ? 1f: 0f);
        positionDelta += -transform.forward * (downKey.isPressed ? 1f : 0f);
        positionDelta += transform.right * (rightKey.isPressed ? 1f : 0f);
        positionDelta += -transform.right * (leftKey.isPressed ? 1f : 0f);
        positionDelta = positionDelta.normalized * (horizontalVelPerSecond * Time.deltaTime);

        float yDelta = kb.spaceKey.isPressed ? 1f : 0f;
        yDelta += kb.ctrlKey.isPressed ? -1f : 0f;
        positionDelta.y = yDelta * verticalVelPerSec * Time.deltaTime;

        transform.position += positionDelta;

        targetScreenRot += Mouse.current.delta.ReadValue() * Time.deltaTime;
        SpringUtils.spring_character_update(screenRot, screenRotVel, targetScreenRot, screenRotHalflife, Time.deltaTime , out screenRot, out screenRotVel);

        Vector2 mouseDelta = screenRotVel;
        //mouseDelta.x *= xSens;
        //mouseDelta.y *= ySens;
        //mouseDelta.Normalize();
        yRot += mouseDelta.x;
        xRot -= mouseDelta.y;
        xRot = Mathf.Clamp(xRot, -90f, 90f);
        transform.rotation = Quaternion.Euler(xRot, yRot, 0f);

        //if (!Mouse.current.rightButton.isPressed)
        //    return;

        //Vector2 mouseDelta = Mouse.current.delta.ReadValue() * Time.deltaTime;
        //mouseDelta.x *= xSens;
        //mouseDelta.y *= ySens;
        ////mouseDelta.Normalize();
        ////mouseDelta *= mouseSensitivity * Time.fixedDeltaTime;
        ////transform.Rotate(new Vector3(0f, mouseDelta.x, 0f));
        //yRot += mouseDelta.x;
        //xRot -= mouseDelta.y;
        //xRot = Mathf.Clamp(xRot, -90f, 90f);
        //transform.rotation = Quaternion.Euler(xRot, yRot, 0f);


    }
}
