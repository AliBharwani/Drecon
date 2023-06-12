using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
public class PlayerCamTarget : MonoBehaviour
{
    Transform target;
    Vector3 targetLastPos;
    Vector3 velocity = Vector3.zero;
    public float halflife = .15f;
    public float mouseRotSensitivity = 20f;
    public float gamepadRotSensitivity = 100f;

    public float xRot, yRot;
    Vector2 rotVel, rotAccel;
    public void init(Transform _target)
    {
        target = _target;
        transform.position = target.position;
        targetLastPos = target.position;
    }

    void Update()
    {
        updateRotation();
        Vector3 targetPos = target.position;
        if ((targetPos - targetLastPos).magnitude > 1f)
        {
            targetLastPos = targetPos;
            transform.position = targetPos;
            return;
        }
        Vector3 currentPos = transform.position;
        SpringUtils.simple_spring_damper_exact(ref currentPos, ref velocity, target.position, halflife, Time.deltaTime);
        transform.position = currentPos;
        targetLastPos = targetPos;
    }

    void updateRotation()
    {
        Vector2 rotUpdateInput;
        if (Gamepad.current != null && Gamepad.current.rightStick.ReadValue() != Vector2.zero)
            rotUpdateInput = Gamepad.current.rightStick.ReadValue() * gamepadRotSensitivity;
        else if (Mouse.current.delta.ReadValue() != Vector2.zero && Cursor.lockState == CursorLockMode.Locked)
            rotUpdateInput = Mouse.current.delta.ReadValue() * mouseRotSensitivity;
        else
            return;
        //Debug.Log($"RotUpdateInput: {rotUpdateInput}");
        SpringUtils.spring_character_update(rotVel, rotAccel, rotUpdateInput * Time.deltaTime, halflife, Time.deltaTime, out rotVel, out rotAccel);
        Vector2 delta = rotVel;
        yRot += delta.x;
        xRot -= delta.y;
        xRot = Mathf.Clamp(xRot, -90f, 90f);
        transform.rotation = Quaternion.Euler(xRot, yRot, 0f);
    }
}
