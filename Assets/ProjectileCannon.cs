using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
public class ProjectileCannon : MonoBehaviour
{
    public GameObject projectilePrefab;

    public float mass = 10f;
    public bool gravity = false;
    public float force = 10f;
    public ForceMode forceType = ForceMode.Force;
    public bool continousFire;
    public float continousFireSleepTime = 1f;
    public float offset = .5f;
    public bool spawnWithTorque;
    public float torqueForce = 1f;
    float lastFireTime = -1f;

    Mouse mouse;
    GameObject projectileInstance;
    Rigidbody projectileRb;
    private void Start()
    {
        mouse = Mouse.current;
    }
    // Update is called once per frame
    void Update()
    {
        if (mouse.leftButton.wasPressedThisFrame || (continousFire && (Time.time - lastFireTime > continousFireSleepTime))) { 
            fireProjectile();
            lastFireTime = Time.time;
        }

    }

    void fireProjectile ()
    {
        if (projectileInstance == null) { 
            projectileInstance = Instantiate(projectilePrefab, transform.position, Quaternion.identity);
            projectileRb = projectileInstance.GetComponent<Rigidbody>();
        }
        projectileRb.useGravity = gravity;
        projectileRb.mass = mass;
        projectileRb.velocity = Vector3.zero;
        projectileRb.angularVelocity = Vector3.zero;

        Vector3 forward = transform.forward;
        projectileInstance.transform.position = transform.position + forward * offset;
        projectileRb.AddForce(forward * force, forceType);
        if (spawnWithTorque)
            projectileRb.AddTorque(new Vector3(Random.value, Random.value, Random.value).normalized * torqueForce, forceType);

    }
}
