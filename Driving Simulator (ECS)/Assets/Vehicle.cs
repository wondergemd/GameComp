// https://docs.unity3d.com/Manual/WheelColliderTutorial.html

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public GameObject leftWheelMeshGameObject;
    public GameObject rightWheelMeshGameObject;
    public bool motor;
    public bool steering;
}

public class Vehicle : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxBrakeTorque;
    public float maxSteeringAngle;

    private Rigidbody rb;

    private float _throttle;
    public float Throttle
    {
        get { return _throttle; }
        set { _throttle = Mathf.Clamp01(value); }
    }

    private float _brake;
    public float Brake
    {
        get { return _brake; }
        set { _brake = Mathf.Clamp01(value); }
    }

    private float _steering;
    public float Steering
    {
        get { return _steering; }
        set { _steering = Mathf.Clamp(value, -1.0f, 1.0f); }
    }

    private Vector3 lastVelocity = Vector3.zero;
    private Vector3 acceleration = Vector3.zero;

    public void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void FixedUpdate()
    {
        Vector3 currVelocity = GetVelocity();
        acceleration = (currVelocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = currVelocity;

        // Calculate motor and braking torque and steering angle based on user inputs
        float motorTorque = maxMotorTorque * Throttle;
        float brakeTorque = maxBrakeTorque * Brake;
        float steeringAngle = maxSteeringAngle * Steering;

        // Set torques and steering angles to wheel collider objects
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steeringAngle;
                axleInfo.rightWheel.steerAngle = steeringAngle;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motorTorque;
                axleInfo.rightWheel.motorTorque = motorTorque;
            }

            axleInfo.leftWheel.brakeTorque = brakeTorque;
            axleInfo.rightWheel.brakeTorque = brakeTorque;

            // Pivot steered wheels visually
            ApplyLocalPositionToVisuals(axleInfo.leftWheel, axleInfo.leftWheelMeshGameObject);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel, axleInfo.rightWheelMeshGameObject);
        }
    }

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider, GameObject gameObject)
    {
        Transform visualWheel = gameObject.transform;

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    public Vector3 GetPosition()
    {
        return rb != null ? rb.position : transform.position;
    }

    public Vector3 GetVelocity()
    {
        return rb != null ? rb.velocity : Vector3.zero;
    }

    public float GetSpeed()
    {
        return GetVelocity().magnitude;
    }

    public Vector3 GetAccelerationVec()
    {
        return acceleration;
    }

    public float GetAcceleration()
    {
        return acceleration.magnitude;
    }

    public Quaternion GetRotation()
    {
        return rb != null ? rb.rotation : Quaternion.identity;
    }
}