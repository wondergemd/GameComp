using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Player : MonoBehaviour
{
    // The controlled vehicle
    public Vehicle Vehicle;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    public void FixedUpdate()
    {
        // Positive for accelerating and negative for braking
        float accbrakeInput = Input.GetAxis("Vertical");
        float steeringInput = Input.GetAxis("Horizontal");

        Vehicle.Throttle = accbrakeInput;
        Vehicle.Brake = -accbrakeInput;
        Vehicle.Steering = steeringInput;
    }
}
