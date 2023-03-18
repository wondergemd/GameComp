using UnityEngine;
using UnityEngine.UI;
using Utils;
using System;
using System.Collections;
using System.Collections.Generic;


public class Player : MonoBehaviour
{
    // Connected scripts/game Objects
    public Vehicle playerVehicle;
    public Text DebugText;
    public AudioSource audioSource;
    public GameObject SteeringWheel;
    public PathFinder pathFinder;
    public TrafficGenerator trafficGenerator;

    private float textUpdateTimer = 0f;
    private MathLib.ExponentialSmoother accLongSmoother, accLatSmoother;
    private float longAcc, latAcc;
    private float accbrakeInput, steeringInput;


    [Header("Forward Collision Sensor")]
    public float holdTime = 2f;
    public float minForwardCollisionDist = 5f;
    public bool collisionDetected = false;
    public float collisionDetectedTime = 0.0f;


    [Space(10)]
    [Header("Blind Spot Sensors")]
    public Vector3 backSensorPos = new Vector3(-1f, 1f, 0.75f);
    public int blindSpotNumSensors = 5;
    public float blindSpotAngle = 20;
    public float blindSpotLength = 5f;
    public bool blindSpotLeft = false;
    public bool blindSpotRight = false;
    public float sameDirectionThreshold = 0.25f;


    [Space(10)]
    [Header("Steering Wheel")]
    public float maxSteeringWheelAngle = 180f;


    [Space(10)]
    [Header("Audio")]
    public bool audioToggle = true;


    // Debug variables, used in BrakeTesting and BrakeTestingMain only
    [Space(10)]
    [Header("Brake testing")]
    public bool testBrakes = false;
    public float brakeTime = 0f;
    public float brakeDistance = 0f;
    public float brakeSpeed = 0f;
    public float estDist = 0f;
    public float estTime = 0f;


    // Utility function for a single Raycast sensor.
    private RaycastHit Sensor(Vector3 sensorPos, float sensorAngle, float sensorLength)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(sensorAngle, playerVehicle.transform.up) * playerVehicle.transform.forward), out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            return hit;
        }
        else
        {
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(sensorAngle, playerVehicle.transform.up) * playerVehicle.transform.forward) * sensorLength, Color.red);
            return hit;
        }
    }


    // Utility function for multiple Raycast sensors. Each hit in list returned must be checked to ensure it is not null.
    private List<RaycastHit> SensorArray(int numSensors, float maxAngle, float sensorLength, Vector3 sensorOffset)
    {
        List<RaycastHit> hits = new List<RaycastHit>();

        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 sensorPos = playerVehicle.transform.position;
        sensorPos += (playerVehicle.transform.forward * sensorOffset.x);
        sensorPos += (playerVehicle.transform.up * sensorOffset.y);

        float angleSubtraction = (360f - (maxAngle * 2)) / (numSensors - 1);
        float tempSensorAngle = maxAngle;

        sensorPos += (playerVehicle.transform.right * sensorOffset.z);
        for (int i = 0; i < numSensors; i++)
        {
            hits.Add(Sensor(sensorPos, tempSensorAngle, sensorLength));
            tempSensorAngle -= angleSubtraction;
        }


        sensorPos -= (playerVehicle.transform.right * (sensorOffset.z * 2));
        tempSensorAngle = maxAngle;
        for (int i = 0; i < numSensors; i++)
        {
            hits.Add(Sensor(sensorPos, -tempSensorAngle, sensorLength));
            tempSensorAngle -= angleSubtraction;
        }

        return hits;
    }


    // Forward Collision Detection based on non-linear AI paths and linear physics equation
    private void ForwardCollisionDetection()
    {
        float distToVehicle = float.MaxValue;
        float aiSpeed = 0f;
        Vector3 aiAccVec = Vector3.zero;


        // 1. Find closest AI vehicle to player vehicle which share non-linear AI paths

        // get playerVehicle Segement info
        (Waypoint, Waypoint, float) currSeg = pathFinder.GetSegmentVehicleOn(playerVehicle.GetPosition());

        // get list of AI controlled vehicles from traffic Generator
        for (int i = 0; i < trafficGenerator.activeAIs.Count; i++)
        {
            // get single AI Vehicle 
            AI ai = trafficGenerator.activeAIs[i];
            Vehicle aiVeh = ai.vehicle;

            // get AI segment info
            AI.SegmentData aiSeg = ai.plan[Mathf.Min(ai.currSegIdx, ai.plan.Count - 1)];

            if (currSeg.Item1 != null && !(currSeg.Item2 == aiSeg.endWp && currSeg.Item3 > ai.currSegXnorm))
            {
                // get path distance (non-linear) between player vehicle and AI vehicle
                float distBetweenAI = pathFinder.DistanceToVehicleOnPathTrajectory(playerVehicle, aiVeh, currSeg.Item2, aiSeg.endWp);

                // if AI vehicle is closest to player vehicle in loop so far
                if (distBetweenAI < distToVehicle)
                {
                    distToVehicle = distBetweenAI;
                    aiSpeed = ai.vehicle.GetSpeed();
                    aiAccVec = ai.vehicle.GetAccelerationVec();
                }
            }
        }

        // 2. Determine if braking needs to be applied
        float aiAcc = Vector3.Dot(aiAccVec, playerVehicle.GetVelocity().normalized);

        float t = TimeToMatchSpeed(aiSpeed);

        // distance AI travels in time it takes player vehicle to stop | d = v0*t + 0.5*a*t^2
        float aiDistTraveledInTimeToBrake = (aiSpeed * t) + (0.5f * aiAcc * (Mathf.Pow(t, 2f)));

        // minimum distance to leave between vehicles
        float leewayDist = 7.5f;

        // distance between player vehicle and AI vehicle if player vehicle applies full brakes now. 
        float DistBetweenWithBraking = (aiDistTraveledInTimeToBrake + distToVehicle) - DistanceToMatchSpeed(aiSpeed) - leewayDist;

        // if estimated distance between player vehicle and AI vehicle after player vehicle theoretically applies
        // full brakes is less than minForwardCollisionDist, then apply emergency braking
        // records time to determine how long to hold emergency braking
        // Should probably update emergency braking to gradual system and make coroutine instead
        if (DistBetweenWithBraking <= 0)
        {
            collisionDetected = true;
            playerVehicle.Throttle = 0;
            playerVehicle.Brake = 1;
            //Debug.Log("Collision Detected");
        }
        else if (DistBetweenWithBraking <= minForwardCollisionDist)
        {
            collisionDetected = true;
        }
        else
        {
            collisionDetected = false;
        }
    }


    IEnumerator ForwardCollisionBraking()
    {

        yield return new WaitUntil(() => playerVehicle.GetSpeed() <= 0.1);

    }


    // Estimates distance needed to match given speed based on the vehicle's braking torque and mass, and current speed.
    private float DistanceToMatchSpeed(float finalSpeed)
    {
        if (finalSpeed > playerVehicle.GetSpeed())
        {
            return 0f;
        }
        float t = TimeToMatchSpeed(finalSpeed);
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return (playerVehicle.GetSpeed() * t) + ((0.5f) * -maxDeceleration * (Mathf.Pow(t, 2f)));
    }


    // Estimates time needed to match given speed based on the vehicle's braking torque and mass, and current speed.
    private float TimeToMatchSpeed(float finalSpeed)
    {
        if (finalSpeed > playerVehicle.GetSpeed())
        {
            return 0f;
        }
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return (finalSpeed - playerVehicle.GetSpeed()) / -maxDeceleration;
    }


    // Estimates time to stop based on the vehicle's braking torque and mass, and current speed.
    private float TimeToStop()
    {
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return playerVehicle.GetSpeed() / maxDeceleration;
    }
    

    // Estimate distance to stop based on the vehicle's braking torque and mass, and current speed.
    private float DistanceToStop()
    {
        float t = TimeToStop();
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return (playerVehicle.GetSpeed() * t) + ((0.5f) * -maxDeceleration * (Mathf.Pow(t, 2f)));
    }


    // Blind spot indicators for passing vehicles in same direction
    private void BlindSpotIndicator()
    {
        List<RaycastHit> hits = new List<RaycastHit>();

        //private List<RaycastHit> SensorArray(int numSensors, float maxAngle, float sensorLength, Vector3 sensorOffset)
        hits = SensorArray(blindSpotNumSensors, blindSpotAngle, blindSpotLength, backSensorPos);

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider != null)
            {
                // ensure hit object is a vehicle
                if (hit.transform.gameObject.tag == "vehicle")
                {
                    Vehicle AIvehicle = hit.transform.gameObject.GetComponent<Vehicle>();

                    // take dot product of velocities to check if vehicle is traveling in same direction as player (IE: passing player, not traveling on opposite road direction)
                    float dotProduct = Vector3.Dot(AIvehicle.GetVelocity().normalized, playerVehicle.GetVelocity().normalized);
                    Debug.Log(dotProduct);
                    if (dotProduct > sameDirectionThreshold)
                    {
                        // take cross product to determine if AI vehicle is on left or right side
                        Vector3 direction = (AIvehicle.transform.position - playerVehicle.transform.position).normalized;
                        Vector3 up = playerVehicle.transform.up;
                        Vector3 crossProduct = Vector3.Cross(direction, up);

                        if (crossProduct.y > 0)
                        {
                            blindSpotLeft = true;
                        }
                        else if (crossProduct.y < 0)
                        {
                            blindSpotRight = true;
                        }
                    }
                }
            }
        }

    }


    // Rotates steering wheel in direction of steering input
    private void SteeringWheelTurning(float steeringInput)
    {
        float angle = -steeringInput * maxSteeringWheelAngle;
        SteeringWheel.transform.localRotation = Quaternion.Euler(0, 0, angle);
    }


    // Coroutine used to test the equation used in the Forward Collision Function, estimating braking time and distance using the vehicle's current speed and acceleration
    // updates the brakeTime and brakeDistance variables with the actual time and distance it took to stop the vehicle
    // compare to estTime and estDist variables to see how accurate the equation is
    IEnumerator BrakeTesting()
    {
        brakeTime = 0f;
        brakeDistance = 0f;
        testBrakes = true;
        float initialTime = Time.time;
        Vector3 initialPos = playerVehicle.GetPosition();
        brakeSpeed = playerVehicle.GetSpeed();

        estTime = TimeToStop();
        estDist = DistanceToStop();

        yield return new WaitUntil(() => playerVehicle.GetSpeed() <= 0.1);

        float finalTime = Time.time;
        Vector3 finalPos = playerVehicle.GetPosition();

        brakeTime = finalTime - initialTime;
        brakeDistance = Vector3.Distance(initialPos, finalPos);

        testBrakes = false;
    }


    // Debug testing to ensure the equation estimating vehicle braking time/distance 
    // was correct. Can be added into script update function, activates brake testing with spacebar
    // Uses BrakeTesting coroutine. 
    private void BrakeTestingMain()
    {

        if (Input.GetKeyDown(KeyCode.Space) && !testBrakes)
        {
            StartCoroutine(BrakeTesting());
        }
        if (testBrakes)
        {
            playerVehicle.Throttle = 0;
            playerVehicle.Brake = 1;
        }
    }


    private void SetGear()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) //drive
            playerVehicle.gear = 1;
        else if (Input.GetKeyDown(KeyCode.Alpha2)) // neutral
            playerVehicle.gear = 2;
        else if (Input.GetKeyDown(KeyCode.Alpha3)) // park
            playerVehicle.gear = 3;
        else if (Input.GetKeyDown(KeyCode.Alpha4)) // reverse
            playerVehicle.gear = 4;
    }


    private void UserInput()
    {
        SetGear();
        // Positive for accelerating and negative for braking
        accbrakeInput = Input.GetAxis("Vertical");
        steeringInput = Input.GetAxis("Horizontal");
        //Debug.Log("accbrakeinput = " + accbrakeInput);

        // Vehicle with user input
        if (!collisionDetected)
        {
            playerVehicle.Throttle = accbrakeInput;
            playerVehicle.Brake = -accbrakeInput;
        }
        playerVehicle.Steering = steeringInput;

    }


    private void DebugTextUpdater()
    {
        Vector3 localAcc = Quaternion.Inverse(playerVehicle.GetRotation()) * playerVehicle.GetAccelerationVec();

        longAcc = accLongSmoother.Get(localAcc.z);
        latAcc = accLatSmoother.Get(localAcc.x);

        // Debug Text Updater
        if (textUpdateTimer > 0.1f)
        {
            textUpdateTimer = 0f;

            (Waypoint, Waypoint, float) currSeg = pathFinder.GetSegmentVehicleOn(playerVehicle.GetPosition());

            string debugStr = string.Format(
        "Throttle: {0:0.00}, Brake: {1:0.00}, Steering: {2:0.00}\n" +
        "Speed: {3:0.00} m/s, Long Acc: {4:0.0} m/s^2, Lat Acc: {5:0.0} m/s^2\n" +
        "Time to Stop: {6: 0.00}\n" +
        "Dist to Stop: {7: 0.00}",
        playerVehicle.Throttle, playerVehicle.Brake, playerVehicle.Steering, playerVehicle.GetSpeed(), longAcc, latAcc, TimeToStop(), DistanceToStop()
        );

            DebugText.text = debugStr;
        }
        textUpdateTimer += Time.fixedDeltaTime;
    }


    // Start is called before the first frame update
    void Start()
    {
        accLongSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
        accLatSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
    }


    public void FixedUpdate()
    {
        ForwardCollisionDetection();

        UserInput();

        SteeringWheelTurning(steeringInput);

        BlindSpotIndicator();

        DebugTextUpdater();


    }
}
