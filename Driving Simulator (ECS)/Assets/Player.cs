using UnityEngine;
using UnityEngine.UI;
using Utils;
using System;
using System.Collections;


public class Player : MonoBehaviour
{
    // The controlled vehicle
    public Vehicle playerVehicle;
    public Text DebugText;
    public AudioSource audioSource;
    public GameObject SteeringWheel;
    public PathFinder pathFinder;
    public TrafficGenerator trafficGenerator;

    private float textUpdateTimer = 0f;
    private MathLib.ExponentialSmoother accLongSmoother, accLatSmoother;


    // Variables used for sensors/ Collision Avoidance

    [Header("Forward Collision Sensor")]
    public float frontSensorLength = 8f;
    public Vector3 frontSensorPos = new Vector3(2.25f, .5f, 1f);
    public float frontSensorAngle = 30;
    public float frontSensorMaxAngle = 45;
    public float numSensors = 5;
    public float sensorMult = 1.5f;
    public float holdTime = 2f;
    public float collisionDistanceRange = 5f;
    public float minForwardCollisionDist = 5f;

    [Space(10)]
    [Header("Blind Spot Sensors")]
    public Vector3 backSensorPos = new Vector3(-1f, 1f, 0.75f);
    public float blindSpotAngle = 20;
    public float blindSpotLength = 5f;
    public bool blindSpotLeft = false;
    public bool blindSpotRight = false;


    [Space(10)]
    [Header("Steering Wheel")]
    public float maxSteeringWheelAngle = 180f;


    [Space(10)]
    [Header("Audio")]
    public bool audioToggle = true;

    [Space(10)]
    [Header("Brake testing")]
    public bool testBrakes = false;
    public float brakeTime = 0f;
    public float brakeDistance = 0f;
    public float brakeSpeed = 0f;
    public float estDist = 0f;
    public float estTime = 0f;

    // Private Variables
    public bool collisionDetected = false;
    public float collisionDetectedTime = 0.0f;
    


    private void DetectCollision(Vehicle otherVehicle)
    {

    }

    private bool Sensor(Vector3 sensorPos, float sensorAngle, float sensorLength)
    {
        RaycastHit hit;
        if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(sensorAngle, playerVehicle.transform.up) * playerVehicle.transform.forward), out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            return true;
        }
        else
        {
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(sensorAngle, playerVehicle.transform.up) * playerVehicle.transform.forward) * sensorLength, Color.red);
            return false;
        }
    }

    private void ForwardCollisionDetectionSensors()
    {
        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 sensorPos = playerVehicle.transform.position;
        sensorPos += (playerVehicle.transform.forward * frontSensorPos.x);
        sensorPos += (playerVehicle.transform.up * frontSensorPos.y);

        // Sensor length is determined by the square of the vehicle's speed
        //frontSensorLength = Mathf.Pow(Vehicle.GetSpeed(), 2f) * sensorMult;

        float frontSensorAngleSub = (frontSensorMaxAngle * 2) / (numSensors - 1);
        float tempSensorAngle = frontSensorMaxAngle;
        for (int i = 0; i < numSensors; i++)
        {
            Sensor(sensorPos, tempSensorAngle, frontSensorLength);
            tempSensorAngle -= frontSensorAngleSub;
        }

    }

    private void ForwardCollisionDetection()
    {
        float minDist = float.MaxValue;
        float aiSpeed = 0f;
        float aiAcc = 0f;

        // get playerVehicle Segement info
        (Waypoint, Waypoint, float) currSeg = pathFinder.GetSegmentVehicleOn(playerVehicle.GetPosition());

        // get list of AI controlled vehicles from traffic Generator
        for (int i = 0; i < trafficGenerator.activeAIs.Count; i++)
        {
            // get single AI Vehicle 
            AI ai = trafficGenerator.activeAIs[i];
            Vehicle aiVeh = ai.vehicle;

            AI.SegmentData aiSeg = ai.plan[Mathf.Min(ai.currSegIdx, ai.plan.Count - 1)];

            if (currSeg.Item1 != null && !(currSeg.Item2 == aiSeg.endWp && currSeg.Item3 > ai.currSegXnorm))
            {
                // get path distance (non-linear) between player vehicle and AI vehicle
                float distBetweenAI = pathFinder.DistanceBetweenTwoPointsOnPath(playerVehicle.GetPosition(), aiVeh.GetPosition(), currSeg.Item2, aiSeg.endWp);

                if (distBetweenAI < minDist)
                {
                    minDist = distBetweenAI;
                    aiSpeed = ai.vehicle.GetSpeed();
                    aiAcc = ai.vehicle.GetAcceleration();
                }
            }
        }

        float t = TimeToStop();

        float aiDistTraveledInTimeToBrake = (aiSpeed * t) + (0.5f * aiAcc * (Mathf.Pow(t, 2f)));

        float DistBetweenWithBraking = (aiDistTraveledInTimeToBrake + minDist) - DistanceToStop();



        if (DistBetweenWithBraking <= minForwardCollisionDist)
        {
            collisionDetected = true;
            collisionDetectedTime = Time.time;
            Debug.Log("COLLISION DETECTED EMERGENCY BRAKING");
        }



    }


    /*
// Center Sensor
if (Physics.Raycast(sensorPos, Vehicle.transform.forward, out hit, frontSensorLength))
{
    Debug.DrawLine(sensorPos, hit.point, Color.green);
    avoiding = true;
    avoidTime = Time.time;
}
else
{
    Debug.DrawRay(sensorPos, Vehicle.transform.forward * frontSensorLength, Color.red);
}

// Left Sensor
blindSpotSensorPos -= (Vehicle.transform.right * backSensorPos.z) * 2;
if (Physics.Raycast(blindSpotSensorPos, (Quaternion.AngleAxis(frontSensorAngle, Vehicle.transform.up) * -Vehicle.transform.forward), out hit, frontSensorLength))
{
    Debug.DrawLine(blindSpotSensorPos, hit.point, Color.green);
    blindSpotLeft = true;
}
else
{
    Debug.DrawRay(blindSpotSensorPos, (Quaternion.AngleAxis(frontSensorAngle, Vehicle.transform.up) * -Vehicle.transform.forward) * blindSpotLength, Color.red);
}
*/

    private void EmergencyBraking()
    {
        // Holds emergency braking for a set amount of time to override player input and allow vehicle to actually brake
        if (collisionDetected && Time.time - collisionDetectedTime >= holdTime)
        {
            collisionDetected = false;
        }

        if (collisionDetected)
        {
            playerVehicle.Throttle = 0;
            playerVehicle.Brake = 1;
        }
    }


    private float TimeToStop()
    {
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return playerVehicle.GetSpeed() / maxDeceleration;
    }

    private float DistanceToStop()
    {
        float t = TimeToStop();
        float maxDeceleration = playerVehicle.maxBrakeTorque / (playerVehicle.GetMass() / 10);
        return (playerVehicle.GetSpeed() * t) + ((0.5f) * -maxDeceleration * (Mathf.Pow(t, 2f)));
    }


    private void TestBrakes()
    {
        float initialTime = Time.time;
        Vector3 initialDist = playerVehicle.GetPosition();
        brakeSpeed = playerVehicle.GetSpeed();

        while (playerVehicle.GetSpeed() != 0)
        {
            playerVehicle.Throttle = 0;
            playerVehicle.Brake = 1;
        }

        float finalTime = Time.time;
        Vector3 finalDist = playerVehicle.GetPosition();

        brakeDistance = Vector3.Distance(initialDist, finalDist);
        brakeTime = finalTime - initialTime;
        testBrakes = false;

    }


    // Start is called before the first frame update
    void Start()
    {
        accLongSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
        accLatSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
    }



    private void BlindSpotIndicator()
    {
        RaycastHit hit;
        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 blindSpotSensorPos = playerVehicle.transform.position;
        blindSpotSensorPos += (playerVehicle.transform.forward * backSensorPos.x);
        blindSpotSensorPos += (playerVehicle.transform.up * backSensorPos.y);
        blindSpotSensorPos += (playerVehicle.transform.right * backSensorPos.z);

        // Right Sensor
        if (Physics.Raycast(blindSpotSensorPos, (Quaternion.AngleAxis(-blindSpotAngle, playerVehicle.transform.up) * -playerVehicle.transform.forward), out hit, blindSpotLength))
        {
            Debug.DrawLine(blindSpotSensorPos, hit.point, Color.green);
            blindSpotRight = true;
        }
        else
        {
            Debug.DrawRay(blindSpotSensorPos, (Quaternion.AngleAxis(-blindSpotAngle, playerVehicle.transform.up) * -playerVehicle.transform.forward) * blindSpotLength, Color.red);
        }

        // Left Sensor
        blindSpotSensorPos -= (playerVehicle.transform.right * backSensorPos.z) * 2;
        if (Physics.Raycast(blindSpotSensorPos, (Quaternion.AngleAxis(blindSpotAngle, playerVehicle.transform.up) * -playerVehicle.transform.forward), out hit, blindSpotLength))
        {
            Debug.DrawLine(blindSpotSensorPos, hit.point, Color.green);
            blindSpotLeft = true;
        }
        else
        {
            Debug.DrawRay(blindSpotSensorPos, (Quaternion.AngleAxis(blindSpotAngle, playerVehicle.transform.up) * -playerVehicle.transform.forward) * blindSpotLength, Color.red);
        }
        

    }

    // Rotates steering wheel in direction of steering input
    private void SteeringWheelTurning(float steeringInput)
    {
        float angle = -steeringInput * maxSteeringWheelAngle;
        SteeringWheel.transform.localRotation = Quaternion.Euler(0, 0, angle);
    }


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

        //yield return null;
    }

    /*
    public bool testBrakes = false;
    public float brakeTime = 0f;
    public float brakeDistance = 0f;
    public float brakeSpeed = 0f;
    */

    public void FixedUpdate()
    {

        // Positive for accelerating and negative for braking
        float accbrakeInput = Input.GetAxis("Vertical");
        float steeringInput = Input.GetAxis("Horizontal");
        //Debug.Log("accbrakeinput = " + accbrakeInput);


        // Vehicle with user input
        playerVehicle.Throttle = accbrakeInput;
        playerVehicle.Brake = -accbrakeInput;
        playerVehicle.Steering = steeringInput;

        Vector3 localAcc = Quaternion.Inverse(playerVehicle.GetRotation()) * playerVehicle.GetAccelerationVec();

        float longAcc = accLongSmoother.Get(localAcc.z);
        float latAcc = accLatSmoother.Get(localAcc.x);


        // Detect object & override AI pathing if detected
        SteeringWheelTurning(steeringInput);
        BlindSpotIndicator();
        ForwardCollisionDetection();
        EmergencyBraking();

        if (Input.GetKeyDown(KeyCode.Space) && !testBrakes)
        {
            StartCoroutine(BrakeTesting());
        }
        if (testBrakes)
        {
            playerVehicle.Throttle = 0;
            playerVehicle.Brake = 1;
        }

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
}
