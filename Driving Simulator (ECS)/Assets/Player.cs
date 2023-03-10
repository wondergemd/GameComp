using UnityEngine;
using UnityEngine.UI;
using Utils;
using System;


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

    // Private Variables
    private bool avoiding = false;
    private float avoidTime;
    public bool collisionDetected = false;
    private float collisionDetectedTime = 0.0f;
    


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
                float distBetweenNow = pathFinder.DistanceBetweenTwoPointsOnPath(playerVehicle.GetPosition(), aiVeh.GetPosition(), currSeg.Item2, aiSeg.endWp);

                minDist = Math.Min(minDist, distBetweenNow);

                /*
                // find closest distance player and AI vehicle will approach (distance = (player.velocity^2 - AI.velocity^2) / (2*player.acceleration))
                float distBetweenFuture = (Mathf.Pow(ai.vehicle.GetSpeed(), 2f) - Mathf.Pow(playerVehicle.GetSpeed(), 2f)) / (2f * playerVehicle.GetAcceleration());

                // activate emergency brakes if collision path detected within range
                float distBetweenDiff = distBetweenNow - distBetweenFuture;
                if (distBetweenDiff <= collisionDistanceRange && distBetweenDiff >= -collisionDistanceRange)
                {
                    collisionDetected = true;
                    collisionDetectedTime = Time.time;
                }
                */
            }
        }
        Debug.Log(minDist);
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



        // Debug Text Updater
        if (textUpdateTimer > 0.1f)
        {
            textUpdateTimer = 0f;

            (Waypoint, Waypoint, float) currSeg = pathFinder.GetSegmentVehicleOn(playerVehicle.GetPosition());

                string debugStr = string.Format(
            "Throttle: {0:0.00}, Brake: {1:0.00}, Steering: {2:0.00}\n" +
            "Speed: {3:0.00} m/s, Long Acc: {4:0.0} m/s^2, Lat Acc: {5:0.0} m/s^2\n" +
            "Avoiding: {6:0}\n" +
            "Seg1: {7: 0}, Seg2: {8: 0}, Seg%: {9: 0.0}",
            playerVehicle.Throttle, playerVehicle.Brake, playerVehicle.Steering, playerVehicle.GetSpeed(), longAcc, latAcc, avoiding, currSeg.Item1, currSeg.Item2, currSeg.Item3
            );

            DebugText.text = debugStr;
        }
        textUpdateTimer += Time.fixedDeltaTime;


    }
}
