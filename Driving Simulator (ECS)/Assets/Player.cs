using UnityEngine;
using UnityEngine.UI;
using Utils;
using System;


public class Player : MonoBehaviour
{
    // The controlled vehicle
    public Vehicle Vehicle;
    public Text DebugText;
    public AudioSource audioSource;
    public GameObject SteeringWheel;

    private float textUpdateTimer = 0f;
    private MathLib.ExponentialSmoother accLongSmoother, accLatSmoother;


    // Variables used for sensors/ Collision Avoidance

    [Header("Forward Collision Sensor")]
    public float sensorLength = 8f;
    public Vector3 frontSensorPos = new Vector3(2.25f, .5f, 0f);
    public float frontSensorAngle = 30;
    public float sensorSidePositions = 0.75f;
    public float sensorMult = 1.5f;
    public float holdTime = 2f;

    [Space(10)]
    [Header("Blind Spot Sensors")]
    public Vector3 backSensorPos = new Vector3(-1f, 1f, 0.75f);
    public float blindSpotAngle = 20;
    public float blindSpotLength = 5f;


    [Space(10)]
    [Header("Steering Wheel")]
    public float maxSteeringWheelAngle = 180f;


    [Space(10)]
    [Header("Audio")]
    public bool audioToggle = true;

    // Private Variables
    private float blindTime;
    private bool blindSpot = false;
    private bool avoiding = false;
    private float avoidTime;


    private void ForwardCollisionDetection()
    {
        RaycastHit hit;
        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 sensorPos = Vehicle.transform.position;
        sensorPos += (Vehicle.transform.forward * frontSensorPos.x);
        sensorPos += (Vehicle.transform.up * frontSensorPos.y);

        // Sensor length is determined by the square of the vehicle's speed
        sensorLength = Mathf.Pow(Vehicle.GetSpeed(), 2f) * sensorMult; 

        if (Physics.Raycast(sensorPos, Vehicle.transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidTime = Time.time;
        }
        else
        {
            Debug.DrawRay(sensorPos, Vehicle.transform.forward * sensorLength, Color.red);
        }
    }


    private void EmergencyBraking()
    {
        // Holds emergency braking for a set amount of time to override player input and allow vehicle to actually brake
        if (avoiding && Time.time - avoidTime >= holdTime)
        {
            avoiding = false;
        }

        if (avoiding)
        {
            Vehicle.Throttle = 0;
            Vehicle.Brake = 1;
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
        Vector3 blindSpotSensorPos = Vehicle.transform.position;
        blindSpotSensorPos += (Vehicle.transform.forward * backSensorPos.x);
        blindSpotSensorPos += (Vehicle.transform.up * backSensorPos.y);
        blindSpotSensorPos += (Vehicle.transform.right * backSensorPos.z);


        if (blindSpot && Time.time - blindTime >= holdTime)
        {
            blindSpot = false;
        }

        // Right Sensor
        if (Physics.Raycast(blindSpotSensorPos, (Quaternion.AngleAxis(-blindSpotAngle, Vehicle.transform.up) * -Vehicle.transform.forward), out hit, blindSpotLength))
        {
            Debug.DrawLine(blindSpotSensorPos, hit.point, Color.green);
            blindSpot = true;
            blindTime = Time.time;
        }
        else
        {
            Debug.DrawRay(blindSpotSensorPos, (Quaternion.AngleAxis(-blindSpotAngle, Vehicle.transform.up) * -Vehicle.transform.forward) * blindSpotLength, Color.red);
        }

        // Left Sensor
        blindSpotSensorPos -= (Vehicle.transform.right * backSensorPos.z) * 2;
        if (Physics.Raycast(blindSpotSensorPos, (Quaternion.AngleAxis(blindSpotAngle, Vehicle.transform.up) * -Vehicle.transform.forward), out hit, blindSpotLength))
        {
            Debug.DrawLine(blindSpotSensorPos, hit.point, Color.green);
            blindSpot = true;
            blindTime = Time.time;
        }
        else
        {
            Debug.DrawRay(blindSpotSensorPos, (Quaternion.AngleAxis(blindSpotAngle, Vehicle.transform.up) * -Vehicle.transform.forward) * blindSpotLength, Color.red);
        }
        

    }


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

        Vehicle.Throttle = accbrakeInput;
        Vehicle.Brake = -accbrakeInput;
        Vehicle.Steering = steeringInput;

        Vector3 localAcc = Quaternion.Inverse(Vehicle.GetRotation()) * Vehicle.GetAccelerationVec();

        float longAcc = accLongSmoother.Get(localAcc.z);
        float latAcc = accLatSmoother.Get(localAcc.x);

        // Detect object & override AI pathing if detected
        BlindSpotIndicator();
        ForwardCollisionDetection();
        EmergencyBraking();
        SteeringWheelTurning(steeringInput);

        if (textUpdateTimer > 0.1f)
        {
            textUpdateTimer = 0f;

            string debugStr = string.Format(
            "Throttle: {0:0.00}, Brake: {1:0.00}, Steering: {2:0.00}\n" +
            "Speed: {3:0.00} m/s, Long Acc: {4:0.0} m/s^2, Lat Acc: {5:0.0} m/s^2\n" +
            "Avoiding: {1:0}",
            Vehicle.Throttle, Vehicle.Brake, Vehicle.Steering, Vehicle.GetSpeed(), longAcc, latAcc, avoiding
            );

            DebugText.text = debugStr;
        }

        textUpdateTimer += Time.fixedDeltaTime;


    }
}
