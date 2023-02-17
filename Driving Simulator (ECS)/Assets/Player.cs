using UnityEngine;
using UnityEngine.UI;
using Utils;

public class Player : MonoBehaviour
{
    // The controlled vehicle
    public Vehicle Vehicle;
    public Text DebugText;
    public AudioSource audioSource;

    private float textUpdateTimer = 0f;
    private MathLib.ExponentialSmoother accLongSmoother, accLatSmoother;


    // Variables used for sensors/ Collision Avoidance
    public float sensorLength = 8f;
    public Vector3 frontSensorPos = new Vector3(2.25f, .5f, 0f);
    public Vector3 sideSensorPos = new Vector3(0.5f, .5f, 0.75f);
    public float frontSensorAngle = 30;
    public float sensorSidePositions = 0.75f;
    public bool avoiding = false;
    public float avoidMult = 0f;
    public float turnSpeed = 1f;
    private float avoidTargetSteerAngle = 0f;
    public bool stopToggle = false;

    public bool audioToggle = true;


    // Collision Avoidance/ADAS
    private void Sensors()
    {
        RaycastHit hit;
        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 sensorPos = Vehicle.transform.position;
        sensorPos += (Vehicle.transform.forward * frontSensorPos.x);
        sensorPos += (Vehicle.transform.up * frontSensorPos.y);

        // if object is detected, avoidMult determines steering direction
        avoiding = false;
        avoidMult = 0;


        // front left sensors
        sensorPos -= Vehicle.transform.right * sensorSidePositions;
        if (Physics.Raycast(sensorPos, Vehicle.transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 1.0f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(-frontSensorAngle, Vehicle.transform.up) * Vehicle.transform.forward), out hit, sensorLength * 0.75f))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 0.75f;
        }
        else if (Physics.Raycast(sensorPos, Quaternion.AngleAxis(-45, Vehicle.transform.up) * Vehicle.transform.forward, out hit, sensorLength * 0.5f))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 0.5f;
        }
        else
        {
            Debug.DrawRay(sensorPos, Vehicle.transform.forward * sensorLength, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(-frontSensorAngle, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength * 0.75f, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(-45, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength * 0.5f, Color.red);
        }


        // front right
        sensorPos += (Vehicle.transform.right * sensorSidePositions) * 2;
        if (Physics.Raycast(sensorPos, Vehicle.transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 1.0f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(frontSensorAngle, Vehicle.transform.up) * Vehicle.transform.forward) * 0.75f, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 0.75f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(45, Vehicle.transform.up) * Vehicle.transform.forward) * 0.5f, out hit, sensorLength / 2))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 0.5f;
        }
        else
        {
            Debug.DrawRay(sensorPos, Vehicle.transform.forward * sensorLength, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(frontSensorAngle, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength * 0.75f, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(45, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength * 0.5f, Color.red);
        }


        // front center
        sensorPos -= Vehicle.transform.right * sensorSidePositions;
        if (Physics.Raycast(sensorPos, Vehicle.transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;

            // if object is detected at front sensor, check the normal of the raycast to determine direction to turn
            if (hit.normal.x < 0)
            {
                avoidMult = +1;
            }
            else if (hit.normal.x > 0)
            {
                avoidMult = -1;
            }
        }
        else
        {
            Debug.DrawRay(sensorPos, Vehicle.transform.forward * sensorLength, Color.red);
        }

    }


    private void SideSensors()
    {
        RaycastHit hit;

        Vector3 sensorPos = Vehicle.transform.position;
        sensorPos += (Vehicle.transform.forward * sideSensorPos.x);
        sensorPos += (Vehicle.transform.up * sideSensorPos.y);

        // Side Sensors
        sensorPos += (Vehicle.transform.right * sideSensorPos.z);
        if (Physics.Raycast(sensorPos, Quaternion.AngleAxis(-10, Vehicle.transform.forward) * Quaternion.AngleAxis(90, Vehicle.transform.up) * Vehicle.transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
        }
        else
        {
            Debug.DrawRay(sensorPos, ((Quaternion.AngleAxis(-10, Vehicle.transform.forward)) * Quaternion.AngleAxis(90, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength, Color.red);
        }

        sensorPos -= (Vehicle.transform.right * sideSensorPos.z) * 2;
        if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(10, Vehicle.transform.forward) * Quaternion.AngleAxis(-90, Vehicle.transform.up) * Vehicle.transform.forward), out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
        }
        else
        {
            Debug.DrawRay(sensorPos, ((Quaternion.AngleAxis(10, Vehicle.transform.forward)) * Quaternion.AngleAxis(-90, Vehicle.transform.up) * Vehicle.transform.forward) * sensorLength, Color.red);
        }
    }


    private void AvoidObject()
    {
        if (avoiding)
        {
            //Debug.Log("AvoidMult = " + avoidMult);
            if (avoidMult == 0)
            {

            }
            avoidTargetSteerAngle = Vehicle.maxSteeringAngle * avoidMult;
            // interpolate target steering angle with the current steering angle over time
            Vehicle.Steering = Mathf.Lerp(Vehicle.Steering, avoidTargetSteerAngle, Time.deltaTime * turnSpeed);
            //Vehicle.Brake = 10000;
            Vehicle.Throttle = 0;

            audioSource.Play();
        }
    }


    // Start is called before the first frame update
    void Start()
    {
        accLongSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
        accLatSmoother = new MathLib.ExponentialSmoother(30, 0.0f, Time.fixedDeltaTime);
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

        if (textUpdateTimer > 0.1f)
        {
            textUpdateTimer = 0f;

            string debugStr = string.Format(
            "Throttle: {0:0.00}, Brake: {1:0.00}, Steering: {2:0.00}\n" +
            "Speed: {3:0.00} m/s, Long Acc: {4:0.0} m/s^2, Lat Acc: {5:0.0} m/s^2",
            Vehicle.Throttle, Vehicle.Brake, Vehicle.Steering, Vehicle.GetSpeed(), longAcc, latAcc
            );

            DebugText.text = debugStr;
        }

        textUpdateTimer += Time.fixedDeltaTime;

        // Detect object & override AI pathing if detected
        Sensors();
        SideSensors();
        AvoidObject();
    }
}
