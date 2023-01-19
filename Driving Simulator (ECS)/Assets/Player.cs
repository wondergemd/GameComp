using UnityEngine;
using UnityEngine.UI;

public class Player : MonoBehaviour
{
    // The controlled vehicle
    public Vehicle Vehicle;
    public Text DebugText;

    private float textUpdateTimer = 0f;

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

        //Vehicle.Throttle = accbrakeInput;
        //Vehicle.Brake = -accbrakeInput;
        //Vehicle.Steering = steeringInput;

        Vector3 localAcc = Quaternion.Inverse(Vehicle.GetRotation()) * Vehicle.GetAccelerationVec();

        float longAcc = localAcc.z;
        float latAcc = localAcc.x;

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

        textUpdateTimer += Time.deltaTime;
    }
}
