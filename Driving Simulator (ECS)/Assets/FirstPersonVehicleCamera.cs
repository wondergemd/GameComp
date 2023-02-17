using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FirstPersonVehicleCamera : MonoBehaviour
{
    public GameObject gameObjectToFollow;

    public float distance;
    public float roll;
    public float moveFactor;
    public Vector3 CameraOffset = new Vector3(0.35f, 1.03f, -0.425f);
    public Vector3 LookAtOffset = new Vector3(1000f, 1f, 0f);

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
        if (Input.GetMouseButton(1))
        {
            roll += Input.GetAxis("Mouse X") * moveFactor;
        }

        /*
        Quaternion yawQuat = Quaternion.AngleAxis(yaw, gameObjectToFollow.transform.up);
        Quaternion pitchQuat = Quaternion.AngleAxis(5f, gameObjectToFollow.transform.right);
        Quaternion rollQuat = Quaternion.AngleAxis(pitch, gameObjectToFollow.transform.forward);

        transform.position = distance * (yawQuat * pitchQuat * rollQuat) + gameObjectToFollow.transform.position;
        transform.LookAt(gameObjectToFollow.transform);
        */


        
        transform.position = gameObjectToFollow.transform.position;
        transform.position += (gameObjectToFollow.transform.forward * CameraOffset.x);
        transform.position += (gameObjectToFollow.transform.up * CameraOffset.y);
        transform.position += (gameObjectToFollow.transform.right * CameraOffset.z);

        

        Vector3 CamLookAt = gameObjectToFollow.transform.position;
        CamLookAt += (gameObjectToFollow.transform.forward * LookAtOffset.x);
        CamLookAt += (gameObjectToFollow.transform.up * LookAtOffset.y);
        transform.LookAt(CamLookAt);

        transform.Rotate(0f, 0f, gameObjectToFollow.transform.rotation.eulerAngles.z);

    }
}
