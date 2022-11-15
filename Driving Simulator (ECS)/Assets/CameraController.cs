using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public GameObject gameObjectToFollow;

    public float distance;
    public float yaw;
    public float pitch;
    public float moveFactor;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButton(1))
        {
            yaw += Input.GetAxis("Mouse X") * moveFactor;
            pitch += Input.GetAxis("Mouse Y") * moveFactor;
        }

        Quaternion yawQuat = Quaternion.AngleAxis(yaw, gameObjectToFollow.transform.up);
        Quaternion pitchQuat = Quaternion.AngleAxis(pitch, gameObjectToFollow.transform.right);

        transform.position = distance * (yawQuat * pitchQuat * gameObjectToFollow.transform.forward) + gameObjectToFollow.transform.position;
        transform.LookAt(gameObjectToFollow.transform);
    }
}
