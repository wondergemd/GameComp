using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;

public class CameraSwitcher : MonoBehaviour
{


    public CinemachineVirtualCamera camera1; // Reference to the first virtual camera
    public CinemachineFreeLook camera2; // Reference to the second virtual camera


    // Start is called before the first frame update
    void Start()
    {
        camera1.enabled = true;
        camera2.enabled = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.F1))
        {
            // Toggle between the two virtual cameras
            camera1.enabled = !camera1.enabled;
            camera2.enabled = !camera2.enabled;

        }
    }
}
