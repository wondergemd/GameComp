using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerCameraLookAtTarget : MonoBehaviour
{
    public Vehicle Vehicle;

    // Start is called before the first frame update
    void Start()
    {
        transform.position = Vehicle.transform.position;
        transform.position += (Vehicle.transform.forward * 100f);
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = Vehicle.transform.position;
        transform.position += (Vehicle.transform.forward * 100f);
    }
}
