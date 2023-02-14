using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrafficGenerator : MonoBehaviour
{
    public int MaxAICars = 3;

    public List<GameObject> spawnList = new List<GameObject>();

    List<AI> activeAIs = new List<AI>();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Add AI cars
        while (activeAIs.Count < MaxAICars)
        {
            Vector3 spawnPos = PathFinder.RandomSpawnWaypoint().GetPosition();
            Quaternion spawnRot = Quaternion.identity;

            GameObject newVehObj = Instantiate(spawnList[Random.Range(0, spawnList.Count - 1)], spawnPos, spawnRot);

            AI ai = newVehObj.GetComponent<AI>();
            ai.GoToTarget(PathFinder.FurthestWaypoint(spawnPos));
            activeAIs.Add(ai);
        }

        // If an AI car has reached destination, despawn the car
        for (int i = 0; i < activeAIs.Count; i++)
        {
            AI aiCar = activeAIs[i];

            if (aiCar.HasReachedDestination())
            {
                DestroyImmediate(aiCar.transform.gameObject);
                activeAIs.RemoveAt(i);
                i--;
            }
        }
    }
}
