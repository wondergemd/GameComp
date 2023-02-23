using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrafficGenerator : MonoBehaviour
{
    public PathFinder pathFinder;
    public List<GameObject> spawnList = new List<GameObject>();
    public int MaxAICars = 3;
    
    public List<AI> activeAIs = new List<AI>();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Add AI cars
        if (activeAIs.Count < MaxAICars)
        {
            int numToSpawn = MaxAICars - activeAIs.Count;

            List<Waypoint> canidateWps = new List<Waypoint>(pathFinder.spawnWaypoints);

            for (int i = 0; i < numToSpawn; i++)
            {
                int randIdx = Random.Range(0, canidateWps.Count - 1);
                
                Waypoint spawnWp = canidateWps[randIdx];
                Vector3 spawnPos = spawnWp.GetPosition();
                Waypoint destWp = pathFinder.FurthestWaypoint(spawnWp);

                var path = pathFinder.CalculatePath(spawnWp, destWp);

                // Use 1st and 2nd waypoint to calculate spawn rotation
                Quaternion spawnRot = Quaternion.LookRotation((path[1].GetPosition() - path[0].GetPosition()).normalized);

                GameObject newVehObj = Instantiate(spawnList[Random.Range(0, spawnList.Count - 1)], spawnPos, spawnRot);

                AI ai = newVehObj.GetComponent<AI>();
                ai.FollowPath(path);

                activeAIs.Add(ai);

                canidateWps.RemoveAt(randIdx);
            }
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

    private void OnDrawGizmos()
    {
        if (pathFinder && pathFinder.spawnWaypoints != null)
        {
            foreach (Waypoint wp in pathFinder.spawnWaypoints)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(wp.GetPosition(), 1);
            }
        }
    }
}
