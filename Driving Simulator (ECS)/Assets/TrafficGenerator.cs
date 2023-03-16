using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrafficGenerator : MonoBehaviour
{
    public PathFinder pathFinder;
    public List<GameObject> spawnList = new List<GameObject>();
    public int MaxAICars = 3;
    
    public List<AI> activeAIs = new List<AI>();

    private HashSet<Waypoint> wpsInUse = new HashSet<Waypoint>();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        AddAIVehicles();
        RemoveAIVehicles();
    }

    private void AddAIVehicles()
    {
        if (activeAIs.Count < MaxAICars)
        {
            wpsInUse.Clear();

            // Get waypoints to spawn at (not in path of current AI vehicles)
            for (int i = 0; i < activeAIs.Count; i++)
            {
                AI ai = activeAIs[i];
                foreach (AI.SegmentData seg in ai.plan)
                {
                    wpsInUse.Add(seg.startWp);
                }
            }

            List<Waypoint> canidateWps = new List<Waypoint>(pathFinder.spawnWaypoints);
            canidateWps.RemoveAll(wp => wpsInUse.Contains(wp));
            if (canidateWps.Count == 0) return;

            int numToSpawn = MaxAICars - activeAIs.Count;

            for (int i = 0; i < numToSpawn; i++)
            {
                int randIdx = Random.Range(0, canidateWps.Count - 1);

                Waypoint spawnWp = canidateWps[randIdx];
                Vector3 spawnPos = spawnWp.GetPosition();
                (Waypoint, float) destWpDist = pathFinder.FurthestWaypoint(spawnWp);

                var path = pathFinder.CalculatePath(spawnWp, destWpDist.Item1);

                // Use 1st and 2nd waypoint to calculate spawn rotation
                Quaternion spawnRot = Quaternion.LookRotation((path[1].GetPosition() - path[0].GetPosition()).normalized);

                GameObject newVehObj = Instantiate(spawnList[Random.Range(0, spawnList.Count - 1)], spawnPos, spawnRot);

                AI ai = newVehObj.GetComponent<AI>();
                ai.FollowPath(path);

                activeAIs.Add(ai);

                canidateWps.RemoveAt(randIdx);
            }
        }
    }

    private void RemoveAIVehicles()
    {
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
