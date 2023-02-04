using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils;

public class PathFinder : MonoBehaviour
{
    Waypoint[] allWaypoints;

    // Start is called before the first frame update
    void Start()
    {
        allWaypoints = FindObjectsOfType<Waypoint>();
    }

    private Waypoint ClosestWaypoint(Vector3 pos)
    {
        float closestDist = float.MaxValue;
        Waypoint closestWp = null;

        foreach (Waypoint wp in allWaypoints)
        {
            Vector3 wpPos = wp.transform.position;

            float dist = (wpPos - pos).sqrMagnitude;
            if (dist < closestDist) {
                closestDist = dist;
                closestWp = wp;
            }
        }

        return closestWp;
    }

    private float HeuristicFunction(Waypoint currentWp, Waypoint endWp)
    {
        return (currentWp.GetPosition() - endWp.GetPosition()).sqrMagnitude;
    }

    private List<Waypoint> ReconstructPath(
        Dictionary<Waypoint, Waypoint> cameFrom, Waypoint lastWp)
    {
        var path = new List<Waypoint>();
        path.Add(lastWp);

        Waypoint prevWp;
        cameFrom.TryGetValue(lastWp, out prevWp);

        while (prevWp != null)
        {
            path.Insert(0, prevWp);
            cameFrom.TryGetValue(prevWp, out prevWp);
        }

        return path;
    }

    // A* pathfinding algorithm (graph) https://en.wikipedia.org/wiki/A*_search_algorithm
    public List<Waypoint> CalculatePath(Vector3 startPos, Vector3 endPos)
    {
        // First find waypoints closest to start and end
        Waypoint start = ClosestWaypoint(startPos);
        Waypoint end = ClosestWaypoint(endPos);

        float startFAndHScore = HeuristicFunction(start, end);

        var openSet = new PriorityQueue<Waypoint, float>();
        openSet.Enqueue(start, startFAndHScore);

        var cameFrom = new Dictionary<Waypoint, Waypoint>();

        var gScore = new Dictionary<Waypoint, float>();
        gScore[start] = 0.0f;

        while (openSet.Count > 0)
        {
            Waypoint current = openSet.Dequeue();

            if (current == end)
            {
                return ReconstructPath(cameFrom, current);
            }

            foreach (Waypoint neighbor in current.neighbors)
            {
                float tentativeGScore = gScore[current] + (current.GetPosition() - neighbor.GetPosition()).sqrMagnitude;
                float score;
                if (!gScore.TryGetValue(neighbor, out score))
                {
                    score = float.MaxValue;
                }

                if (tentativeGScore < score)
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;

                    bool exists = false;
                    foreach (var item in openSet.UnorderedItems)
                    {
                        if (neighbor.Equals(item))
                        {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists)
                    {
                        openSet.Enqueue(neighbor, tentativeGScore + HeuristicFunction(neighbor, end));
                    }
                }
            }
        }

        return null;
    }

}
