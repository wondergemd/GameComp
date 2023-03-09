using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Text.RegularExpressions;
using UnityEngine;
using Utils;

public class PathFinder : MonoBehaviour
{
    public DebugDrawer debugDrawer;
    public Waypoint[] allWaypoints;

    // Waypoints that cars should spawn at
    public List<Waypoint> spawnWaypoints = new List<Waypoint>();

    void Awake()
    {
        // First update list of waypoints avaliable
        allWaypoints = GameObject.FindObjectsOfType<Waypoint>();

        // Calculate waypoint neighbours on one line by their names
        // gameobject name suffixed with "(0)" indicates start of a waypoint group

        // First just go through all waypoints and add them to a dictionary,
        // where their name's prefix is the key
        // and the index in parenthesis is added as the index in list of waypoints

        var wpsDict = new Dictionary<string, SortedDictionary<double, Waypoint>>();

        string pattern = @"^(.*)\((\d*\.?\d+)\)$";

        foreach (Waypoint waypoint in allWaypoints)
        {
            Match match = Regex.Match(waypoint.name, pattern);

            string prefix = match.Groups[1].Value;
            string strIdx = match.Groups[2].Value;

            if (!wpsDict.ContainsKey(prefix)) wpsDict.Add(prefix, new SortedDictionary<double, Waypoint>());

            double idx;
            if (!double.TryParse(strIdx, out idx)) continue;
            wpsDict[prefix].Add(idx, waypoint);
        }

        // If index zero is not null of a set, then calculate neighbours for that set
        foreach (var kv in wpsDict)
        {
            var wps = kv.Value;
            if (wps.ContainsKey(0))
            {
                Waypoint lastWp = null;

                foreach (var kv2 in wps)
                {
                    Waypoint wp = kv2.Value;

                    if (lastWp != null)
                    {
                        lastWp.neighbors.Add(wp);
                    }

                    lastWp = wp;
                }
            }
        }

        // Spawn waypoints must have longest path of 3 including self
        foreach (Waypoint wp in allWaypoints)
        {
            if (GreatestNumWPsPath(wp) >= 3)
            {
                spawnWaypoints.Add(wp);
            }

            // Add previous neighbors to waypoints
            foreach (Waypoint wpAdj in wp.neighbors)
            {
                wpAdj.prevNeighbors.Add(wp);
            }
        }
    }

    private void LateUpdate()
    {
        if (debugDrawer != null)
        {
            foreach (Waypoint wp in allWaypoints)
            {
                debugDrawer.Draw3DText(wp.GetPosition() + Vector3.up * 2, wp.gameObject.name);
            }
        }
    }

    private int GreatestNumWPsPath(Waypoint startWp)
    {
        int longest = 1;
        foreach (Waypoint wp in startWp.neighbors)
        {
            longest = Mathf.Max(GreatestNumWPsPath(wp) + 1, longest);
        }
        return longest;
    }

    // Waypoints should be close to respective points
    public float DistanceBetweenTwoPointsOnPath(Vector3 a, Vector3 b, Waypoint aWp, Waypoint bWp)
    {
        if (aWp == bWp) return (a - b).magnitude;

        List<Waypoint> path = CalculatePath(aWp, bWp);
        if (path == null) return float.MaxValue;

        float dist = 0;

        Waypoint lastWp = null;

        foreach (Waypoint wp in path)
        {
            if (lastWp != null)
            {
                dist += (wp.GetPosition() - lastWp.GetPosition()).magnitude;
            }

            lastWp = wp;
        }

        Vector3 a1WpPos = path[0].GetPosition();
        Vector3 a2WpPos = path[1].GetPosition();

        Vector3 b1WpPos = path[path.Count - 2].GetPosition();
        Vector3 b2WpPos = path[path.Count - 1].GetPosition();

        float aXnorm = -MathLib.InverseLerp(a, a1WpPos, a2WpPos);
        float bXnorm = -MathLib.InverseLerp(b, b2WpPos, b1WpPos);

        float aDist = (a2WpPos - a1WpPos).magnitude * aXnorm;
        float bDist = (b2WpPos - b1WpPos).magnitude * bXnorm;

        return dist + aDist + bDist;
    }

    public float DistanceBetweenWaypoints(Waypoint a, Waypoint b)
    {
        List<Waypoint> path = CalculatePath(a, b);
        if (path == null) return float.MaxValue;

        float dist = 0;

        Waypoint lastWp = null;

        foreach (Waypoint wp in path)
        {
            if (lastWp != null)
            {
                dist += (wp.GetPosition() - lastWp.GetPosition()).magnitude;
            }

            lastWp = wp;
        }

        return dist;
    }

    public (Waypoint, Waypoint, float) GetSegmentVehicleOn(Vector3 pos)
    {
        var wps = ClosestWaypoints(pos);

        foreach (Waypoint wp in wps)
        {
            Vector3 wpPos = wp.GetPosition();

            foreach (Waypoint nextWp in wp.neighbors)
            {
                Vector3 nextWpPos = nextWp.GetPosition();
                float xnorm = MathLib.InverseLerp(pos, wpPos, nextWpPos);

                if (xnorm >= 0 && xnorm <= 1)
                {
                    Vector3 perpVec = 0.5f * wp.width * Vector3.Cross(nextWpPos - wpPos, Vector3.up).normalized;
                    
                    float xnorm2 = MathLib.InverseLerp(pos, wpPos, wpPos + perpVec);

                    if (xnorm2 >= -1 && xnorm2 <= 1)
                    {
                        return (wp, nextWp, xnorm);
                    }
                }
            }

            foreach (Waypoint prevWp in wp.prevNeighbors)
            {
                Vector3 prevWpPos = prevWp.GetPosition();
                float xnorm = MathLib.InverseLerp(pos, prevWpPos, wpPos);

                if (xnorm >= 0 && xnorm <= 1)
                {
                    Vector3 perpVec = 0.5f * wp.width * Vector3.Cross(wpPos - prevWpPos, Vector3.up).normalized;

                    float xnorm2 = MathLib.InverseLerp(pos, wpPos, wpPos + perpVec);

                    if (xnorm2 >= -1 && xnorm2 <= 1)
                    {
                        return (prevWp, wp, xnorm);
                    }
                }
            }
        }
        return (null, null, 0.0f);
    }

    public Waypoint RandomWaypoint()
    {
        return allWaypoints[Random.Range(0, allWaypoints.Length - 1)];
    }

    public Waypoint RandomSpawnWaypoint()
    {
        return spawnWaypoints[Random.Range(0, spawnWaypoints.Count - 1)];
    }

    public Waypoint ClosestWaypoint(Vector3 pos)
    {
        float closestDist = float.MaxValue;
        Waypoint closestWp = null;

        foreach (Waypoint wp in allWaypoints)
        {
            Vector3 wpPos = wp.transform.position;

            float dist = (wpPos - pos).sqrMagnitude;
            if (dist < closestDist)
            {
                closestDist = dist;
                closestWp = wp;
            }
        }

        return closestWp;
    }

    public Waypoint[] ClosestWaypoints(Vector3 pos)
    {
        int numWps = allWaypoints.Length;

        float[] dists = new float[numWps];
        for (int i = 0; i < numWps; i++)
        {
            dists[i] = float.MaxValue;
        }

        Waypoint[] wps = new Waypoint[numWps];

        foreach (Waypoint wp in allWaypoints)
        {
            float dist = (wp.GetPosition() - pos).sqrMagnitude;

            if (dist < dists[numWps - 1])
            {
                for (int i1 = 0; i1 < numWps; i1++)
                {
                    if (dist < dists[i1])
                    {
                        for (int i2 = numWps - 1; i2 >= i1 + 1; i2--)
                        {
                            wps[i2] = wps[i2 - 1];
                            dists[i2] = dists[i2 - 1];
                        }
                        wps[i1] = wp;
                        dists[i1] = dist;
                        break;
                    }
                }
            }
        }

        return wps;
    }

    public Waypoint FurthestWaypoint(Waypoint startWp)
    {
        float dist = 0;
        Waypoint wp = _FurthestWaypoint(startWp, ref dist);
        return wp;
    }

    private Waypoint _FurthestWaypoint(Waypoint startWp, ref float dist)
    {
        Waypoint furthestWp = startWp;
        float furthestDist = -1;
        foreach (Waypoint wp in startWp.neighbors)
        {
            float newDist = dist + (wp.GetPosition() - startWp.GetPosition()).sqrMagnitude;
            Waypoint canidateWp = _FurthestWaypoint(wp, ref newDist);

            if (newDist > furthestDist)
            {
                furthestDist = newDist;
                furthestWp = canidateWp;
            }
        }

        return furthestWp;
    }

    public List<Waypoint> CalculatePath(Vector3 startPos, Waypoint end)
    {
        var wps = ClosestWaypoints(startPos);
        foreach (Waypoint wp in wps)
        {
            List<Waypoint> path = CalculatePath(wp, end);

            if (path != null)
            {
                return path;
            }
        }

        return null;
    }

    // A* pathfinding algorithm (graph) https://en.wikipedia.org/wiki/A*_search_algorithm
    public List<Waypoint> CalculatePath(Waypoint start, Waypoint end)
    {
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
}
