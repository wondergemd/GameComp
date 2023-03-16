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

        _dists = new float[allWaypoints.Length];
        for (int i = 0; i < allWaypoints.Length; i++)
        {
            _dists[i] = float.MaxValue;
        }

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

    public bool IsWaypointOnPath(Waypoint wp, List<Waypoint> path)
    {
        foreach (Waypoint pathWp in path)
        {
            if (wp == pathWp)
            {
                return true;
            }
        }

        return false;
    }

    private int _GreatestNumWPsPath(Waypoint startWp, HashSet<Waypoint> wps)
    {
        wps.Add(startWp);

        int longest = 1;
        foreach (Waypoint wp in startWp.neighbors)
        {
            if (!wps.Contains(wp))
            {
                longest = Mathf.Max(_GreatestNumWPsPath(wp, wps) + 1, longest);
            }
        }
        wps.Remove(startWp);

        return longest;
    }

    private int GreatestNumWPsPath(Waypoint startWp)
    {
        return _GreatestNumWPsPath(startWp, new HashSet<Waypoint>());
    }

    public float DistanceToVehicleOnPathTrajectory(Vehicle aVeh, Vehicle bVeh, Waypoint aWp, Waypoint bWp)
    {
        Vector3 a = aVeh.GetPosition();
        Vector3 b = bVeh.GetPosition();

        if (aWp == bWp) return (a - b).magnitude;

        List<Waypoint> path = CalculatePath(aWp, bWp);
        if (path == null) return float.MaxValue;

        float dist = 0;

        for (int i = 0; i < path.Count; i++)
        {
            Waypoint currWp = path[i];
            Waypoint lastWp = null;
            Waypoint nextWp = null;
            if (i > 0) lastWp = path[i - 1];
            if (i < path.Count - 1) nextWp = path[i + 1];

            if (lastWp != null)
            {
                // If we are at a junction, check if the calculated path to vehicle
                // is in line with our trajectory.
                // Basically we assume we go "straight" over junctions.
                // If the path takes a left or right turn, then we assume our vehicle
                // won't travel in that direction and just return float.MaxValue.
                if (currWp.neighbors.Count > 1 && nextWp != null)
                {
                    Vector3 desiredDir = (currWp.GetPosition() - lastWp.GetPosition()).normalized;

                    Waypoint desiredNextWp = null;
                    float maxDot = 0f;

                    foreach (Waypoint neighWp in currWp.neighbors)
                    {
                        Vector3 neighDir = (neighWp.GetPosition() - currWp.GetPosition()).normalized;
                        float dotProd = Vector3.Dot(desiredDir, neighDir);

                        if (dotProd > maxDot)
                        {
                            desiredNextWp = neighWp;
                            maxDot = dotProd;
                        }
                    }

                    if (desiredNextWp != nextWp)
                    {
                        return float.MaxValue;
                    }
                }
                
                dist += (currWp.GetPosition() - lastWp.GetPosition()).magnitude;
            }
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
        var wps = ClosestWaypoints(pos, 10);

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

    float[] _dists;

    public Waypoint[] ClosestWaypoints(Vector3 pos, int? numClosest)
    {
        int numWps = numClosest ?? allWaypoints.Length;

        for (int i = 0; i < numWps; i++)
        {
            _dists[i] = float.MaxValue;
        }

        Waypoint[] wps = new Waypoint[numWps];

        foreach (Waypoint wp in allWaypoints)
        {
            float dist = (wp.GetPosition() - pos).sqrMagnitude;

            if (dist < _dists[numWps - 1])
            {
                for (int i1 = 0; i1 < numWps; i1++)
                {
                    if (dist < _dists[i1])
                    {
                        for (int i2 = numWps - 1; i2 >= i1 + 1; i2--)
                        {
                            wps[i2] = wps[i2 - 1];
                            _dists[i2] = _dists[i2 - 1];
                        }
                        wps[i1] = wp;
                        _dists[i1] = dist;
                        break;
                    }
                }
            }
        }

        return wps;
    }

    public (Waypoint, float) FurthestWaypoint(Waypoint startWp)
    {
        (Waypoint, float) res = _FurthestWaypoint(startWp, new HashSet<Waypoint>());
        return res;
    }

    private (Waypoint, float) _FurthestWaypoint(Waypoint startWp, HashSet<Waypoint> wps)
    {
        wps.Add(startWp);
        Waypoint furthestWp = startWp;
        float furthestDist = 0;
        foreach (Waypoint wp in startWp.neighbors)
        {
            if (!wps.Contains(wp))
            {
                (Waypoint, float) res = _FurthestWaypoint(wp, wps);
                float currDist = res.Item2 + Vector3.Distance(wp.GetPosition(), startWp.GetPosition());

                if (currDist > furthestDist)
                {
                    furthestDist = currDist;
                    furthestWp = res.Item1;
                }
            }
        }

        wps.Remove(startWp);

        return (furthestWp, furthestDist);
    }

    public List<Waypoint> CalculatePath(Vector3 startPos, Waypoint end)
    {
        var wps = ClosestWaypoints(startPos, 10);
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

    PriorityQueue<Waypoint, float> openSet = new PriorityQueue<Waypoint, float>();
    Dictionary<Waypoint, Waypoint> cameFrom = new Dictionary<Waypoint, Waypoint>();
    Dictionary<Waypoint, float> gScore = new Dictionary<Waypoint, float>();

    // A* pathfinding algorithm (graph) https://en.wikipedia.org/wiki/A*_search_algorithm
    public List<Waypoint> CalculatePath(Waypoint start, Waypoint end)
    {
        openSet.Clear();
        cameFrom.Clear();
        gScore.Clear();

        float startFAndHScore = HeuristicFunction(start, end);

        openSet.Enqueue(start, startFAndHScore);
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

    private List<Waypoint> ReconstructPath(Dictionary<Waypoint, Waypoint> cameFrom, Waypoint lastWp)
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
