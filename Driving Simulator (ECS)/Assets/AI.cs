using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Utils;

// Adapted from BeamNG.drive's lua/vehicle/ai.lua open source code and with permission
public class AI : MonoBehaviour
{
    public class SegmentData {
        public MathLib.CatmullRomCurve catmullCurve;
        public Waypoint startWp;
        public Waypoint endWp;
        public Vector3 startPos;
        public Vector3 endPos;
        public Vector3 segmentVec;
        public float length;
        public float startRadius;
        public float endRadius;
        public float startMaxSpeed; // max speed to negotiate segment start
        public float endMaxSpeed; // max speed to negotiate segment end
        public float startSpeed; // segment start speed
        public float endSpeed; // segment start speed
    }

    public DebugDrawer debugDrawer; // can be null
    public PathFinder pathFinder;
    public TrafficGenerator trafficGenerator;

    public Vehicle vehicle;
    public List<SegmentData> plan = new List<SegmentData>();
    public Vehicle playerVehicle;

    // m/s^2
    public float minAcc = 2f;
    public float maxAcc = 3f; 

    Vector3 vehPos = Vector3.zero;
    float vehSpeed = 0f; // m/s

    public int currSegIdx = 0;
    public float currSegXnorm = 0f;

    float tempdistBetween = 0f;

    public bool debugPaths = false;

    void Awake()
    {
        vehicle = GetComponent<Vehicle>();
        vehPos = vehicle.GetPosition();

        debugDrawer = FindObjectOfType<DebugDrawer>();
        pathFinder = FindObjectOfType<PathFinder>();
        trafficGenerator = FindObjectOfType<TrafficGenerator>();
        playerVehicle = FindObjectOfType<Player>().GetComponent<Vehicle>();
    }

    /*
    public void GoToTarget(Waypoint targetWP)
    {
        path = PathFinder.CalculatePath(vehPos, targetWP);

        if (path != null)
        {
            if (!GeneratePlan(path))
            {
                Debug.LogError("Plan unsuccessfully generated!!!");
            }
        }
        else
        {
            Debug.LogError("Path unsuccessfully generated!!!");
        }
    }
    */

    public void FollowPath(List<Waypoint> path)
    {
        if (!GeneratePlan(path))
        {
            Debug.LogError("Plan unsuccessfully generated!!!");
        }
    }

    // Calculate route data ahead of time
    bool GeneratePlan(List<Waypoint> path)
    {
        plan.Clear();

        if (path.Count < 3)
        {
            Debug.LogError("Given Path Length < 3! Path length = " + path.Count);
            return false;
        }

        Vector3[] points = new Vector3[path.Count];

        for (int i = 0; i < path.Count - 1; i++)
        {
            SegmentData segment = new SegmentData();

            Waypoint startWp = path[i];
            Waypoint endWp = path[i + 1];
            Vector3 p0 = (i - 1 >= 0 ? path[i - 1] : path[0]).GetPosition();
            Vector3 p3 = (i + 2 < path.Count ? path[i + 2] : path[path.Count - 1]).GetPosition();
            Vector3 wpStartPos = startWp.GetPosition();
            Vector3 wpEndPos = endWp.GetPosition();
            Vector3 wpVec = wpEndPos - wpStartPos;
            Vector3 wpLeft = Vector3.Cross(wpVec, Vector3.up).normalized;

            segment.catmullCurve = new MathLib.CatmullRomCurve(p0, wpStartPos, wpEndPos, p3);

            float startMaxSpeed, startRadius;

            if (i > 0)
            {
                Vector3 wpLastPos = path[i - 1].GetPosition();
                //float curvature = MathLib.GetCurvature(wpVec, nextwpPos - wpPos);
                //turnSpeed = Mathf.Sqrt(maxAcc / curvature);
                //turnSpeed = turnSpeed * Mathf.Sin(Mathf.Min(Mathf.Asin(Mathf.Min(1, n2SpeedSq / turnSpeedSq)) + 2 * curvature * n1.length, pi * 0.5))

                startRadius = segment.catmullCurve.ClosestPointOnCurveMoreInfo(wpStartPos, 10).radius; //MathLib.GetRadius(wpLastPos, wpStartPos, wpEndPos);
                startMaxSpeed = Mathf.Min(100, Mathf.Sqrt(maxAcc * Mathf.Abs(startRadius)));

                SegmentData prevSegment = plan[i - 1];
                prevSegment.endRadius = startRadius;
                prevSegment.endMaxSpeed = startMaxSpeed;
                prevSegment.endSpeed = startMaxSpeed;

                //Debug.Log(string.Format("radius: {0:0.00} m", radius));
                //Debug.DrawLine(nextwpPos, nextwpPos + Vector3.up, Color.green);
            }
            else
            {
                // first segment
                startRadius = float.MaxValue;
                startMaxSpeed = 5;
            }

            segment.startWp = startWp;
            segment.endWp = endWp;
            segment.startPos = wpStartPos;
            segment.endPos = wpEndPos;
            segment.segmentVec = wpVec;
            segment.length = wpVec.magnitude;
            segment.startRadius = startRadius;
            segment.startMaxSpeed = startMaxSpeed;
            segment.startSpeed = startMaxSpeed; // will calculate new value later

            plan.Add(segment);

            // For catmull rom spline
            points[i] = wpStartPos;
        }
        SegmentData lastSeg = plan[plan.Count - 1];
        lastSeg.endRadius = float.MaxValue;

        // 1 m/s to get them to cross over last waypoint and get despawned by traffic generator
        lastSeg.endMaxSpeed = 1f;
        lastSeg.endSpeed = 1f; 

        int resets = 0;

        // Calculate actual speeds to travel at waypoints
        for (int i = 0; i < plan.Count - 1; i++)
        {
            SegmentData currSeg = plan[i];
            SegmentData nextSeg = plan[i + 1];

            // vf^2 = vi^2 + 2 * a * d
            float nextSegSpeedSqrAfterSlowing = currSeg.endSpeed * currSeg.endSpeed + 2 * -maxAcc * nextSeg.length;

            //Debug.Log("i: " + i + ", " + currSeg.speed + ", " + Mathf.Sqrt(nextSegSpeedSqrAfterSlowing) + ", " + nextSeg.speed);

            // If we can't slow down before next segment speed, we must reduce our initial speed
            // and backtrace to make it so we can slow down
            if (nextSegSpeedSqrAfterSlowing - 0.001f > nextSeg.endSpeed * nextSeg.endSpeed)
            {
                currSeg.endSpeed = Mathf.Sqrt(nextSeg.endSpeed * nextSeg.endSpeed + 2 * maxAcc * nextSeg.length);
                nextSeg.startSpeed = currSeg.endSpeed;

                //Debug.Log("new currsegSpeed: " + currSeg.speed);
                if (i > 0)
                {
                    i -= 2;
                    resets++;
                }
            }
            
            if (resets > 100000)
            {
                Debug.LogError("GeneratePlan forever loop condition halted");
                return false;
            }
        }
        return true;
    }

    Vector3 CalculateTarget(SegmentData currSeg, float vehXnormOnSeg)
    {
        float targetLength = Mathf.Max(vehSpeed, 5f);
        float remainder = targetLength;

        Vector3 targetPos = plan[plan.Count - 1].endPos;
        Vector3 prevPos = Vector3.LerpUnclamped(currSeg.startPos, currSeg.endPos, vehXnormOnSeg);

        for (int i = currSegIdx; i < plan.Count; i++)
        {
            SegmentData seg = plan[i];

            Vector3 pos = seg.endPos;
            Vector3 segVec = i == currSegIdx ? pos - prevPos : seg.segmentVec;
            float segLen = segVec.magnitude;

            if (remainder <= segLen)
            {
                targetPos = seg.catmullCurve.ClosestPointOnCurve(segVec * (remainder / (segLen + 1e-30f)) + prevPos, 10);
                break;
            }

            prevPos = pos;
            remainder -= segLen;
        }

        return targetPos;
    }
    
    float CalculateTargetSpeed(SegmentData currSeg, float vehXnormOnSeg)
    {
        // Target speed calculated to traverse path
        float targetSpeed = Mathf.Lerp(currSeg.startSpeed, currSeg.endSpeed, vehXnormOnSeg);

        // Take into account AI vehicles and player vehicle by keeping distance between them
        this.tempdistBetween = float.MaxValue;

        foreach (AI ai in trafficGenerator.activeAIs)
        {
            if (this != ai)
            {
                SegmentData otherCurrSeg = ai.plan[Mathf.Min(ai.currSegIdx, ai.plan.Count - 1)];

                if (currSeg.endWp == otherCurrSeg.endWp)
                {
                    if (this.currSegXnorm > ai.currSegXnorm)
                    {
                        continue;
                    }
                }

                float distBetween = pathFinder.DistanceToVehicleOnPathTrajectory(this.vehicle, ai.vehicle, currSeg.endWp, otherCurrSeg.endWp) - 2f * vehSpeed - 5f;
                this.tempdistBetween = Mathf.Min(distBetween, tempdistBetween);

                // vf^2 = vi^2 + 2 * a * d
                float speedSqrAfterSlowing = vehSpeed * vehSpeed + 2 * -maxAcc * distBetween;

                if (speedSqrAfterSlowing > ai.vehSpeed * ai.vehSpeed)
                {
                    float followAISpeed = Mathf.Sqrt(ai.vehSpeed * ai.vehSpeed + 2 * maxAcc * distBetween);
                    targetSpeed = Mathf.Min(targetSpeed, followAISpeed);
                }
            }
        }

        // Predict trajectory of player vehicle
        Vector3 pPos = playerVehicle.GetPosition();
        float pSpeed = playerVehicle.GetSpeed();

        /*
        float pTimeToStop = pSpeed / maxAcc;
        Vector3 pFuturePos =
            playerVehicle.GetVelocity() * pTimeToStop +
            0.5f * pTimeToStop * pTimeToStop * playerVehicle.GetAccelerationVec() +
            pPos;
        */

        (Waypoint, Waypoint, float) seg = pathFinder.GetSegmentVehicleOn(pPos);
        if (seg.Item1 != null && !(currSeg.endWp == seg.Item2 && this.currSegXnorm > seg.Item3))
        {
            float distBetween = pathFinder.DistanceBetweenTwoPointsOnPath(this.vehPos, pPos, currSeg.endWp, seg.Item2) - 2f * vehSpeed - 5f;
            this.tempdistBetween = Mathf.Min(distBetween, tempdistBetween);

            // vf^2 = vi^2 + 2 * a * d
            float speedSqrAfterSlowing = vehSpeed * vehSpeed + 2 * -maxAcc * distBetween;

            if (speedSqrAfterSlowing > pSpeed * pSpeed)
            {
                float followAISpeed = Mathf.Sqrt(pSpeed * pSpeed + 2 * maxAcc * distBetween);
                targetSpeed = Mathf.Min(targetSpeed, followAISpeed);
            }

            //Debug.DrawLine(seg.Item1.GetPosition(), seg.Item2.GetPosition(), Color.red);
            /*
            List<Waypoint> pPath = pathFinder.CalculatePath(seg.Item1, pathFinder.ClosestWaypoint(pFuturePos));
            if (pPath != null)
            {

            }
            for (int i = 0; i < pPath.Count - 1; i++)
            {
                Debug.DrawLine(pPath[i].GetPosition(), pPath[i + 1].GetPosition(), Color.red);
            }
            */
        }

        return targetSpeed;
    }

    void FixedUpdate()
    {
        if (currSegIdx == plan.Count) return;
        
        vehPos = vehicle.GetPosition();
        vehSpeed = vehicle.GetSpeed();

        // Contains precalculated information
        SegmentData segment = plan[currSegIdx];

        currSegXnorm = MathLib.InverseLerp(vehPos, segment.startPos, segment.endPos);
        Vector3 targetPos = CalculateTarget(segment, currSegXnorm);
        Vector3 vehToTarget = targetPos - vehPos;

        float targetSpeed = CalculateTargetSpeed(segment, currSegXnorm);
        float speedDiff = targetSpeed - vehSpeed;
        //Debug.Log(string.Format("speedDiff: {0:0.00} m/s", speedDiff));
        //Debug.Log(targetSpeed);

        float angleToTarget = Mathf.Asin(Vector3.Dot(vehicle.transform.right, vehToTarget.normalized));

        vehicle.Steering = angleToTarget;
        vehicle.Brake = Mathf.Min(-speedDiff, 1.0f);
        vehicle.Throttle = Mathf.Min(speedDiff, 1.0f);

        //Debug.Log("currentCorner: " + currentCorner);

        //Gizmos.DrawSphere(lastwpPos, 1.0f);
        //Gizmos.DrawSphere(wpPos, 1.0f);
        Debug.DrawLine(targetPos, targetPos + Vector3.up, Color.cyan);
        //Debug.DrawLine(lastwpPos, lastwpPos + wpLeft);

        if (currSegXnorm > 1.0f)
        {
            currSegIdx = Mathf.Min(++currSegIdx, plan.Count);
        }

    }

    void LateUpdate()
    {

        if (debugPaths)
        {
            for (int s = 0; s < plan.Count; s++)
            {
                MathLib.CatmullRomCurve curve = plan[s].catmullCurve;

                const int detail = 32;
                Vector3 prev = curve.p1;
                for (int i = 1; i < detail; i++)
                {
                    float t = i / (detail - 1f);
                    Vector3 pt = curve.GetPoint(t);
                    Debug.DrawLine(prev, pt, Color.HSVToRGB(s / (float)plan.Count, 1.0f, 1.0f));
                    prev = pt;
                }

                Debug.DrawLine(curve.p1, curve.p1 + Vector3.up, Color.red);
                Debug.DrawLine(curve.p2, curve.p2 + Vector3.up, Color.green);
            }
        }

        /*
        if (path != null)
        {
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i].GetPosition(), path[i + 1].GetPosition(), Color.red);
                Debug.DrawLine(path[i].GetPosition(), path[i].GetPosition() + Vector3.up, Color.red);
            }
        }
        */

        //Debug.DrawLine(lastwpPos, lastwpPos + Vector3.up, Color.green);
        //Debug.DrawLine(wpPos, wpPos + Vector3.up, Color.green);

        if (debugDrawer != null)
        {
            for (int i = 0; i < plan.Count; i++)
            {
                SegmentData planData = plan[i];

                debugDrawer.Draw3DText(planData.endPos, string.Format("{0:0.00} km/h", planData.endSpeed * 3.6f));
                //debugDrawer.Draw3DText(planData.startPos, string.Format("Width: {0:0.00} m", planData.startWidth));
            }

            debugDrawer.Draw3DText(vehPos + Vector3.up * 2f, string.Format("dist between {0:0.00} m", this.tempdistBetween));
        }
    }

    public bool HasReachedDestination()
    {
        return currSegIdx == plan.Count;
    }
}
