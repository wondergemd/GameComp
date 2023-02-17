using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Utils;

// Adapted from BeamNG.drive's lua/vehicle/ai.lua open source code and with permission
public class AI : MonoBehaviour
{
    class SegmentData {
        public MathLib.CatmullRomCurve catmullCurve;
        public Vector3 startPos;
        public Vector3 endPos;
        public Vector3 segmentVec;
        public float length;
        public float startWidth;
        public float endWidth;
        public float startRadius;
        public float endRadius;
        public float startMaxSpeed; // max speed to negotiate segment start
        public float endMaxSpeed; // max speed to negotiate segment end
        public float startSpeed; // segment start speed
        public float endSpeed; // segment start speed
    }

    public DebugDrawer debugDrawer; // can be null

    private Vehicle vehicle;
    private List<Waypoint> path = new List<Waypoint>();
    private List<SegmentData> plan = new List<SegmentData>();

    private float maxAcc = 3f; // m/s^2

    Vector3 vehPos = Vector3.zero;
    float vehSpeed = 0f; // m/s

    private int currSegIdx = 0;

    // Variables used for sensors
    public float sensorLength = 8f;
    public Vector3 frontSensorPos = new Vector3(2.25f, .5f, 0f);
    public float frontSensorAngle = 30;
    public float sensorSidePositions = 0.75f;
    public bool avoiding = false;
    public float avoidMult = 0f;
    public float turnSpeed = 1f;
    private float avoidTargetSteerAngle = 0f;

    void Awake()
    {
        vehicle = GetComponent<Vehicle>();
        vehPos = vehicle.GetPosition();
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
        this.path = path;

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

            Vector3 p0 = (i - 1 >= 0 ? path[i - 1] : path[0]).GetPosition();
            Vector3 p3 = (i + 2 < path.Count ? path[i + 2] : path[path.Count - 1]).GetPosition();
            Vector3 wpStartPos = path[i].GetPosition();
            Vector3 wpEndPos = path[i + 1].GetPosition();
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

            segment.startPos = wpStartPos;
            segment.endPos = wpEndPos;
            segment.segmentVec = wpVec;
            segment.length = wpVec.magnitude;
            //segment.startWidth = path[i].width;
            //segment.endWidth = path[i + 1].width;
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

    // Collision Avoidance/ADAS
    private void Sensors()
    {
        RaycastHit hit;
        // Sensor starts at middle of car, additions set sensors at front of car and account for changes in direction
        Vector3 sensorPos = transform.position;
        sensorPos += (transform.forward * frontSensorPos.x);
        sensorPos += (transform.up * frontSensorPos.y);

        // if object is detected, avoidMult determines steering direction
        avoiding = false;
        avoidMult = 0;


        // front left sensors
        sensorPos -= transform.right * sensorSidePositions;
        if (Physics.Raycast(sensorPos, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 1.0f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward), out hit, sensorLength * 0.75f))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 0.75f;
        }
        else if (Physics.Raycast(sensorPos, Quaternion.AngleAxis(-45, transform.up) * transform.forward, out hit, sensorLength * 0.5f))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult += 0.5f;
        }
        else
        {
            Debug.DrawRay(sensorPos, transform.forward * sensorLength, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(-frontSensorAngle, transform.up) * transform.forward) * sensorLength * 0.75f, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(-45, transform.up) * transform.forward) * sensorLength * 0.5f, Color.red);
        }


        // front right
        sensorPos += (transform.right * sensorSidePositions) * 2;
        if (Physics.Raycast(sensorPos, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 1.0f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * 0.75f, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 0.75f;
        }
        else if (Physics.Raycast(sensorPos, (Quaternion.AngleAxis(45, transform.up) * transform.forward) * 0.5f, out hit, sensorLength / 2))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;
            avoidMult -= 0.5f;
        }
        else
        {
            Debug.DrawRay(sensorPos, transform.forward * sensorLength, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(frontSensorAngle, transform.up) * transform.forward) * sensorLength * 0.75f, Color.red);
            Debug.DrawRay(sensorPos, (Quaternion.AngleAxis(45, transform.up) * transform.forward) * sensorLength * 0.5f, Color.red);
        }


        // front center
        sensorPos -= transform.right * sensorSidePositions;
        if (Physics.Raycast(sensorPos, transform.forward, out hit, sensorLength))
        {
            Debug.DrawLine(sensorPos, hit.point, Color.green);
            avoiding = true;

            // if object is detected at front sensor, check the normal of the raycast to determine direction to turn
            if (hit.normal.x < 0)
            {
                avoidMult = +1;
            }
            else if (hit.normal.x > 0)
            {
                avoidMult = -1;
            }
        }
        else
        {
            Debug.DrawRay(sensorPos, transform.forward * sensorLength, Color.red);
        }
    }

    private void AvoidObject()
    {
        if (avoiding)
        {
            Debug.Log("AvoidMult = " + avoidMult);
            if (avoidMult == 0)
            {

            }
            avoidTargetSteerAngle = vehicle.maxSteeringAngle * avoidMult;
            // interpolate target steering angle with the current steering angle over time
            vehicle.Steering = Mathf.Lerp(vehicle.Steering, avoidTargetSteerAngle, Time.deltaTime * turnSpeed);
        }
    }

    void FixedUpdate()
    {
        if (currSegIdx == plan.Count) return;
        
        vehPos = vehicle.GetPosition();
        vehSpeed = vehicle.GetSpeed();

        // Contains precalculated information
        SegmentData segment = plan[currSegIdx];

        float vehXnormOnSeg = MathLib.InverseLerp(vehPos, segment.startPos, segment.endPos);
        Vector3 targetPos = CalculateTarget(segment, vehXnormOnSeg);
        Vector3 vehToTarget = targetPos - vehPos;
       
        float targetSpeed = Mathf.Lerp(segment.startSpeed, segment.endSpeed, vehXnormOnSeg);

        float speedDiff = targetSpeed - vehSpeed;
        //Debug.Log(string.Format("speedDiff: {0:0.00} m/s", speedDiff));
        //Debug.Log(targetSpeed);

        float angleToTarget = Mathf.Asin(Vector3.Dot(vehicle.transform.right, vehToTarget.normalized));

        vehicle.Steering = angleToTarget;
        vehicle.Brake = Mathf.Min(-speedDiff, 0.5f);
        vehicle.Throttle = Mathf.Min(speedDiff, 0.5f);

        //Debug.Log("currentCorner: " + currentCorner);

        //Gizmos.DrawSphere(lastwpPos, 1.0f);
        //Gizmos.DrawSphere(wpPos, 1.0f);
        Debug.DrawLine(targetPos, targetPos + Vector3.up, Color.cyan);
        //Debug.DrawLine(lastwpPos, lastwpPos + wpLeft);

        if (vehXnormOnSeg > 1.0f)
        {
            currSegIdx = Mathf.Min(++currSegIdx, plan.Count);
        }

        // Detect object & override AI pathing if detected
        Sensors();
        AvoidObject();

    }

    void LateUpdate()
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
        }
    }

    public bool HasReachedDestination()
    {
        return currSegIdx == plan.Count;
    }
}
