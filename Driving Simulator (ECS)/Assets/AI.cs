using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Adapted from BeamNG.drive's lua/vehicle/ai.lua open source code and with permission
public class AI : MonoBehaviour
{
    class SegmentData {
        public Vector3 startPos;
        public Vector3 endPos;
        public Vector3 segmentVec;
        public float length;
        public float radius;
        public float maxSpeed; // max speed to negotiate segment
        public float speed; // segment start speed
    }

    public DebugDrawer debugDrawer;

    public Vehicle vehicle;

    public Transform origin;
    public Transform target;
    private NavMeshPath path;
    private List<SegmentData> plan = new List<SegmentData>();
    

    private float maxAcc = 3f; // m/s^2

    Vector3 vehPos = Vector3.zero;
    float vehSpeed = 0f; // m/s

    private int currSeg = 0;

    void Start()
    {
        path = new NavMeshPath();
        NavMesh.CalculatePath(origin.position, target.position, NavMesh.AllAreas, path);
        GeneratePlan(path);
    }

    // Calculate route data ahead of time
    void GeneratePlan(NavMeshPath path)
    {
        plan.Clear();

        for (int i = 0; i < path.corners.Length - 1; i++)
        {
            SegmentData segment = new SegmentData();

            Vector3 wpStartPos = path.corners[i];
            Vector3 wpEndPos = path.corners[i + 1];
            Vector3 wpVec = wpEndPos - wpStartPos;
            Vector3 wpLeft = Vector3.Cross(wpVec, Vector3.up).normalized;

            float maxSpeed = 1f;
            float radius = 1f;

            if (i > 0)
            {
                Vector3 wpLastPos = path.corners[i - 1];
                //float curvature = MathLib.GetCurvature(wpVec, nextwpPos - wpPos);
                //turnSpeed = Mathf.Sqrt(maxAcc / curvature);
                //turnSpeed = turnSpeed * Mathf.Sin(Mathf.Min(Mathf.Asin(Mathf.Min(1, n2SpeedSq / turnSpeedSq)) + 2 * curvature * n1.length, pi * 0.5))

                radius = MathLib.GetRadius(wpLastPos, wpStartPos, wpEndPos);
                maxSpeed = Mathf.Sqrt(maxAcc * Mathf.Abs(radius));
                //Debug.Log(string.Format("radius: {0:0.00} m", radius));
                //Debug.DrawLine(nextwpPos, nextwpPos + Vector3.up, Color.green);
            }
            else
            {
                // first segment
                radius = float.MaxValue;
                maxSpeed = float.MaxValue;
            }

            segment.startPos = wpStartPos;
            segment.endPos = wpEndPos;
            segment.segmentVec = wpVec;
            segment.length = wpVec.magnitude;
            segment.radius = radius;
            segment.maxSpeed = maxSpeed;
            segment.speed = maxSpeed; // will calculate new value later

            plan.Add(segment);
        }

        int resets = 0;

        // Calculate actual speeds to travel at waypoints
        for (int i = 0; i < plan.Count - 1; i++)
        {
            SegmentData currSeg = plan[i];
            SegmentData nextSeg = plan[i + 1];

            // vf^2 = vi^2 + 2 * a * d
            float nextSegSpeedSqrAfterSlowing = currSeg.speed * currSeg.speed + 2 * -maxAcc * currSeg.length;

            //Debug.Log("i: " + i + ", " + currSeg.speed + ", " + Mathf.Sqrt(nextSegSpeedSqrAfterSlowing) + ", " + nextSeg.speed);

            // If we can't slow down before next segment speed, we must reduce our initial speed
            // and backtrace to make it so we can slow down
            if (nextSegSpeedSqrAfterSlowing - 0.001f > nextSeg.speed * nextSeg.speed)
            {
                currSeg.speed = Mathf.Sqrt(nextSeg.speed * nextSeg.speed + 2 * maxAcc * currSeg.length);
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
                break;
            }
        }
    }

    Vector3 CalculateTarget(float vehXnormOnSeg)
    {
        Vector3 wpPos0 = path.corners[currSeg];
        Vector3 wpPos1 = path.corners[currSeg + 1];
        
        //Vector3 wp01Vec = wpPos1 - wpPos0;

        float targetLength = Mathf.Max(vehSpeed, 5f);
        float remainder = targetLength;

        Vector3 targetPos = path.corners[path.corners.Length - 1];
        Vector3 prevPos = Vector3.LerpUnclamped(wpPos0, wpPos1, vehXnormOnSeg);

        for (int i = currSeg + 1; i < path.corners.Length; i++)
        {
            Vector3 pos = path.corners[i];
            Vector3 segVec = pos - prevPos;
            float segLen = segVec.magnitude;

            if (remainder <= segLen)
            {
                targetPos = segVec * (remainder / (segLen + 1e-30f)) + prevPos;
                break;
            }

            prevPos = pos;
            remainder -= segLen;
        }

        return targetPos;
    }

    void FixedUpdate()
    {
        if (currSeg >= plan.Count - 1) return;
        
        vehPos = vehicle.GetPosition();
        vehSpeed = vehicle.GetSpeed();

        // Contains precalculated information
        SegmentData segment = plan[currSeg];
        SegmentData nextSegment = plan[currSeg + 1];

        float vehXnormOnSeg = MathLib.InverseLerp(vehPos, segment.startPos, segment.endPos);
        Vector3 targetPos = CalculateTarget(vehXnormOnSeg);
        Vector3 vehToTarget = targetPos - vehPos;
       
        float targetSpeed = Mathf.Lerp(segment.speed, nextSegment.speed, vehXnormOnSeg);

        float speedDiff = targetSpeed - vehSpeed;
        //Debug.Log(string.Format("speedDiff: {0:0.00} m/s", speedDiff));

        float angleToTarget = Mathf.Asin(Vector3.Dot(vehicle.transform.right, vehToTarget.normalized));

        vehicle.Steering = angleToTarget;
        vehicle.Brake = Mathf.Min(-speedDiff, 0.5f);
        vehicle.Throttle = Mathf.Min(speedDiff, 0.5f);

        if (vehXnormOnSeg > 1.0f)
        {
            currSeg = Mathf.Min(++currSeg, path.corners.Length - 1);
        }

        //Debug.Log("currentCorner: " + currentCorner);

        //Gizmos.DrawSphere(lastwpPos, 1.0f);
        //Gizmos.DrawSphere(wpPos, 1.0f);
        Debug.DrawLine(targetPos, targetPos + Vector3.up, Color.cyan);
        //Debug.DrawLine(lastwpPos, lastwpPos + wpLeft);
    }

    void LateUpdate()
    {
        /*
        for (int i = 0; i < path.corners.Length - 1; i++)
        {
            Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.red);
            Debug.DrawLine(path.corners[i], path.corners[i] + Vector3.up, Color.red);
        }
        */

        //Debug.DrawLine(lastwpPos, lastwpPos + Vector3.up, Color.green);
        //Debug.DrawLine(wpPos, wpPos + Vector3.up, Color.green);

        for (int i = 0; i < plan.Count; i++)
        {
            SegmentData planData = plan[i];

            //debugDrawer.Draw3DText(planData.startPos, string.Format("{0:0.00}", planData.speed));
            debugDrawer.Draw3DText(planData.startPos, string.Format("{0:0.00} km/h", planData.speed * 3.6f));
        }
    }

}
