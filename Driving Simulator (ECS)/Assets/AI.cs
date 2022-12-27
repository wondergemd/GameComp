using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

// Adapted from BeamNG.drive's lua/vehicle/ai.lua open source code and with permission
public class AI : MonoBehaviour
{
    public Vehicle vehicle;

    public Transform origin;
    public Transform target;
    private NavMeshPath path;

    private float maxLateralAcc = 3f;

    Vector3 vehPos = Vector3.zero;
    float vehSpeed = 0f;

    private int currentCorner = 1;

    void Start()
    {
        path = new NavMeshPath();
        NavMesh.CalculatePath(origin.position, target.position, NavMesh.AllAreas, path);
    }

    Vector3 CalculateTarget(float vehXnormOnSeg)
    {
        int prevCor = currentCorner - 1;

        Vector3 wpPos0 = path.corners[prevCor];
        Vector3 wpPos1 = path.corners[prevCor + 1];
        
        //Vector3 wp01Vec = wpPos1 - wpPos0;

        float targetLength = Mathf.Max(vehSpeed, 5f);
        float remainder = targetLength;

        Vector3 targetPos = path.corners[path.corners.Length - 1];
        Vector3 prevPos = Vector3.LerpUnclamped(wpPos0, wpPos1, vehXnormOnSeg);

        for (int i = prevCor + 1; i < path.corners.Length; i++)
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
        vehPos = vehicle.transform.position;
        vehSpeed = vehicle.GetSpeed();

        Vector3 lastwpPos = path.corners[currentCorner - 1];
        Vector3 wpPos = path.corners[currentCorner];
        Vector3 wpVec = wpPos - lastwpPos;
        Vector3 wpLeft = Vector3.Cross(wpVec, Vector3.up).normalized;

        float vehXnormOnSeg = InverseLerp(vehPos, lastwpPos, wpPos);

        Vector3 targetPos = CalculateTarget(vehXnormOnSeg);

        Vector3 vehToTarget = targetPos - vehPos;

        float turnSpeed = 10f;

        if (currentCorner + 1 < path.corners.Length)
        {
            Vector3 nextwpPos = path.corners[currentCorner + 1];
            float curvature = GetCurvature(wpVec, nextwpPos - wpPos);
            turnSpeed = Mathf.Sqrt(maxLateralAcc / curvature);
        }

        Debug.Log("turnspeed: " + turnSpeed);

        float speedDiff = turnSpeed - vehSpeed;

        float angleToTarget = Mathf.Asin(Vector3.Dot(vehicle.transform.right, vehToTarget.normalized));

        vehicle.Steering = angleToTarget;
        vehicle.Brake = Mathf.Min(-speedDiff, 0.5f);
        vehicle.Throttle = Mathf.Min(speedDiff, 0.5f);

        if (vehXnormOnSeg > 1.0f)
        {
            currentCorner = Mathf.Min(++currentCorner, path.corners.Length - 1);
        }

        Debug.Log("currentCorner: " + currentCorner);

        //Gizmos.DrawSphere(lastwpPos, 1.0f);
        //Gizmos.DrawSphere(wpPos, 1.0f);
        Debug.DrawLine(targetPos, targetPos + Vector3.up, Color.cyan);
        Debug.DrawLine(lastwpPos, lastwpPos + wpLeft);


        for (int i = 0; i < path.corners.Length - 1; i++)
        {
            Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.red);
            Debug.DrawLine(path.corners[i], path.corners[i] + Vector3.up, Color.red);
        }
    }

    float InverseLerp(Vector3 pos, Vector3 a, Vector3 b)
    {
        float bax = b.x - a.x, bay = b.y - a.y, baz = b.z - a.z;
        return (bax * (pos.x - a.x) + bay * (pos.y - a.y) + baz * (pos.z - a.z)) / (bax * bax + bay * bay + baz * baz + 1e-30f);
    }

    float GetCurvature(Vector3 vec1, Vector3 vec2)
    {
        float vec1Sqlen = vec1.sqrMagnitude, vec2Sqlen = vec2.sqrMagnitude;
        float dot12 = Vector3.Dot(vec1, vec2);
        float cos8sq = Mathf.Min(1, dot12 * dot12 / Mathf.Max(1e-30f, vec1Sqlen * vec2Sqlen));

        if (dot12 < 0)
        {
            float minDsq = Mathf.Min(vec1Sqlen, vec2Sqlen);
            float maxDsq = minDsq / Mathf.Max(1e-30f, cos8sq);
            if (Mathf.Max(vec1Sqlen, vec2Sqlen) > (minDsq + maxDsq) * 0.5f)
            {
                if (vec1Sqlen > vec2Sqlen)
                {
                    Vector3 tempVec = vec1;
                    vec1 = vec2;
                    vec2 = tempVec;

                    float tempLen = vec1Sqlen;
                    vec1Sqlen = vec2Sqlen;
                    vec2Sqlen = tempLen;
                }
                vec2 *= Mathf.Sqrt(0.5f * (minDsq + maxDsq) / Mathf.Max(1e-30f, vec2Sqlen));
            }
        }

        vec2 *= -1f;
        return 2 * Mathf.Sqrt((1 - cos8sq) / Mathf.Max(1e-30f, (vec1 - vec2).sqrMagnitude));
    }
}
