using UnityEngine;
using UnityEngine.AI;

// Adapted from BeamNG.drive's lua/vehicle/ai.lua open source code and with permission
public class AI : MonoBehaviour
{
    public Vehicle vehicle;

    public Transform origin;
    public Transform target;
    private NavMeshPath path;

    private float maxAcc = 3f; // m/s^2

    Vector3 vehPos = Vector3.zero;
    float vehSpeed = 0f; // m/s

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
        vehPos = vehicle.GetPosition();
        vehSpeed = vehicle.GetSpeed();

        Vector3 lastwpPos = path.corners[currentCorner - 1];
        Vector3 wpPos = path.corners[currentCorner];
        Vector3 wpVec = wpPos - lastwpPos;
        Vector3 wpLeft = Vector3.Cross(wpVec, Vector3.up).normalized;

        float vehXnormOnSeg = MathLib.InverseLerp(vehPos, lastwpPos, wpPos);

        Vector3 targetPos = CalculateTarget(vehXnormOnSeg);

        Vector3 vehToTarget = targetPos - vehPos;

        float lastTurnSpeed = 10f;
        float turnSpeed = 10f;

        if (currentCorner - 2 >= 0)
        {
            Vector3 last2wpPos = path.corners[currentCorner - 2];
            float radius = MathLib.GetRadius(last2wpPos, lastwpPos, wpPos);
            lastTurnSpeed = Mathf.Sqrt(maxAcc * Mathf.Abs(radius));
        }

        if (currentCorner + 1 < path.corners.Length)
        {
            Vector3 nextwpPos = path.corners[currentCorner + 1];
            float radius = MathLib.GetRadius(lastwpPos, wpPos, nextwpPos);
            turnSpeed = Mathf.Sqrt(maxAcc * Mathf.Abs(radius));
            //Debug.Log(string.Format("radius: {0:0.00} m", radius));
            Debug.DrawLine(nextwpPos, nextwpPos + Vector3.up, Color.green);
        }

        Debug.Log(string.Format("turnspeed: {0:0.00} m/s", turnSpeed));

        //float acc = (turnSpeed * turnSpeed - vehSpeed * vehSpeed) / (2 * (vehPos - wpPos).magnitude);
        float targetSpeed = Mathf.Lerp(lastTurnSpeed, turnSpeed, vehXnormOnSeg);

        Debug.Log(string.Format("targetSpeed: {0:0.00} m/s", targetSpeed));
        
        float speedDiff = targetSpeed - vehSpeed;
        //Debug.Log(string.Format("speedDiff: {0:0.00} m/s", speedDiff));

        float angleToTarget = Mathf.Asin(Vector3.Dot(vehicle.transform.right, vehToTarget.normalized));

        vehicle.Steering = angleToTarget;
        vehicle.Brake = Mathf.Min(-speedDiff, 0.5f);
        vehicle.Throttle = Mathf.Min(speedDiff, 0.5f);

        if (vehXnormOnSeg > 1.0f)
        {
            currentCorner = Mathf.Min(++currentCorner, path.corners.Length - 1);
        }

        //Debug.Log("currentCorner: " + currentCorner);

        //Gizmos.DrawSphere(lastwpPos, 1.0f);
        //Gizmos.DrawSphere(wpPos, 1.0f);
        Debug.DrawLine(targetPos, targetPos + Vector3.up, Color.cyan);
        Debug.DrawLine(lastwpPos, lastwpPos + wpLeft);

        /*
        for (int i = 0; i < path.corners.Length - 1; i++)
        {
            Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.red);
            Debug.DrawLine(path.corners[i], path.corners[i] + Vector3.up, Color.red);
        }
        */

        Debug.DrawLine(lastwpPos, lastwpPos + Vector3.up, Color.green);
        Debug.DrawLine(wpPos, wpPos + Vector3.up, Color.green);
    }

    
}
