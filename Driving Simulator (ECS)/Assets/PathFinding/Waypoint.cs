using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint : MonoBehaviour
{
    //public string wpName;
    //public float width;
    //public int lanes;
    //public bool oneWay;
    
    public List<Waypoint> neighbors;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
    }

    public Vector3 GetPosition()
    {
        return transform.position;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(GetPosition(), 1.0f);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(GetPosition(), 1.0f);

        Gizmos.color = Color.green;
        foreach (Waypoint wp in neighbors)
        {
            if (wp != null)
            {
                Gizmos.DrawSphere(wp.GetPosition(), 1.0f);
            }
        }
    }
}
