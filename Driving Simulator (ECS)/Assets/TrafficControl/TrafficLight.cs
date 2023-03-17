using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrafficLight : TrafficControlDevice
{
    public enum State
    {
        GO, CAUTION, STOP, COUNT
    }

    public State state = State.GO;
    public float[] lightDurations = { 8f, 2f, 10f };

    private float timer = 0;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Loops through light state according to durations of each light state
    void FixedUpdate()
    {
        float currDuration = lightDurations[(int)state];

        if (timer >= currDuration)
        {
            state = (State)(((int)++state) % ((int) State.COUNT));
            timer -= currDuration;
            //Debug.Log(state);
        }
        
        timer += Time.fixedDeltaTime;
    }
}
