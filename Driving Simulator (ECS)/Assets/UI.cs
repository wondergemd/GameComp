using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;




public class UI : MonoBehaviour
{

    public Vehicle vehicle;
    public Player player;

    public Image speedometerArrow;
    public Image blindSpotIndicatorLeft;
    public Image blindSpotIndicatorRight;
    public Image forwardCollisionDetection;

    public Text SpeedText;

    private float textUpdateTimer = 0f;

    public float flashInterval = 0.2f;
    public float flashDuration = 2.0f;

    public Color baseColor;
    public Color flashColor;

    public bool debug = false;

    public bool leftFlashing = false;
    public bool rightFlashing = false;
    public bool forwardCollisionFlashing = false;


    IEnumerator FlashLeftCoroutine()
    {
        leftFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            blindSpotIndicatorLeft.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            blindSpotIndicatorLeft.color = baseColor;
            yield return new WaitForSeconds(flashInterval);
        }
        leftFlashing = false;
    }


    IEnumerator FlashRightCoroutine()
    {
        rightFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            blindSpotIndicatorRight.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            blindSpotIndicatorRight.color = baseColor;
            yield return new WaitForSeconds(flashInterval);
        }
        rightFlashing = false;
    }


    IEnumerator FlashForwardCollisionCoroutine()
    {
        forwardCollisionFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            forwardCollisionDetection.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            forwardCollisionDetection.color = baseColor;
            yield return new WaitForSeconds(flashInterval);
        }
        forwardCollisionFlashing = false;
    }


    // Utility function to flash individual UI image element
    // currently not working because the function cannot pass by reference, meaning function 
    // will repeat every update cycle since the bool value is not being updated in the public scope
    IEnumerator FlashElement(bool flashing, Image image)
    {
        flashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            image.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            image.color = baseColor;
            yield return new WaitForSeconds(flashInterval);
        }
        flashing = false;
    }


    private void BlindSpotIndicatorsMain()
    {
        // LEFT BLIND SPOT INDICATOR
        if (player.blindSpotLeft)
        {
            if (!leftFlashing)
            {
                StartCoroutine(FlashLeftCoroutine());
            }
            player.blindSpotLeft = false;
        }

        // LEFT BLIND SPOT INDICATOR
        if (player.blindSpotRight)
        {
            if (!rightFlashing)
            {
                StartCoroutine(FlashRightCoroutine());
            }
            player.blindSpotRight = false;
        }

    }

    private void ForwardCollisionIndicatorsMain()
    {
        if (player.collisionDetected)
        {
            if (!forwardCollisionFlashing)
            {
                StartCoroutine(FlashForwardCollisionCoroutine());
            }
        }

    }


    // Rotates Speedometer
    private void RotateSpeedometer()
    {
        // convert m/s to MPH
        float mph = (player.playerVehicle.GetSpeed() * 2.237f);

        // percentage of mph value to max speedometer value - 240 mph
        float mphSpeedometer = mph / 240f;

        // rotate speedometer UI element
        speedometerArrow.transform.rotation = Quaternion.Euler(0, 0, -(mphSpeedometer * 360f));
    }


    // used to display speed converted to MPH next to speedometer for debugging
    // can probably be deleted
    private void debugSpeed()
    {
        //SPEED DEBUG TEXT
        if (debug)
        {
            SpeedText.enabled = true;
            if (textUpdateTimer > 0.1f)
            {
                textUpdateTimer = 0f;

                string speedStr = string.Format("M/S: {0:0.00}\n" +
                    "MPH: {1:0.00}", player.playerVehicle.GetSpeed(), player.playerVehicle.GetSpeed() * 2.237);
                SpeedText.text = speedStr;
            }

            textUpdateTimer += Time.fixedDeltaTime;
        }
        else
        {
            SpeedText.enabled = false;
        }
    }


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        RotateSpeedometer();

        BlindSpotIndicatorsMain();

        ForwardCollisionIndicatorsMain();
    }
}
