using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UI : MonoBehaviour
{

    public Vehicle vehicle;
    public Player player;

    public Text SpeedText;

    private float textUpdateTimer = 0f;

    public Image speedometerArrow;
    public Image blindSpotIndicatorLeft;
    public Image blindSpotIndicatorRight;
    public Image forwardCollisionDetection;
    public float flashInterval = 0.2f;
    public float flashDuration = 2.0f;
    public Color flashColor;
    public bool debug = false;

    private bool leftFlashing = false;
    private bool rightFlashing = false;
    private bool collisionDetectionFlashing = false;

    private Color originalColor;



    IEnumerator FlashLeftCoroutine()
    {
        leftFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            blindSpotIndicatorLeft.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            blindSpotIndicatorLeft.color = originalColor;
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
            blindSpotIndicatorRight.color = originalColor;
            yield return new WaitForSeconds(flashInterval);
        }
        rightFlashing = false;
    }

    IEnumerator Flash(bool flashing, Image image)
    {
        flashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            image.color = flashColor;
            yield return new WaitForSeconds(flashInterval);
            image.color = originalColor;
            yield return new WaitForSeconds(flashInterval);
        }
        flashing = false;
    }

    // Start is called before the first frame update
    void Start()
    {
        originalColor = blindSpotIndicatorLeft.color;
    }

    // Update is called once per frame
    void Update()
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
        } else
        {
            SpeedText.enabled = false;
        }


        speedometerArrow.transform.rotation = Quaternion.Euler(0, 0, -(((player.playerVehicle.GetSpeed() * 2.237f)/240f)*360f));

        if (player.blindSpotLeft)
        {
            if (!leftFlashing)
            {
                StartCoroutine(FlashLeftCoroutine());
            }
            player.blindSpotLeft = false;
        }

        if (player.blindSpotRight)
        {
            if (!rightFlashing)
            {
                StartCoroutine(FlashRightCoroutine());
            }
            player.blindSpotRight = false;
        }

        /*
        if (player.collisionDetected)
        {
            StartCoroutine(Flash(collisionDetectionFlashing, forwardCollisionDetection));
        }
        */

    }
}
