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
    public float speedScalingFactor = 2.5f;
    public float blindSpotFlashInterval = 0.2f;
    public float flashDuration = 2.0f;
    public Color flashColor;

    private bool leftFlashing = false;
    private bool rightFlashing = false;

    private Color originalColor;


    IEnumerator FlashLeftCoroutine()
    {
        leftFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            blindSpotIndicatorLeft.color = flashColor;
            yield return new WaitForSeconds(blindSpotFlashInterval);
            blindSpotIndicatorLeft.color = originalColor;
            yield return new WaitForSeconds(blindSpotFlashInterval);
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
            yield return new WaitForSeconds(blindSpotFlashInterval);
            blindSpotIndicatorRight.color = originalColor;
            yield return new WaitForSeconds(blindSpotFlashInterval);
        }
        rightFlashing = false;
    }

    // Start is called before the first frame update
    void Start()
    {
        originalColor = blindSpotIndicatorLeft.color;
    }

    // Update is called once per frame
    void Update()
    {
        player.Vehicle.GetSpeed();

        speedometerArrow.transform.rotation = Quaternion.Euler(0, 0, -player.Vehicle.GetSpeed() * speedScalingFactor);

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

    }
}
