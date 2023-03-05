using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UI : MonoBehaviour
{

    public Vehicle vehicle;
    public Image speedometerArrow;
    public Image blindSpotIndicatorLeft;
    public Image blindSpotIndicatorRight;
    public float speedScalingFactor = 2.5f;
    public float blindSpotFlashInterval = 0.2f;
    public float flashDuration = 2.0f;
    public Color flashColor;
    public bool blindSpotleft = false;
    public bool blindSpotright = false;
    public bool blindSpotFlashing = false;

    private Color originalColor;

    private void BlindSpotIndicatorFlash(int LeftOrRight)
    {
        
    }

    IEnumerator FlashCoroutine()
    {
        blindSpotFlashing = true;
        float startTime = Time.time;
        while (Time.time - startTime < flashDuration)
        {
            blindSpotIndicatorLeft.color = flashColor;
            yield return new WaitForSeconds(blindSpotFlashInterval);
            blindSpotIndicatorLeft.color = originalColor;
            yield return new WaitForSeconds(blindSpotFlashInterval);
        }
        blindSpotFlashing = false;
    }

    // Start is called before the first frame update
    void Start()
    {
        originalColor = blindSpotIndicatorLeft.color;

    }

    // Update is called once per frame
    void Update()
    {
        vehicle.GetSpeed();

        speedometerArrow.transform.rotation = Quaternion.Euler(0, 0, -vehicle.GetSpeed() * speedScalingFactor);

        if (blindSpotleft)
        {
            if (!blindSpotFlashing)
            {
                StartCoroutine(FlashCoroutine());
            }
            blindSpotleft = false;
        }

    }
}
