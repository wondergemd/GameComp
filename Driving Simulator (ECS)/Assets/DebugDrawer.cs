using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class DebugDrawer : MonoBehaviour
{
    // Pooling algorithm to efficiently draw stuff every frame
    const int INIT_NUM_DRAWERS = 1;
    
    List<GameObject> drawers = new List<GameObject>(INIT_NUM_DRAWERS);
    int drawnPerFrame = 0;

    void Start()
    {
        for (int i = 0; i < INIT_NUM_DRAWERS; i++)
        {
            AddDrawer();
        }
    }

    // Call in LateUpdate
    public void Draw3DText(Vector3 pos, string text)
    {
        if (drawnPerFrame == drawers.Count)
        {
            // Expand number of drawers
            AddDrawer();
        }

        // Set text to the next avaliable drawer
        GameObject drawer = drawers[drawnPerFrame];
        drawer.transform.position = pos;
        drawer.transform.rotation = Camera.main.transform.rotation;

        var textMesh = drawer.GetComponent<TextMesh>();
        textMesh.text = text;

        ++drawnPerFrame;
    }

    void Update()
    {
        // Reset drawers after drawing for one frame
        for (int i = drawers.Count - 1; i >= 0; i--)
        {
            GameObject drawer = drawers[i];
            if (i >= drawnPerFrame)
            {
                Destroy(drawer);
                drawers.RemoveAt(i);
            }
            else
            {
                var textMesh = drawer.GetComponent<TextMesh>();
                textMesh.text = "";
            }
        }

        drawnPerFrame = 0;
    }

    GameObject AddDrawer()
    {
        var drawer = new GameObject("TextDrawer");
        var textMesh = drawer.AddComponent<TextMesh>();
        textMesh.characterSize = 0.05f;
        textMesh.fontSize = 64;
        drawers.Add(drawer);
        return drawer;
    }
}
