using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    LineRenderer lineRender;

    Vector3 start;
    Vector3 end;

    int pathNum;
    Vector3[] smoothPath = new Vector3[2048];

    // Use this for initialization
    void Start ()
    {
        lineRender = GetComponent<LineRenderer>();
    }
	
	// Update is called once per frame
	void Update ()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                start = hit.point;
                Debug.Log("Start: " + start);

                if (RecastNavigationDllImports.PathFind(start, end, ref pathNum, ref smoothPath))
                {
                    lineRender.positionCount = pathNum;
                    for (int i = 0; i < pathNum; i++)
                    {
                        lineRender.SetPosition(i, smoothPath[i]);
                    }
                }
            }
        }

        if (Input.GetMouseButtonDown(1))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                end = hit.point;
                Debug.Log("end: " + end);

                if (RecastNavigationDllImports.PathFind(start, end, ref pathNum, ref smoothPath))
                {
                    lineRender.positionCount = pathNum;
                    for (int i = 0; i < pathNum; i++)
                    {
                        lineRender.SetPosition(i, smoothPath[i]);
                    }
                }
            }
        }
    }
}
