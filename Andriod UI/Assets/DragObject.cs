using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Input = InputWrapper.Input;



public class DragObject : MonoBehaviour
{


    public float endOfFrameLock = 7f;

    void Update()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Stationary || touch.phase == TouchPhase.Moved)
            {

                Ray ray = Camera.main.ScreenPointToRay(touch.position);
                RaycastHit hit;
                Debug.DrawRay(ray.origin, ray.direction * 100, Color.yellow, 100f);
                if (Physics.Raycast(ray, out hit))
                {
                    Debug.Log(hit.transform.name);
                    if (hit.collider != null)
                    {

                        GameObject touchedObject = hit.transform.gameObject;

                        Debug.Log("Touched " + touchedObject.transform.name);
                    }
                }

                Vector3 touchedPos = Camera.main.ScreenToWorldPoint(new Vector3(touch.position.x, touch.position.y, 10));

                touchedPos.y = 0;
                touchedPos.z = 0;

                if (touchedPos.x > endOfFrameLock)
                {
                    touchedPos.x = endOfFrameLock;
                }
                if (touchedPos.x < -endOfFrameLock)
                {
                    touchedPos.x = -endOfFrameLock;
                }

                transform.position = touchedPos;
            }
        }
    }
}
