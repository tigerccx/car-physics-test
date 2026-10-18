using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClickingManager : MonoBehaviour
{
    public IClickable m_currentClick = null;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        bool bEnterClick = Input.GetMouseButtonDown(0);
        bool bDuringClick = Input.GetMouseButton(0);
        bool bExitClick = Input.GetMouseButtonUp(0);

        if (bEnterClick)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if(Physics.Raycast(ray, out hit, 1000))
            {
                IClickable clickable = hit.collider.GetComponent<IClickable>();
                if (clickable != null && m_currentClick == null)
                {
                    m_currentClick = clickable;
                    m_currentClick.OnEnterClicked();
                }
            }
        }
        else if(bDuringClick)
        {
            if(m_currentClick != null)
            {
                m_currentClick.OnHoldClicked();
            }
        }
        else if(bExitClick)
        {
            if (m_currentClick != null)
            {
                m_currentClick.OnExitClicked();
                m_currentClick = null;
            }
        }
    }
}
