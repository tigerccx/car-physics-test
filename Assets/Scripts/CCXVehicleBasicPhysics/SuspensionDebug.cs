using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class SuspensionDebug : MonoBehaviour, IClickable
{
    public Vector3 m_posOriginal_ws;
    public Vector3 m_dir_ws;

    public WheelProperties m_wheelProperties;
    public SuspensionProperties m_suspensionProperties;

    [SerializeField, ReadOnly]
    private SuspensionState m_suspensionState;
    [SerializeField, ReadOnly]
    private VehicleState m_vehicleState;
    private Rigidbody m_rigidbody;

    //Debug
    [Header("Debug")]
    [ReadOnly]
    public bool m_isInteracting;
    public float m_debugDrawScale = 0.1f;

    // Start is called before the first frame update
    void Start()
    {
        m_posOriginal_ws = transform.localPosition;
        m_suspensionProperties.m_suspensionDirection_vehicle_ls.Normalize();
        m_dir_ws = transform.TransformDirection(m_suspensionProperties.m_suspensionDirection_vehicle_ls);
        m_dir_ws.Normalize();

        m_rigidbody = GetComponent<Rigidbody>();
        m_vehicleState.m_rigidbody = m_rigidbody;
    }

    // Update is called once per frame
    void Update()
    {
        // Debug drag
        if(m_isInteracting)
        {
            DebugDrag();
        }
    }

    private void FixedUpdate()
    {
        if (m_isInteracting)
        {
            return;
        }

        m_suspensionState.m_currentSuspensionVelocity = Vector3.Dot(m_rigidbody.velocity, m_dir_ws);
        m_suspensionState.m_currentSuspensionCompression = Vector3.Dot((transform.position - m_posOriginal_ws), m_dir_ws);
        m_suspensionState.m_currentVehicleAttachPoint_ws = transform.position;
        m_suspensionState.m_currentDir_ws = m_dir_ws;

        VehiclePhysics.CalculateSuspensionForce(m_suspensionProperties, ref m_suspensionState);

        // Eps以下记为已经settle
        if (VehiclePhysics.CutOffSuspensionEndStatesIfSettled(ref m_suspensionState))
        {
            return;
        }

        VehiclePhysics.UpdateVehicleStateFromSuspension(m_suspensionState, ref m_vehicleState);
    }

    #region Debug
    public void OnHoldClicked()
    {
        m_isInteracting = true;
    }

    public void OnExitClicked()
    {
        m_isInteracting = false;
    }

    public void DebugDrag()
    {
        Matrix4x4 localToWorld = transform.parent ? transform.parent.localToWorldMatrix : Matrix4x4.identity;

        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        Vector3 p0 = ray.origin;
        Vector3 d0 = ray.direction;
        
        Vector3 p1 = m_posOriginal_ws;
        Vector3 d1 = m_dir_ws;

        Vector3 u = d0.normalized;
        Vector3 v = d1.normalized;
        Vector3 w0 = p0 - p1;

        float a = Vector3.Dot(u, u); // = 1
        float b = Vector3.Dot(u, v);
        float c = Vector3.Dot(v, v); // = 1
        float d = Vector3.Dot(u, w0);
        float e = Vector3.Dot(v, w0);

        float denom = a * c - b * b;

        float t, s;

        if (Mathf.Abs(denom) < 1e-6f)
        {
            t = 0;
            s = e / c;
        }
        else
        {
            t = (b * e - c * d) / denom;
            s = (a * e - b * d) / denom;
        }

        Vector3 newPos_ws = p1 + s * v;

        transform.position = newPos_ws;
    }

    public void OnDrawGizmos()
    {
        if (CCXVehicleCVars.DrawDebug.Value)
        {
            Vector3 start = transform.position;
            Vector3 dirVelocity = m_suspensionState.m_currentSuspensionVelocity * Vector3.up * m_debugDrawScale;
            Vector3 dirSuspensionDamperF = m_suspensionState.m_currentSuspensionDamperForce * Vector3.up * m_debugDrawScale;
            Vector3 dirSuspensionF = m_suspensionState.m_currentSuspensionForce * Vector3.up * m_debugDrawScale;
            DebugDraw.DrawArrow(start, dirVelocity, Color.blue);
            DebugDraw.DrawArrow(start, dirSuspensionDamperF, Color.red);
            DebugDraw.DrawArrow(start, dirSuspensionF, Color.green);
        }
    }
    #endregion
}
