using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Suspension : MonoBehaviour
{
    public SuspensionProperties m_suspensionProperties;
    [ReadOnly]
    public SuspensionState m_suspensionState;

    [Header("Debug")]
    float m_debugDrawScale = 1f;

    private void Start()
    {
        m_suspensionProperties.m_posConnection_vehicle_ls = transform.localPosition;
    }

    public void OnDrawGizmos()
    {
        if (CCXVehicleCVars.DrawDebug.Value)
        {
            Vector3 start = m_suspensionState.m_currentVehicleAttachPoint_ws;
            Vector3 dirVelocity = m_suspensionState.m_currentSuspensionVelocity * m_suspensionState.m_currentDir_ws * m_debugDrawScale;
            Vector3 dirSuspensionDamperF = m_suspensionState.m_currentSuspensionDamperForce * m_suspensionState.m_currentDir_ws * m_debugDrawScale;
            Vector3 dirSuspensionF = m_suspensionState.m_currentSuspensionForce * m_suspensionState.m_currentDir_ws * m_debugDrawScale;
            DebugDraw.DrawArrow(start, dirVelocity, Color.blue);
            DebugDraw.DrawArrow(start, dirSuspensionDamperF, Color.red);
            DebugDraw.DrawArrow(start, dirSuspensionF, Color.green);
        }
    }
}
