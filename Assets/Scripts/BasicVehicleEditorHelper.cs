using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

#if UNITY_EDITOR

[ExecuteAlways, RequireComponent(typeof(BasicVehicle))]
public class BasicVehicleEditorHelper : MonoBehaviour
{
    public BasicVehicle m_basicVehicle;

    private void Awake()
    {
        
    }

    void Update()
    {
        if(!Application.isPlaying)
        {
            m_basicVehicle.UpdateWheels();
        }
    }

    private void OnValidate()
    {
        m_basicVehicle = GetComponent<BasicVehicle>();
        m_basicVehicle.UpdateWheels();
    }

    private Vector3 CalculateWheelDebugLabelPosition(WheelCollider wheel, Vector3 offset)
    {
        Vector3 center = transform.TransformPoint(wheel.center);
        float steering = wheel.steerAngle;
        Vector3 offsetWheelRight = (Quaternion.Euler(0, steering, 0) * transform.rotation) * offset;
        Vector3 pos = center + offsetWheelRight;
        return pos;
    }

    private string GetWheelDebugLabel(WheelCollider wheel, WheelHit hit, string name)
    {
        Vector3 pointV = m_basicVehicle.m_rigidBody.GetRelativePointVelocity(hit.point);
        float pointSpd = Vector3.Dot(pointV, hit.forwardDir.normalized);
        float loadMass = wheel.sprungMass;
        float wheelForce = hit.force;

        float rotLinearSpd = Mathf.Deg2Rad * wheel.rotationSpeed * wheel.radius;
        string strDebug = $"[{name}]\n" +
            $"MotorTorq: {wheel.motorTorque:F1}(Nm)" +
            //$"\nRotSpeed|RPM: {wheel.rotationSpeed:F2}(deg/s) | {wheel.rpm:F1}(r/m)" +
            $"\nLnrSpd|PtSpd: {rotLinearSpd:F2}(m/s) | {pointSpd:F2}(m/s)" +
            $"\nLoadMass: {loadMass:F2}(kg)" +
            $"\nForce: {wheelForce:F2}(N)" +
            $"\nFwdSlip|SdSlip: {hit.forwardSlip:F2}|{hit.sidewaysSlip:F2}";
        return strDebug;
    }

    static float spanX = 2.0f;
    static float spanY = 1.0f;
    static Vector3[] labelOffsets = new Vector3[]
    {
        new Vector3(-spanX, spanY, 0),
        new Vector3(spanX, spanY, 0),
        new Vector3(-spanX, -spanY, 0),
        new Vector3(spanX, -spanY, 0)
    };

    static string[] wheelNames = new string[]
    {
        "FL","FR","RL","RR"
    };

    private void OnDrawGizmos()
    {
        if (!m_basicVehicle.m_rigidBody 
            || m_basicVehicle.Wheels is null
            || m_basicVehicle.WheelHits is null)
        {
            return;
        }

        GUIStyle debugGUIStyleNormal = new GUIStyle(EditorStyles.boldLabel)
        {
            normal = { textColor = Color.blue },
            alignment = TextAnchor.MiddleCenter,
            fontSize = 8
        };

        bool bDrawVehicleDebug = VehicleCVars.DrawVehicleDebug.Value;
        bool bDrawWheelDebug = VehicleCVars.DrawWheelDebug.Value;

        if (bDrawVehicleDebug)
        {
            Vector3 dragForce = m_basicVehicle.VehicleDragForce;
            DebugDraw.DrawArrow(m_basicVehicle.m_rigidBody.transform.position,
                dragForce, Color.red, 0.8f);
        }

        if (bDrawWheelDebug)
        {
            for (int i = 0; i< m_basicVehicle.Wheels.Count; ++i)
            {
                WheelCollider wheel = m_basicVehicle.Wheels[i];
                WheelHit wheelHit = m_basicVehicle.WheelHits[i];
                Vector3 labelOffset = labelOffsets[i];
                string labelName = wheelNames[i];

                Vector3 pos = CalculateWheelDebugLabelPosition(wheel, labelOffset);
                string label = GetWheelDebugLabel(wheel, wheelHit, labelName);
                Handles.Label(pos, label, debugGUIStyleNormal);

                //Vector3 arrowStart = wheelHit.point;
                //Vector3 force = wheelHit.force;
                //arrowStart.y += 0.1f;
                //DebugDraw.DrawArrow(arrowStart, )
            }

            //Vector3 posLF = CalculateWheelDebugLabelPosition(m_basicVehicle.m_frontLeft, m_basicVehicle. );
            //string labelLF = GetWheelDebugLabel(m_basicVehicle.m_frontLeft, "FL");
            //Handles.Label(posLF, labelLF, debugGUIStyleNormal);

            //Vector3 posRF = CalculateWheelDebugLabelPosition(m_basicVehicle.m_frontRight, );
            //string labelRF = GetWheelDebugLabel(m_basicVehicle.m_frontRight, "FR");
            //Handles.Label(posRF, labelRF, debugGUIStyleNormal);

            //Vector3 posLR = CalculateWheelDebugLabelPosition(m_basicVehicle.m_rearLeft, );
            //string labelLR = GetWheelDebugLabel(m_basicVehicle.m_rearLeft, "RL");
            //Handles.Label(posLR, labelLR, debugGUIStyleNormal);

            //Vector3 posRR = CalculateWheelDebugLabelPosition(m_basicVehicle.m_rearRight, );
            //string labelRR = GetWheelDebugLabel(m_basicVehicle.m_rearRight, "RR");
            //Handles.Label(posRR, labelRR, debugGUIStyleNormal);
        }
    }
}

#endif