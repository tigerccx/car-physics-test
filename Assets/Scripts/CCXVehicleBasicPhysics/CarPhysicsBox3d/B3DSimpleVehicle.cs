using B3DVehicle;
using Box3D;
using Box3D.Hybrid;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[DefaultExecutionOrder(100)]

public class B3DSimpleVehicle : MonoBehaviour
{
    public VehicleProperties m_vehicleProperties;

    [SerializeField, ReadOnly]
    private VehicleState m_vehicleState;

    // Vehicle Properties
    //TODO: may be added into VehicleProperties later
    private Box3DBody m_b3dBody;
    private List<Suspension> m_suspensions = new List<Suspension>();
    private List<Wheel> m_wheels = new List<Wheel>();

    //Debug
    [Header("Debug")]
    public float m_debugDrawScale = 0.1f;


    // Start is called before the first frame update
    void Start()
    {
        m_b3dBody = GetComponent<Box3DBody>();
        m_vehicleState.m_b3dBody = m_b3dBody;

        m_suspensions = GetComponentsInChildren<Suspension>().ToList();
        m_wheels.Capacity = m_suspensions.Count;
        foreach (Suspension suspension in m_suspensions)
        {
            Wheel wheel = suspension.GetComponent<Wheel>();
            if(!wheel)
            {
                Debug.LogError("Missing a Wheel for Suspension!");
            }
            m_wheels.Add(wheel);

            VehiclePhysics.InitSuspensionState(suspension.m_suspensionProperties, ref suspension.m_suspensionState);
        }

        Box3DDispatcher.Register(m_b3dBody.Body, OnPreIntegratePositions);
    }

    void FixedUpdate()
    {
        //TODO: Temp Context. Patch in Control
        VehicleContext context = new VehicleContext();
        context.m_surfaceFrictionCoeff = 1.0f;
        context.m_inputs.m_steering = 0.0f;

        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            //Suspension
            VehiclePhysics.UpdateSuspensionAndApplyToVehicle(
                Time.fixedDeltaTime,
                suspension.m_suspensionProperties,
                wheel.m_wheelProperties,
                ref suspension.m_suspensionState,
                ref m_vehicleState);
        }

        //Wheel
        int countIterWheel = 1;
        for (int iterWheel = 0; iterWheel < countIterWheel; ++iterWheel)
        {
            for (int i = 0; i < m_suspensions.Count; ++i)
            {
                Suspension suspension = m_suspensions[i];
                Wheel wheel = m_wheels[i];

                VehiclePhysics.UpdateWheelAndApplyToVehicle_Main(
                    Time.fixedDeltaTime,
                    context,
                    suspension.m_suspensionProperties,
                    wheel.m_wheelProperties,
                    in suspension.m_suspensionState,
                    ref wheel.m_wheelState,
                    ref m_vehicleState);
            }
        }

        //Vehicle
        //VehiclePhysics.UpdateVehicle(m_vehicleProperties, ref m_vehicleState);

        //print($"{m_wheels[0].gameObject.name} -> {m_suspensions[0].m_suspensionState.ToString()}");
        //print($"{m_wheels[0].gameObject.name} -> {m_wheels[0].m_wheelState.ToString()}");

        //print($"{m_wheels[1].gameObject.name} -> {m_suspensions[1].m_suspensionState.ToString()}");
        //print($"{m_wheels[1].gameObject.name} -> {m_wheels[1].m_wheelState.ToString()}");

        //print(new string('-', 100));
    }

    private void OnPreIntegratePositions(in Body body,
        in Vector3 deltaPosition, in Quaternion deltaRotation, float subStepDeltaTime)
    {
        //TODO: Temp Context. Patch in Control
        VehicleContext context = new VehicleContext();
        context.m_surfaceFrictionCoeff = 1.0f;
        context.m_inputs.m_steering = 0.0f;

        //Wheel
        int countIterWheel = 1;
        for (int iterWheel = 0; iterWheel < countIterWheel; ++iterWheel)
        {
            for (int i = 0; i < m_suspensions.Count; ++i)
            {
                Suspension suspension = m_suspensions[i];
                Wheel wheel = m_wheels[i];

                VehiclePhysics.UpdateWheelAndApplyToVehicle_SubStep_PrePosIntegration(
                    in deltaPosition, in deltaRotation, subStepDeltaTime,
                    context,
                    suspension.m_suspensionProperties,
                    wheel.m_wheelProperties,
                    in suspension.m_suspensionState,
                    ref wheel.m_wheelState,
                    ref m_vehicleState);
            }
        }
    }

    #region Debug
    public void OnDrawGizmos()
    {
        if (CCXVehicleCVars.DrawDebug.Value || CCXVehicleCVars.DrawSuspensionDebug.Value)
        {
            float drawScale = CCXVehicleCVars.DrawSuspensionDebugScale.Value;
            foreach (Suspension suspension in m_suspensions)
            {
                SuspensionState suspensionState = suspension.m_suspensionState;
                Vector3 start = suspensionState.m_currentVehicleAttachPoint_ws;
                Vector3 dir = suspensionState.m_currentDir_ws;
                Vector3 dirVelocity = suspensionState.m_currentSuspensionVelocity * dir * drawScale;
                Vector3 dirSuspensionDamperF = suspensionState.m_currentSuspensionDamperForce * dir * drawScale;
                Vector3 dirSuspensionF = suspensionState.m_currentSuspensionForce * dir * drawScale;
                Vector3 end = start - suspensionState.m_currentSuspensionLength * dir;
                DebugDraw.DrawArrow(start, dirVelocity, Color.blue);
                DebugDraw.DrawArrow(start, dirSuspensionDamperF, Color.red);
                DebugDraw.DrawArrow(start, dirSuspensionF, Color.green);
                DebugDraw.DrawSuspension(start, end, Color.yellow);
            }
        }

        if(CCXVehicleCVars.DrawDebug.Value || CCXVehicleCVars.DrawWheelDebug.Value)
        {
            float drawScale = CCXVehicleCVars.DrawWheelDebugScale.Value;
            foreach (Wheel wheel in m_wheels)
            {
                WheelState wheelState = wheel.m_wheelState;
                WheelProperties wheelProperties = wheel.m_wheelProperties;
                Vector3 start = wheelState.m_currentPos_ws;
                Vector3 dirVelocity = wheelState.m_currentVelocity_ws * drawScale;
                Vector3 dirCounterSlide = wheelState.m_currentLateralImpulse * wheelState.m_currentDirLateral_ws * drawScale;
                Vector3 dirFwd = wheelState.m_currentDirFoward_ws;
                Vector3 dirUp = wheelState.m_currentDirUp_ws;
                Vector3 dirRight = Vector3.Cross(dirFwd, dirUp);
                Quaternion rot = Quaternion.LookRotation(dirUp, dirRight);
                DebugDraw.DrawArrow(start, dirVelocity, Color.blue);
                DebugDraw.DrawArrow(start, dirCounterSlide, Color.red);
                DebugDraw.DrawArrow(start, dirFwd, Color.yellow);
                DebugDraw.DrawWheel(start, rot, wheelProperties.m_wheelRadius, new Color(0,0,0, 0.5f));
            }
        }
    }
    #endregion
}
