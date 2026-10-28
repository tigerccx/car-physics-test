using B3DVehicle;
using Box3D;
using Box3D.Hybrid;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows;

[DefaultExecutionOrder(100)]

public class B3DSimpleVehicle : MonoBehaviour, IBox3DFixedPreUpdate, IBox3DFixedPostUpdate
{
    public VehicleProperties m_vehicleProperties;

    [SerializeField, ReadOnly]
    private VehicleState m_vehicleState = new VehicleState();

    [SerializeField, ReadOnly]
    private VehicleContext m_vehicleContext = new VehicleContext();

    // Vehicle Properties
    //TODO: may be added into VehicleProperties later
    private Box3DBody m_b3dBody;
    private List<Suspension> m_suspensions = new List<Suspension>();
    private List<Wheel> m_wheels = new List<Wheel>();
    
    //TODO: 之后应该只分驱动轮与非驱动轮，转向轮与非转向轮。哪一对轮子共用一对差速器。
    private Wheel m_wheelFL = null;
    private Wheel m_wheelFR = null;
    private Wheel m_wheelRL = null;
    private Wheel m_wheelRR = null;

    // Driving
    [Header("Gas")]
    public NormalizedCurveAsset m_gasInputCurve;

    [Header("Steering")]
    public float m_maxSteerAngle = 30.0f;
    public NormalizedCurveAsset m_steeringInputCurve;

    [Header("Engine")]
    public float m_testEngineTorque = 1000.0f;

    // Input States
    private float throttleInput;
    private float steeringInput;

    private System.Random m_random = new System.Random();

    //Debug
    [Header("Debug")]
    public float m_debugDrawScale = 0.1f;

    [HideInInspector] public bool m_debugUseLockedSteering;
    [HideInInspector, Range(-1f, 1f)] public float m_debugLockedSteering;

    private void OnEnable()
    {
        Box3DWorld.RegisterFixedUpdate(this);
    }

    private void OnDisable()
    {
        Box3DWorld.UnregisterFixedUpdate(this);
    }

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
        }

        Box3DDispatcher.Register(m_b3dBody.Body, OnPreIntegratePositions);

        //TODO: Temp here. 之后应该只分驱动轮与非驱动轮，转向轮与非转向轮。哪一对轮子共用一对差速器。
        foreach(Wheel wheel in m_wheels)
        {
            if(wheel.name.EndsWith("_FL"))
            {
                m_wheelFL = wheel;
            }
            else if(wheel.name.EndsWith("_FR"))
            {
                m_wheelFR = wheel;
            }
            else if (wheel.name.EndsWith("_RL"))
            {
                m_wheelRL = wheel;
            }
            else if (wheel.name.EndsWith("_RR"))
            {
                m_wheelRR = wheel;
            }
        }

        InitVehiclePhysics();
    }

    private void Update()
    {
        CacheInputs();
    }

    private void CacheInputs()
    {
        throttleInput = UnityEngine.Input.GetAxis("Vertical");
        steeringInput = UnityEngine.Input.GetAxis("Horizontal");
        //braking = Input.GetKey(KeyCode.Space);
        //if (Input.GetButtonDown("Gear Up"))
        //{
        //    gearChange += 1;
        //}
        //if (Input.GetButtonDown("Gear Down"))
        //{
        //    gearChange -= 1;
        //}
    }

    private void CalculateInputState()
    {
        VehicleInputs inputs = m_vehicleContext.m_inputs;
        inputs.m_throttle = throttleInput;
        //brake = 0.0f;
        inputs.m_steering = m_debugUseLockedSteering ? Mathf.Clamp(m_debugLockedSteering, -1f, 1f) : steeringInput;

        // Remap throttle
        if (m_gasInputCurve)
        {
            inputs.m_throttle = Mathf.Sign(throttleInput) * m_gasInputCurve.Evaluate(Mathf.Abs(throttleInput));
        }

        //if (IsInForwardGear)
        //{
        //    // When in forward gear, reverse throttle input means applying brake. 
        //    if (throttle < 0)
        //    {
        //        brake = Mathf.Abs(throttle);
        //        throttle = 0;
        //    }
        //}
        ////TODO: 之后空挡需要处理
        //else if (IsInReverseGear)
        //{
        //    // When in reverse gear, forward throttle input means applying brake. 
        //    if (throttle > 0)
        //    {
        //        brake = Mathf.Abs(throttle);
        //        throttle = 0;
        //    }
        //}

        //// Nothing to remap with steering for now

        //// Remap braking
        //if (braking)
        //{
        //    brake = 1.0f;
        //}
    }

    private void DevideTorqueBetweenWheels(float torque, out float torqueA, out float torqueB)
    {
        //TODO: 目前仅使用开放差速器，之后支持限滑差速器。
        torqueA = torqueB = torque * 0.5f;
    }

    private void ApplyDrive()
    {
        //float differentialTorque = m_driveOutputShaftTorque * m_driveShaftToDriveAxleGearRatio;
        float differentialTorque = m_testEngineTorque * m_vehicleContext.m_inputs.m_throttle;

        float torqueL = 0.0f;
        float torqueR = 0.0f;
        DevideTorqueBetweenWheels(differentialTorque, out torqueL, out torqueR);

        // Rear-wheel drive
        m_wheelRL.m_wheelState.m_currentTorque = torqueL;
        m_wheelRR.m_wheelState.m_currentTorque = torqueR;

        // Front wheels do not drive
        m_wheelFL.m_wheelState.m_currentTorque = 0f;
        m_wheelFR.m_wheelState.m_currentTorque = 0f;
    }

    private void ApplySteering(float steeringInput)
    {
        float steeringVal = steeringInput;
        if (m_steeringInputCurve)
        {
            steeringVal = m_steeringInputCurve.Evaluate(steeringInput);
        }
        float steerAngle = steeringVal * m_maxSteerAngle;

        //TODO: 实现steering平滑
        m_wheelFL.m_wheelState.m_currentSteerAngle = steerAngle;
        m_wheelFR.m_wheelState.m_currentSteerAngle = steerAngle;
    }

    private void UpdateVehiclePhysics_SuspensionStates()
    {
        //Suspension
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            VehiclePhysics.UpdateSuspensionStateAndForce(
                Time.fixedDeltaTime,
                suspension.m_suspensionProperties,
                wheel.m_wheelProperties,
                m_vehicleState,
                ref suspension.m_suspensionState);
        }
    }

    private void UpdateVehiclePhysics_WheelStates()
    {
        //Wheel
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            VehiclePhysics.UpdateWheelState(
                m_vehicleContext,
                wheel.m_wheelProperties,
                suspension.m_suspensionProperties,
                m_vehicleState,
                suspension.m_suspensionState,
                ref wheel.m_wheelState);
        }
    }

    private void UpdateVehiclePhysics_ApplySuspensionForce()
    {
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            VehiclePhysics.UpdateVehicleStateFromSuspension(suspension.m_suspensionState, ref m_vehicleState);
        }
    }

    private void UpdateVehiclePhysics_FixedUpdate()
    {
        // Update wheel state with new inputs
        UpdateVehiclePhysics_WheelStates();

        // Apply suspension force
        UpdateVehiclePhysics_ApplySuspensionForce();
    }

    public void InitVehiclePhysics()
    {
        foreach (Suspension suspension in m_suspensions)
        {
            VehiclePhysics.InitSuspensionState(suspension.m_suspensionProperties, ref suspension.m_suspensionState);
        }

        UpdateVehiclePhysics_SuspensionStates();
        UpdateVehiclePhysics_WheelStates();

        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            suspension.m_suspensionState.m_currentSuspensionVelocity = 0.0f;
        }

        UpdateVehiclePhysics_SuspensionStates();
    }

    private void UpdateVehiclePhysics_PrePhysics()
    {
        // Initialize wheel state for transitional state calculations and substeps
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            VehiclePhysics.PrepareWheelSubsteps(
                    Time.fixedDeltaTime,
                    m_vehicleContext,
                    suspension.m_suspensionProperties,
                    wheel.m_wheelProperties,
                    in suspension.m_suspensionState,
                    ref wheel.m_wheelState,
                    ref m_vehicleState);
        }
    }

    private void UpdateVehiclePhysics_Substep_PrePosIntegration(
        Vector3 deltaPosition, 
        Quaternion deltaRotation, 
        float subStepDeltaTime)
    {
        // This is already a substep on Box3D level (if configured)
        // But more substeps for wheel simulation itself

        int countIterWheel = 4; //More wheel substep
        //float dividedSubStepDeltaTime = subStepDeltaTime / countIterWheel;
        //for (int iterWheel = 0; iterWheel < countIterWheel; ++iterWheel)
        //{
        //    foreach (int idx in Enumerable.Range(0, 4).OrderBy(x => m_random.Next()))
        //        //for (int i = 0; i < m_suspensions.Count; ++i)
        //    {
        //        //int idx = (iterWheel + i) % m_suspensions.Count;

        //        Suspension suspension = m_suspensions[idx];
        //        Wheel wheel = m_wheels[idx];

        //        VehiclePhysics.UpdateWheelSubstep(
        //            in deltaPosition, in deltaRotation, 
        //            dividedSubStepDeltaTime,
        //            m_vehicleContext,
        //            suspension.m_suspensionProperties,
        //            wheel.m_wheelProperties,
        //            in suspension.m_suspensionState,
        //            ref wheel.m_wheelState,
        //            ref m_vehicleState);
        //    }
        //}

        VehiclePhysics.UpdateWheelSubstep(
                    in deltaPosition, in deltaRotation,
                    subStepDeltaTime,
                    m_vehicleContext, 4, countIterWheel,
                    m_suspensions,
                    m_wheels,
                    ref m_vehicleState);
    }

    private void UpdateVehiclePhysics_PostPhysics()
    {
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            Wheel wheel = m_wheels[i];

            VehiclePhysics.FinishWheelSubsteps(
                Time.fixedDeltaTime,
                m_vehicleContext,
                suspension.m_suspensionProperties,
                wheel.m_wheelProperties,
                in suspension.m_suspensionState,
                ref wheel.m_wheelState,
                ref m_vehicleState);
        }

        UpdateVehiclePhysics_SuspensionStates();
        UpdateVehiclePhysics_WheelStates();
    }

    void FixedUpdate()
    {
        CalculateInputState();
        ApplySteering(m_vehicleContext.m_inputs.m_steering);
        ApplyDrive();

        m_vehicleContext.m_surfaceFrictionCoeff = 1.0f;

        UpdateVehiclePhysics_FixedUpdate();
    }

    public void Box3DPrePhysics()
    {
        UpdateVehiclePhysics_PrePhysics();
    }

    private void OnPreIntegratePositions(in Body body,
        in Vector3 deltaPosition, in Quaternion deltaRotation, float subStepDeltaTime)
    {
        UpdateVehiclePhysics_Substep_PrePosIntegration(deltaPosition, deltaRotation, subStepDeltaTime);
    }

    public void Box3DPostPhysics()
    {
        UpdateVehiclePhysics_PostPhysics();
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
