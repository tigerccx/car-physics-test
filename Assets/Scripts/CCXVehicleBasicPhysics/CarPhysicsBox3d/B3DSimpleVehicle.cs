using B3DVehicle;
using Box3D;
using Box3D.Hybrid;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Windows;

[DefaultExecutionOrder(100)]

public class B3DSimpleVehicle : MonoBehaviour, IBox3DFixedPreUpdate, IBox3DFixedPostUpdate
{
    [FormerlySerializedAs("m_vehicleProperties")]
    public VehicleProperties m_vehicleProperty;

    [SerializeField, ReadOnly]
    public VehicleState m_vehicleState = new VehicleState();

    [SerializeField, ReadOnly]
    public VehicleContext m_vehicleContext = new VehicleContext();

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

    // Input States
    public float m_throttleInput = 0f;
    public bool m_brakeInput = false;
    public bool m_handbrakeInput = false;
    public float m_steeringInput = 0f;
    public int m_gearChangeInput = 0;

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

    private void OnValidate()
    {
        VehiclePhysics.AutoConfigATGearSettings(ref m_vehicleProperty);
    }

    // Start is called before the first frame update
    void Start()
    {
        m_b3dBody = GetComponent<Box3DBody>();
        m_vehicleState.m_b3dBody = m_b3dBody;

        GatherSuspensions(transform, m_suspensions);
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

        foreach (Wheel wheel in m_wheels)
        {
            InitWheelMesh(wheel);
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
        UpdateWheels();
    }

    private void CalculateInputState()
    {
        VehicleInputs inputs = m_vehicleContext.m_inputs;

        float steering = m_debugUseLockedSteering ? Mathf.Clamp(m_debugLockedSteering, -1f, 1f) : m_steeringInput;
        inputs.m_steering = steering;

        float throttle = m_throttleInput;
        float brake = 0.0f;

        if (VehiclePhysics.IsInForwardGear(m_vehicleState))
        {
            // When in forward gear, reverse throttle input means applying brake. 
            if (throttle < 0)
            {
                brake = Mathf.Abs(throttle);
                throttle = 0;
            }
        }
        //TODO: 之后空挡需要处理
        else if (VehiclePhysics.IsInReverseGear(m_vehicleState))
        {
            // When in reverse gear, forward throttle input means applying brake. 
            if (throttle > 0)
            {
                brake = Mathf.Abs(throttle);
                throttle = 0;
            }
        }

        inputs.m_throttle = throttle;
        inputs.m_throttleIntention = m_throttleInput;

        // Nothing to remap with steering for now

        // Remap braking
        if (m_brakeInput)
        {
            brake = 1.0f;
        }

        inputs.m_brake = brake;
        inputs.m_handbrake = m_handbrakeInput;

        inputs.m_gearChange = m_gearChangeInput;
        m_gearChangeInput = 0;
    }

    private void GatherSuspensions(Transform node, List<Suspension> suspensions)
    {
        suspensions.AddRange(node.GetComponents<Suspension>());

        for (int i = 0; i < node.childCount; i++)
        {
            GatherSuspensions(node.GetChild(i), suspensions);
        }
    }

    private void DevideTorqueBetweenWheels(float torque, out float torqueA, out float torqueB)
    {
        //TODO: 目前仅使用开放差速器，之后支持限滑差速器。
        torqueA = torqueB = torque * 0.5f;
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
        // Update wheel driving states
        VehiclePhysics.UpdateWheelStatesFromVehicle(Time.fixedDeltaTime,
            m_vehicleContext,
            m_vehicleProperty,
            m_vehicleState,
            m_wheels);

        // Update wheel state with new inputs
        UpdateVehiclePhysics_WheelStates();

        // Apply suspension force
        UpdateVehiclePhysics_ApplySuspensionForce();
    }

    public void ResetPhysics(Vector3 position, Quaternion rotation)
    {
        if (m_b3dBody == null || !m_b3dBody.Body.IsValid)
        {
            return;
        }

        // Sleeping clears accumulated forces before the teleport wakes the body.
        m_b3dBody.SetAwake(false);
        m_b3dBody.SetTransform(position, rotation);
        m_b3dBody.velocity = Vector3.zero;
        m_b3dBody.angularVelocity = Vector3.zero;

        m_throttleInput = 0f;
        m_steeringInput = 0f;
        m_brakeInput = false;
        m_handbrakeInput = false;
        m_gearChangeInput = 0;

        m_vehicleState = new VehicleState { m_b3dBody = m_b3dBody };
        m_vehicleContext = new VehicleContext();
        foreach (Wheel wheel in m_wheels)
        {
            wheel.m_wheelState = new WheelState { m_transitional = new WheelTransitionalState() };
        }
        foreach (Suspension suspension in m_suspensions)
        {
            suspension.m_suspensionState = new SuspensionState();
        }

        InitVehiclePhysics();
        UpdateWheels();
    }

    public void InitVehiclePhysics()
    {
        for (int i = 0; i < m_suspensions.Count; ++i)
        {
            Suspension suspension = m_suspensions[i];
            VehiclePhysics.InitSuspension(m_vehicleState, ref suspension);
        }

        VehiclePhysics.InitVehicleState(ref m_vehicleProperty, m_wheels, ref m_vehicleState);

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
                    m_vehicleProperty,
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
        VehiclePhysics.UpdateWheelSubstep(
                    in deltaPosition, in deltaRotation,
                    subStepDeltaTime,
                    m_vehicleContext, m_vehicleProperty,
                    countIterWheel,
                    m_suspensions,
                    m_wheels,
                    ref m_vehicleState);

        VehiclePhysics.UpdateVehicleSubstep(
            subStepDeltaTime,
            m_vehicleContext,
            m_wheels,
            m_vehicleProperty,
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

        VehiclePhysics.UpdateVehicleInternalStates(Time.fixedDeltaTime,
            ref m_vehicleContext,
            m_vehicleProperty,
            m_wheels,
            ref m_vehicleState);
    }

    void FixedUpdate()
    {
        CalculateInputState();

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


    public void UpdateWheels()
    {
        UpdateWheel(m_wheelFL, false);
        UpdateWheel(m_wheelFR, true);
        UpdateWheel(m_wheelRL, false);
        UpdateWheel(m_wheelRR, true);
    }

    private void InitWheelMesh(Wheel wheel)
    {
        if (!wheel.m_wheelMesh)
        {
            return;
        }
        Vector3 localScale = wheel.m_wheelMesh.transform.localScale;
        localScale.x = localScale.z = wheel.m_wheelProperties.m_wheelRadius * 2f;
        wheel.m_wheelMesh.transform.localScale = localScale;
    }

    private void UpdateWheel(Wheel wheel, bool isRightSide)
    {
        if (!wheel.m_wheelMesh)
        {
            return;
        }

        Vector3 position;
        Quaternion rotation;
        VehiclePhysics.GetWheelPose(wheel.m_wheelState, isRightSide, out position, out rotation);

        wheel.m_wheelMesh.transform.position = position;
        wheel.m_wheelMesh.transform.rotation = rotation;
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
