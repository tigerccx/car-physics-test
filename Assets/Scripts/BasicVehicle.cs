using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SocialPlatforms.Impl;
using System.Linq;
using Unity.Burst.CompilerServices;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using System;




#if UNITY_EDITOR
using UnityEditor;
#endif

//public class WheelAxle
//{

//}

public enum TransmissionSystemType
{
    ManualTransmission,
    AutoTransmission
}

[Serializable]
public class DrivingGear
{
    public string m_gearName;
    // Input Shaft to Output Shaft
    [Min(0.001f)]
    public float m_gearRatio;
    [ReadOnly]
    public float m_ATGearUpRPM;
    [ReadOnly]
    public float m_ATGearDownRPM;
}

public class BasicVehicle : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider m_frontLeft;
    public WheelCollider m_frontRight;
    public WheelCollider m_rearLeft;
    public WheelCollider m_rearRight;
    public float m_substepThresSpeed = 10.0f;
    public int m_substepCountLowSpeed = 12;
    public int m_substepCountHighSpeed = 6;


    [Header("Basic Wheels")]
    public BasicWheel m_frontLeftBW;
    public BasicWheel m_frontRightBW;
    public BasicWheel m_rearLeftBW;
    public BasicWheel m_rearRightBW;

    [Header("Wheel Meshes")]
    public GameObject m_frontLeftMesh;
    public GameObject m_frontRightMesh;
    public GameObject m_rearLeftMesh;
    public GameObject m_rearRightMesh;

    [Header("Body")]
    public float m_dragCoefficient = 0.1f;
    public float m_frontalArea = 1.0f;
    public float m_sideArea = 1.0f;
    public float m_topArea = 1.0f;

    [Header("Mechanics")]
    public float m_mechanicalLoadTorque = 10.0f;
    public float m_driveShaftToDriveAxleGearRatio = 4.0f;

    [Header("Engine")]
    public float m_engineMaxRPM = 5000f;
    public float m_engineIdleRPM = 1000f;
    private float m_engineIdleRotationSpeed = 0.0f;
    //public float m_engineMarginRPM = 20f; //暂不用Margin Grace
    public float m_engineInertia = 10.0f;
    public float m_revLimitReactionTime = 0.2f;
    public NormalizedCurveAsset m_torqueCurve;
    public NormalizedCurveAsset m_redlineTorqueCurve;
    public NormalizedCurveAsset m_gasInputCurve;

    [Header("Transmission - Gear Set")]
    public List<DrivingGear> m_gears = new List<DrivingGear>();
    public DrivingGear m_reverseGear = new DrivingGear();

    [Header("Transmission - Transmission System")]
    public TransmissionSystemType m_transmissionSystemType;
    [Tooltip("AT will shift up gear at Scale * EngineMaxRPM")]
    public float m_ATGearUpRPMScale = 0.99f;
    [Tooltip("AT will shift down gear to meet Scale * EngineMaxRPM")]
    public float m_ATGearDownTargetRPMScale = 0.5f;
    [Tooltip("Transmission will be allowed to switch to reverse gear if speed is lower than Threshold m/s")]
    public float m_forwardToReverseSpeedThreshold = 0.1f;
    [Tooltip("Transmission will be allowed to switch to forward gear if speed is lower than Threshold m/s")]
    public float m_reverseToForwardSpeedThreshold = 0.1f;
    public float ForwardToReverseOutputShaftSpeedThreadhold
    {
        get => AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(m_forwardToReverseSpeedThreshold);
    }
    public float ReverseToForwardOutputShaftSpeedThreadhold
    {
        get => AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(m_reverseToForwardSpeedThreshold);
    }

    [Header("Break")]
    public NormalizedCurveAsset m_brakeTorqueCurve;

    [Header("Driving")]
    public float m_maxSteerAngle = 30f;
    public NormalizedCurveAsset m_steeringInputCurve;

    [Header("Optional Rigidbody Settings")]
    public Vector3 m_centerOfMassOffset = new Vector3(0f, -0.5f, 0f);

    public Rigidbody m_rigidBody;

    private float m_EPS = 1e-8f;

    // Input States
    private float throttleInput;
    private float steeringInput;
    private bool braking;
    private int gearChange;

    //
    // Internal States
    //
    // Vehicle
    public Vector3 m_dragForce;
    public Vector3 VehicleDragForce { get => m_dragForce; }
    public float VehicleSpeed { 
        get
        { 
            if (m_rigidBody != null)
            {
                return Vector3.Dot(m_rigidBody.velocity, m_rigidBody.transform.forward);
            }
            return 0f;
        }
    }
    public float VehicleSpeedKmsPerHour { get => VehicleSpeed * 3.6f; }

    // Wheels
    private List<WheelCollider> m_wheels;
    private List<BasicWheel> m_wheelBWs;
    private List<WheelHit> m_wheelHits;
    public float SlipRL
    {
        get
        {
            if (m_wheelHits is null || m_wheelHits.Count < 4)
            {
                return 0.0f;
            }
            return m_wheelHits[2].forwardSlip;
        }
    }
    public float SlipRR
    {
        get
        {
            if (m_wheelHits is null || m_wheelHits.Count < 4)
            {
                return 0.0f;
            }
            return m_wheelHits[3].forwardSlip;
        }
    }
    public List<WheelCollider> Wheels { get => m_wheels; }
    public List<WheelHit> WheelHits { get => m_wheelHits; }

    // Engine
    private float m_engineRotationSpeed; 
    private float m_engineRPM; 
    private float m_engineOutputTorque; 
    private float m_totalLoadTorque; 
    private float m_engineNetTorque;
    private bool m_revLimitEngaged = false;
    private float m_revLimitEngageTimer = 0.0f;

    public float EngineRotationSpeed { get => m_engineRotationSpeed; private set => m_engineRotationSpeed = value; }
    public float EngineRPM { get => m_engineRPM; private set => m_engineRPM = value; }
    public float EngineOutputTorque { get => m_engineOutputTorque; private set => m_engineOutputTorque = value; }
    public float TotalLoadTorque { get => m_totalLoadTorque; private set => m_totalLoadTorque = value; }
    public float EngineNetTorque { get => m_engineNetTorque; private set => m_engineNetTorque = value; }
    public float EngineOutputPower { get => m_engineOutputTorque * m_engineRotationSpeed * Mathf.Deg2Rad;  }

    // Brake
    public float m_brakeTorque;

    // Gear set
    private int m_idxCurrentGear = 0;
    public DrivingGear CurrentGear 
    { 
        get
        {
            if (m_idxCurrentGear >= 0)
            {
                return m_gears[m_idxCurrentGear];
            }
            else
            {
                return m_reverseGear;
            }
        }
    }
    public bool IsInForwardGear { get { return m_idxCurrentGear >= 0; } }
    public bool IsInReverseGear { get { return m_idxCurrentGear < 0; } }

    // Drive train
    private bool m_bClutchEngaged;
    private float m_driveInputShaftRotationSpeed;
    private float m_driveInputShaftTorque;
    private float m_driveOutputShaftRotationSpeed; 
    private float m_driveOutputShaftTorque; 
    public float DriveOutputShaftRotationSpeed { get => m_driveOutputShaftRotationSpeed; private set => m_driveOutputShaftRotationSpeed = value; }
    public float DriveOutputShaftRPM { get => m_driveOutputShaftRotationSpeed / 6.0f; }

    private void Awake()
    {
        m_rigidBody = GetComponent<Rigidbody>();

        if (m_rigidBody != null)
        {
            m_rigidBody.centerOfMass += m_centerOfMassOffset;
        }

        m_wheels = new[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight }.ToList();

        m_frontLeftBW = m_frontLeft.GetComponent<BasicWheel>();
        m_frontRightBW = m_frontRight.GetComponent<BasicWheel>();
        m_rearLeftBW = m_rearLeft.GetComponent<BasicWheel>();
        m_rearRightBW = m_rearRight.GetComponent<BasicWheel>();
        m_wheelBWs = new[] { m_frontLeftBW, m_frontRightBW, m_rearLeftBW, m_rearRightBW }.ToList();

        m_wheelHits = new List<WheelHit>();

        foreach(WheelCollider wheel in m_wheels)
        {
            wheel.ConfigureVehicleSubsteps(m_substepThresSpeed, m_substepCountLowSpeed, m_substepCountHighSpeed);
        }

        m_engineIdleRotationSpeed = m_engineIdleRPM * 6.0f;

        if(m_gears.Count == 0)
        {
            DrivingGear defaultGear = new DrivingGear();
            defaultGear.m_gearName = "Default Gear";
            defaultGear.m_gearRatio = 1.0f;
            m_gears.Add(defaultGear);
        }
    }

    private void OnValidate()
    {
        //switch (m_transmissionSystemType)
        //{
        //    case TransmissionSystemType.ManualTransmission:
        //        break;
        //    case TransmissionSystemType.AutoTransmission:
        //        break;
        //}

        CalculateATGearSettings();
    }

    private void Update()
    {
        CacheInputs();
    }

    private void CacheInputs()
    {
        throttleInput = Input.GetAxis("Vertical");
        steeringInput = Input.GetAxis("Horizontal");
        braking = Input.GetKey(KeyCode.Space);
        if (Input.GetButtonDown("Gear Up"))
        {
            gearChange += 1;
        }
        if (Input.GetButtonDown("Gear Down"))
        {
            gearChange -= 1;
        }
    }

    private void ClearInstantInputs()
    {
        gearChange = 0;
    }

    private void FixedUpdate()
    {
        //TOOD: 暂时离合总是合拢
        m_bClutchEngaged = true;

        RetrieveWheelHitResults();

        CalculateLoadTorque();
        CalculateDriveShaftRotation();
        CalculateEngineRotation();

        float throttle = 0.0f;
        float steering = 0.0f;
        float brake = 0.0f;

        CalculateInputState(out throttle, out steering, out brake);

        Debug.Log($"Throttle {throttle}; Steering {steering}; Brake {brake}");

        if (m_transmissionSystemType == TransmissionSystemType.ManualTransmission)
        {
            ChangeGear(gearChange);
        }
        else
        {
            CalculateATSwitchForwardReverse();
        }
        CalculateEngineTorque(throttle);

        CalculateDriveShaftTorque();

        ApplyDrag();

        ApplySteering(steeringInput);
        ApplyDrive();
        ApplyBrake(brake);
        UpdateWheels();

        ClearInstantInputs();
    }

    private void CalculateInputState(out float throttle, out float steering, out float brake)
    {
        
        throttle = throttleInput;
        brake = 0.0f;
        steering = steeringInput;

        // Remap throttle
        if (m_gasInputCurve)
        {
            throttle = Mathf.Sign(throttleInput) * m_gasInputCurve.Evaluate(Mathf.Abs(throttleInput));
        }

        if (IsInForwardGear)
        {
            // When in forward gear, reverse throttle input means applying brake. 
            if (throttle < 0)
            {
                brake = Mathf.Abs(throttle);
                throttle = 0;
            }
        }
        //TODO: 之后空挡需要处理
        else if(IsInReverseGear)
        {
            // When in reverse gear, forward throttle input means applying brake. 
            if (throttle > 0)
            {
                brake = Mathf.Abs(throttle);
                throttle = 0;
            }
        }

        // Nothing to remap with steering for now
        
        // Remap braking
        if (braking)
        {
            brake = 1.0f;
        }
    }

    private void ApplySteering(float steeringInput)
    {
        float steeringVal = steeringInput;
        if (m_steeringInputCurve)
        {
            steeringVal = m_steeringInputCurve.Evaluate(steeringInput);
        }
        float steerAngle = steeringVal * m_maxSteerAngle;

        m_frontLeft.steerAngle = steerAngle;
        m_frontRight.steerAngle = steerAngle;
    }

    private void RetrieveWheelHitResults()
    {
        m_wheelHits.Clear();
        foreach (WheelCollider wheelCollider in m_wheels)
        {
            WheelHit wheelHit;
            wheelCollider.GetGroundHit(out wheelHit);
            m_wheelHits.Add(wheelHit);
        }
    }

    private bool CanShiftToReverse()
    {
        return m_idxCurrentGear == 0 && m_driveOutputShaftRotationSpeed < ForwardToReverseOutputShaftSpeedThreadhold;
    }

    private bool CanShiftToForward()
    {
        return m_idxCurrentGear == -1 && m_driveOutputShaftRotationSpeed > -ReverseToForwardOutputShaftSpeedThreadhold;
    }

    private void ChangeGear(int gearChange)
    {
        if (gearChange == 0)
        {
            return;
        }

        int maxGear = m_gears.Count - 1;
        int newGear = math.clamp(m_idxCurrentGear + gearChange, -1, maxGear);

        // Shift to reverse gear
        if (m_idxCurrentGear == 0 && newGear == -1)
        {
            // Forward speed is too high to shift into reverse gear
            if (!CanShiftToReverse())
            {
                newGear = 0;
            }
        }
        // Shift to forward gear
        else if (m_idxCurrentGear == -1 && newGear == 1)
        {
            // Reverse speed is too high to shift into forward gear
            if (!CanShiftToForward())
            {
                newGear = -1;
            }
        }

        m_idxCurrentGear = newGear;
    }

    private void CalculateATSwitchForwardReverse()
    {
        if (IsInForwardGear)
        {
            if (throttleInput < 0 && CanShiftToReverse())
            {
                ChangeGear(-1);
            }
            else
            {
                CalculateATChangeForwardGear();
            }
        }
        //TODO: 实现空挡时这里需要加空挡
        else if (IsInReverseGear)
        {
            if (throttleInput > 0 && CanShiftToForward())
            {
                ChangeGear(1);
            }
        }
    }

    private void CalculateATChangeForwardGear()
    {
        DrivingGear curGear = CurrentGear;
        if (m_engineRPM >= curGear.m_ATGearUpRPM)
        {
            ChangeGear(1);
        }
        else if(m_engineRPM <= curGear.m_ATGearDownRPM)
        {
            ChangeGear(-1);
        }
    }

    private void CalculateLoadTorque()
    {
        float totalLoadTorque = 0.0f;

        // Since slip is unstable, calculating engine load from slip is very unstable as well.
        // The load will be implicitly deducted from wheel rotation speed, which is used to estimate engine rpm

        //Debug.Log("Load frictions calculation");
        //// Wheel Load
        //for (int i=0;i<m_wheels.Count;++i)
        //{
        //    WheelCollider wheelCollider = m_wheels[i];
        //    BasicWheel wheelBW = m_wheelBWs[i];
        //    float sampledFriction = BasicVehicleHelper.SampleFrictionCurve(wheelCollider.forwardFriction, m_wheelHits[i].forwardSlip);
        //    float gravityComp = Vector3.Dot(Physics.gravity, m_wheelHits[i].normal);
        //    gravityComp = Mathf.Abs(gravityComp);
        //    float loadFriction = sampledFriction * wheelCollider.sprungMass * gravityComp;
        //    float loadTorque = loadFriction * wheelCollider.radius * wheelBW.m_loadCoeff;
        //    Debug.Log($"Fric {sampledFriction} | Toq {loadTorque}");
        //    totalLoadTorque += loadTorque;
        //}

        // Mechanical Load
        totalLoadTorque += m_mechanicalLoadTorque;

        //TODO: 暂时先忽略 engine ineria

        m_totalLoadTorque = totalLoadTorque;
    }

    private void CalculateDriveShaftRotation()
    {
        //TODO: 目前先走从驱动轮倒推的EngineRPM
        float diffCarrierRotationSpeed = (m_rearLeft.rotationSpeed + m_rearRight.rotationSpeed) * 0.5f;
        m_driveOutputShaftRotationSpeed = diffCarrierRotationSpeed * m_driveShaftToDriveAxleGearRatio;
        float gearRatio = CurrentGear.m_gearRatio;
        m_driveInputShaftRotationSpeed = Mathf.Max(m_driveOutputShaftRotationSpeed * gearRatio, m_engineIdleRotationSpeed);
    }

    private void CalculateEngineRotation()
    {
        if (m_bClutchEngaged)
        {
            m_engineRotationSpeed = m_driveInputShaftRotationSpeed;
        }
        else
        {
            // TODO: 给Clutch disengage一个曲线，而非用01描述这个过程
            float deltaEngineRotSpeed = m_engineOutputTorque / m_engineInertia * Time.fixedDeltaTime;
            m_engineRotationSpeed += deltaEngineRotSpeed;
        }

        m_engineRPM = m_engineRotationSpeed / 6.0f;
    }

    private void CalculateRevLimitState()
    {
        if (!m_revLimitEngaged)
        {
            if (m_engineRPM > m_engineMaxRPM)
            {
                m_revLimitEngageTimer += Time.fixedDeltaTime;
                if (m_revLimitEngageTimer >= m_revLimitReactionTime)
                {
                    m_revLimitEngaged = true;
                    m_revLimitEngageTimer = 0.0f;
                }
            }
            else
            {
                m_revLimitEngageTimer = 0.0f;
            }
        }
        else
        {
            if (m_engineRPM <= m_engineMaxRPM)
            {
                m_revLimitEngageTimer += Time.fixedDeltaTime;
                if (m_revLimitEngageTimer >= m_revLimitReactionTime)
                {
                    m_revLimitEngaged = false;
                    m_revLimitEngageTimer = 0.0f;
                }
            }
            else
            {
                m_revLimitEngageTimer = 0.0f;
            }
        }
    }

    private void CalculateEngineTorque(float throttle)
    {
        //if (m_engineRPM <= m_engineMaxRPM)
        //{
        //    float sampledTorque = m_torqueCurve.Evaluate(m_engineRPM);
        //    // Engine will shut off torque output when reaching set max RPM
        //    float marginGrace = Mathf.Clamp01((m_engineMaxRPM - m_engineRPM) / Mathf.Max(m_engineMarginRPM, m_EPS));

        //    m_engineOutputTorque = sampledTorque * marginGrace;
        //    m_engineOutputTorque *= throttle;
        //}
        //else
        //{
        //    float engineOverRedlineRPM = m_engineRPM - m_engineMaxRPM;
        //    float engineOverRedlineRPM
        //}

        //float sampledTorque = m_torqueCurve.Evaluate(m_engineRPM);
        //// Engine will shut off torque output when reaching set max RPM
        //float marginGrace = Mathf.Clamp01((m_engineMaxRPM - m_engineRPM) / Mathf.Max(m_engineMarginRPM, m_EPS));

        //m_engineOutputTorque = sampledTorque * marginGrace;
        //m_engineOutputTorque *= throttle;

        CalculateRevLimitState();

        float sampledTorque = m_torqueCurve.Evaluate(m_engineRPM);

        if (!m_revLimitEngaged)
        {
            m_engineOutputTorque = sampledTorque;
            m_engineOutputTorque *= throttle;
        }
        else
        {
            float overRedlineRPM = Mathf.Max(m_engineRPM - m_engineMaxRPM, 0.0f);
            float sampledRedlineTorque = m_redlineTorqueCurve.Evaluate(overRedlineRPM);
            m_engineOutputTorque = -sampledRedlineTorque;
        }

        m_engineNetTorque = m_engineOutputTorque - m_totalLoadTorque;
        if (m_engineOutputTorque > -m_EPS)
        {
            m_engineNetTorque = Mathf.Max(m_engineNetTorque, 0.0f);
        }
    }

    private void CalculateDriveShaftTorque()
    {
        if (m_bClutchEngaged)
        {
            m_driveInputShaftTorque = m_engineNetTorque;
        }
        else
        {
            // Clutch disengaged 
            m_driveInputShaftTorque = 0.0f;
        }

        m_driveOutputShaftTorque = m_driveInputShaftTorque * CurrentGear.m_gearRatio;
    }

    private void ApplyDrag()
    {
        // Calculate Air Drag
        Vector3 v_ws = m_rigidBody.velocity;
        Vector3 v_ls = m_rigidBody.transform.InverseTransformDirection(v_ws);
        float speed = v_ls.magnitude;
        if (speed < m_EPS)
        {
            m_dragForce = Vector3.zero;
            return;
        }

        Vector3 dragDir_ls = -v_ls / speed;
        //Factor in area
        Vector3 areas_ls = new Vector3(m_sideArea, m_topArea, m_frontalArea);
        float areaOnDir = Mathf.Abs(Vector3.Dot(areas_ls, dragDir_ls));
        //Factor in v^2
        float dragOnDir = areaOnDir * speed * speed;
        dragOnDir *= m_dragCoefficient;
        Vector3 drag_ls = dragDir_ls * dragOnDir;
        m_dragForce = m_rigidBody.transform.TransformDirection(drag_ls);
        m_rigidBody.AddForce(m_dragForce, ForceMode.Force);
    }

    private void DevideTorqueBetweenWheels(float torque, out float torqueL, out float torqueR)
    {
        float slipRL = m_wheelHits[2].forwardSlip;
        float slipRR = m_wheelHits[3].forwardSlip;
        slipRL = Mathf.Max(0.0f, slipRL);
        slipRR = Mathf.Max(0.0f, slipRR);

        float totalSlip = slipRL + slipRR;
        // Approximate torque splitting. If one wheel is slipping more, more torque goes to the other, 
        float scaleTorqueLeft = 0.5f;
        float scaleTorqueRight = 0.5f;
        if (totalSlip > m_EPS)
        {
            float scaleLeft = slipRL / totalSlip;
            float scaleRight = slipRR / totalSlip;

            float diffScale = 0.3f;
            scaleTorqueLeft = Mathf.Lerp(scaleLeft, scaleRight, diffScale);
            scaleTorqueRight = Mathf.Lerp(scaleRight, scaleLeft, diffScale);
        }

        torqueL = torque * scaleTorqueLeft;
        torqueR = torque * scaleTorqueRight;
    }

    private void ApplyDrive()
    {
        float differentialTorque = m_driveOutputShaftTorque * m_driveShaftToDriveAxleGearRatio;

        float torqueL = 0.0f;
        float torqueR = 0.0f;
        DevideTorqueBetweenWheels(differentialTorque, out torqueL, out torqueR);

        // Rear-wheel drive
        m_rearLeft.motorTorque = torqueL;
        m_rearRight.motorTorque = torqueR;

        // Front wheels do not drive
        m_frontLeft.motorTorque = 0f;
        m_frontRight.motorTorque = 0f;
    }

    private void ApplyBrake(float brake)
    {
        m_brakeTorque = m_brakeTorqueCurve.Evaluate(brake);

        m_frontLeft.brakeTorque = m_brakeTorque;
        m_frontRight.brakeTorque = m_brakeTorque;
        m_rearLeft.brakeTorque = m_brakeTorque;
        m_rearRight.brakeTorque = m_brakeTorque;
    }

    private void UpdateWheel(GameObject wheelMesh, WheelCollider wheel)
    {
        Vector3 position;
        Quaternion rotation;
        wheel.GetWorldPose(out position, out rotation);
        wheelMesh.transform.position = position;
        wheelMesh.transform.rotation = rotation;
    }

    public void UpdateWheels()
    {
        UpdateWheel(m_frontLeftMesh, m_frontLeft);
        UpdateWheel(m_frontRightMesh, m_frontRight);
        UpdateWheel(m_rearLeftMesh, m_rearLeft);
        UpdateWheel(m_rearRightMesh, m_rearRight);
    }

    private void CalculateATGearSettings()
    {
        if(m_gears.Count <= 0)
        {
            return;
        }

        float gearUpRPM = m_engineMaxRPM * m_ATGearUpRPMScale;
        float targetRPM = m_engineMaxRPM * m_ATGearDownTargetRPMScale;

        DrivingGear gear1 = m_gears[0];
        gear1.m_ATGearUpRPM = gearUpRPM;
        gear1.m_ATGearDownRPM = -10000;

        for (int iGear = 1; iGear<m_gears.Count; ++iGear)
        {
            DrivingGear prevGear = m_gears[iGear-1];
            DrivingGear curGear = m_gears[iGear];

            float gearRatioScale = prevGear.m_gearRatio / curGear.m_gearRatio;
            float safeTargetRPM = Mathf.Min(m_engineMaxRPM, targetRPM);
            float gearDownRPM = safeTargetRPM / gearRatioScale;

            curGear.m_ATGearUpRPM = gearUpRPM;
            curGear.m_ATGearDownRPM = gearDownRPM;
        }
    }

    private float AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(float speed)
    {
        float wheelRad = 0.5f * (m_rearLeft.radius + m_rearRight.radius);
        float drivingAxleRPMThres = speed / (2 * wheelRad * Mathf.PI);
        return drivingAxleRPMThres * m_driveShaftToDriveAxleGearRatio;
    }
}
