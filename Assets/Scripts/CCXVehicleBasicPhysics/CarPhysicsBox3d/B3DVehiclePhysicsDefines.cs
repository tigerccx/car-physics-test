using Box3D.Hybrid;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace B3DVehicle
{
    [System.Serializable]
    public class WheelProperties
    {
        public float m_mass = 25.0f;
        public float m_wheelRadius = 0.5f;
        public float m_gripScale = 1.0f;
        [Description("Set to -1 to use automatic calculation from mass")]
        public float m_inertia = -1.0f;
        //TODO: Might need to distinguish this against the resistence engine and drive train provides
        public float m_extraDragTorque = 350.0f; 
        public NormalizedCurveAsset m_slipAngleToLateralForceCurve;
        public NormalizedCurveAsset m_slipRatioToGripCurve;
        public float m_staticFrictionCoeff = 0.7f;

        public float m_brakeTorque = 2000.0f;
        public bool m_allowHandbrake = false;
        public float m_handbrakeTorque = 5000.0f;
        
        public bool m_isSteerable = false;
        public bool m_isConnectedToDriveTrain = false;

        public float m_allowedAngle = 40.0f;
    };

    [System.Serializable]
    public class SuspensionProperties
    {
        public Vector3 m_posConnection_vehicle_ls = Vector3.zero;
        public Vector3 m_suspensionDirection_vehicle_ls = Vector3.up;
        public float m_suspensionHardness = 100.0f;
        public float m_suspensionDamper = 1.0f;
        public float m_suspensionLength = 0.5f;
    };

    [System.Serializable]
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

    [System.Serializable]
    public class EngineProperties
    {
        public float m_engineMaxRPM = 5000f;
        public float m_engineIdleRPM = 1000f;
        public NormalizedCurveAsset m_torqueCurve;
        public float m_engineInertia = 10.0f;
        public float m_engineInternalLoadTorque = 15.0f;

        [Header("Speed Limiter")]
        public float m_forwardSpeedLimit = -1.0f;
        public float m_backwardSpeedLimit = 11.5f;
        public float m_limiterReactionTime = 0.2f;
        public NormalizedCurveAsset m_revLimiterEngagedRPMToTorqueCurve;
        public NormalizedCurveAsset m_speedLimiterEngagedSpeedToTorqueCurve;
    }

    [System.Serializable]
    public enum TransmissionSystemType
    {
        ManualTransmission,
        AutoTransmission
    }

    [System.Serializable]
    public class GearBoxProperties
    {
        public TransmissionSystemType m_transmissionSystemType = TransmissionSystemType.ManualTransmission;

        public List<DrivingGear> m_gears = new List<DrivingGear>();
        public DrivingGear m_reverseGear = new DrivingGear();

        [Tooltip("AT will shift up gear at Scale * EngineMaxRPM")]
        public float m_ATGearUpRPMScale = 0.99f;
        [Tooltip("AT will shift down gear to meet Scale * EngineMaxRPM")]
        public float m_ATGearDownTargetRPMScale = 0.5f;
        [Tooltip("Transmission will be allowed to switch to reverse gear if speed is lower than Threshold m/s")]
        public float m_forwardToReverseSpeedThreshold = 0.1f;
        [Tooltip("Transmission will be allowed to switch to forward gear if speed is lower than Threshold m/s")]
        public float m_reverseToForwardSpeedThreshold = 0.1f;
    }

    [System.Serializable]
    public class VehicleProperties
    {
        [Header("Engine")]
        //public float m_engineInertia = 0.2f;
        public EngineProperties m_engineProperty;

        [Header("Transmission - Gear Set")]
        public GearBoxProperties m_gearBoxProperties;

        [Header("Transmission - Mechanics")]
        public float m_differentialGearRatio = 1.0f;
        //public float m_driveTrainInertia = 0.1f;

        [Header("Driving")]
        public bool m_enableABS = true;
        public float m_ABSOptimalSlipRatio = 0f;
        public NormalizedCurveAsset m_ABSSlipToTorqueScaleCurve;

        [Header("Book Keeping")]
        [ReadOnly]
        public float m_avgDrivingWheelRad = 0.0f;

        [Header("Tmp")]
        public float m_antiRollBarScale = 10.0f;
    };

    [System.Serializable]
    public class VehicleContext
    {
        public float m_surfaceFrictionCoeff = 1.0f;
        public VehicleInputs m_inputs = new VehicleInputs();
    };

    [System.Serializable]
    public class VehicleInputs
    {
        public float m_steering = 0.0f; // (-1 ~ 1)
        public float m_throttle = 0.0f; // (0 ~ 1)
        public float m_throttleIntention = 0.0f; // (0 ~ 1) //TODO: This is temp before an input manager to support AT. Remodel this. 
        public float m_brake = 0.0f; // (0 ~ 1)
        public bool m_handbrake = false; // (0 ~ 1)
        public bool m_clutchEngaged = true;
        public int m_gearChange = 0;
    }

    [System.Serializable]
    // 用于记录计算中间状态
    public class WheelTransitionalState
    {
        public Vector3 m_velocityContact_ws = Vector3.zero;
        public float m_velocityLateral = 0.0f;
        public float m_velocityLongitudinal = 0.0f;
        public float m_slipAngle = 0.0f;
        public float m_slipRatio = 0.0f;

        public float m_angularVelocity = 0.0f; //possitive = spin from back to front through top

        public Vector2 m_substepAccumImpulse = Vector2.zero;

        public float m_desiredDeltaLateralImpulse = 0.0f;
        public float m_desiredDeltaLongitudinalImpulse = 0.0f;
        public float m_maxGripImpulse = 0.0f;
        public float m_currentDeltaLateralImpulse = 0.0f;
        public float m_currentDeltaLongitudinalImpulse = 0.0f;

        public float m_accumDesiredLateralImpulse = 0.0f;
        public float m_accumDesiredLongitudinalImpulse = 0.0f;
        public float m_accumAvailableGripImpulse = 0.0f;
        public float m_accumGeneratedLateralImpulse = 0.0f;
        public float m_accumGeneratedLongitudinalImpulse = 0.0f;

        public bool m_ABSEngaged = false;
        public float m_accumBrakeTorque = 0.0f;

        public int m_subStepCount = 0;
    }

    [System.Serializable]
    public enum WheelBrakeType
    {
        Brake,
        HandBrake
    }

    [System.Serializable]
    public class WheelState
    {
        //TODO: 轮胎超过一定角度后不再提供抓地（抓地力 vs 角度 曲线？）
        public float m_currentTorque = 0.0f;
        public float m_currentBrakeTorque = 0.0f; // >=0
        public WheelBrakeType m_brakeType = WheelBrakeType.Brake;
        public float m_currentSteerAngle = 0.0f;

        public Vector3 m_currentDirFoward_ls = Vector3.zero;
        public Vector3 m_currentDirFoward_ws = Vector3.zero;
        public Vector3 m_currentDirUp_ws = Vector3.zero;
        //public Vector3 m_currentDirRight_ws = Vector3.zero;
        public Vector3 m_currentDirLateral_ws = Vector3.zero;
        public Vector3 m_currentDirLongitudinal_ws = Vector3.zero;

        //public Vector3 m_currentPos_ls = Vector3.zero;
        public Vector3 m_currentPos_ws = Vector3.zero;
        public float m_effectiveMassAtContactLateral = 0.0f;
        public float m_effectiveMassAtContactLongitudinal = 0.0f;

        public Vector3 m_currentVelocity_ws = Vector3.zero;
        public float m_currentVelocityLateral = 0;
        public float m_currentVelocityLongitudinal = 0.0f;
        public float m_currentSlipAngle = 0.0f;
        public float m_currentSlipRatio = 0.0f;

        public bool m_isGrounded = false;
        public Vector3 m_currentGroundContact_ws = Vector3.zero;

        public WheelTransitionalState m_transitional;

        public float m_desiredLateralImpulse = 0.0f;
        public float m_desiredLongitudinalImpulse = 0.0f;
        public float m_availableGripImpulse = 0.0f;
        public float m_currentLateralImpulse = 0.0f;
        public float m_currentLongitudinalImpulse = 0.0f;
        public float m_currentAngularVelocity = 0.0f; //possitive = spin from back to front through top

        public bool m_isABSEngaged = false;

        public override string ToString()
        {
            var fields = GetType()
            .GetFields(BindingFlags.Instance | BindingFlags.Public)
            .Select(f => $"{f.Name}={f.GetValue(this)}");

            return $"{GetType().Name} {{ {string.Join(", ", fields)} }}";
        }
    }

    [System.Serializable]
    public class SuspensionState
    {
        public float m_currentSuspensionVelocity = 0;
        public float m_currentSuspensionCompression = 0;

        public Vector3 m_currentVehicleAttachPoint_ws = Vector3.zero;
        public Vector3 m_currentWheelAttachPoint_ws = Vector3.zero;
        public Vector3 m_currentDir_ws = Vector3.zero;

        public bool m_isGrounded = false;
        public Vector3 m_dirGroundNormal_ws = Vector3.zero;
        public Vector3 m_posHitGround_ws = Vector3.zero;
        public float m_maxDistanceToGround = 0;
        public float m_currentSuspensionLength = 0;
        public float m_currentSuspensionDamperForce = 0;
        public float m_currentSuspensionForce = 0;

        public override string ToString()
        {
            var fields = GetType()
            .GetFields(BindingFlags.Instance | BindingFlags.Public)
            .Select(f => $"{f.Name}={f.GetValue(this)}");

            return $"{GetType().Name} {{ {string.Join(", ", fields)} }}";
        }
    };

    [System.Serializable]
    public class VehicleState
    {
        public Box3DBody m_b3dBody = null;

        public float m_measuredVehicleSpeed = 0.0f;

        //Engine
        public float m_engineAngularVelocity = 0.0f;
        public float m_engineRPM = 0.0f;
        public float m_engineOutputTorque = 0.0f;
        public float m_engineNetOutputTorque = 0.0f;

        //Limiter
        public bool m_limiterEngaged = false;
        public float m_limitEngageTimer = 0.0f;

        //Transmission - Gearbox
        public int m_idxCurrentGear = 0;
        public bool m_bClutchEngaged = true;

        //Transmission
        //public float m_differentialTorque = 0.0f;
        public float m_driveInputShaftRotationSpeed;
        public float m_driveInputShaftTorque;
        public float m_driveOutputShaftRotationSpeed;
        public float m_driveOutputShaftTorque;

        public float m_antiRollTorque = 0;
    };
}