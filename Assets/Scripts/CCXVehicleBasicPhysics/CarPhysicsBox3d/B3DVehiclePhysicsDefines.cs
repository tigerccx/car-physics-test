using Box3D.Hybrid;
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
        //因为轮胎非刚体，所以其最大静态摩擦力与压力的关系并非定值系数，而是曲线
        public NormalizedCurveAsset m_slipAngleToGripCurve;
        public float m_staticFrictionCoeff = 0.7f;

        //public float m_dynamicFrictionScale = 0.9f;

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
    public class VehicleProperties
    {
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
                                        //brake
    }

    [System.Serializable]
    // 用于记录计算中间状态
    public class WheelTransitionalState
    {
        public Vector3 m_velocityContact_ws = Vector3.zero;
        public float m_velocityLateral = 0.0f;
        public float m_velocityLongitudinal = 0.0f;
        public float m_angularVelocity = 0.0f; //possitive = spin from back to front through top

        public float m_desiredLateralImpulse = 0.0f;
        public float m_desiredLongitudinalImpulse = 0.0f;
        
        public float m_availableGripImpulse = 0.0f;

        public float m_currentLateralImpulse = 0.0f;
        public float m_currentLongitudinalImpulse = 0.0f;

        // 目前先粗略平均一下，以近似这一帧的Lateral和Longitudinal Impulse情况
        public float m_accumDesiredLateralImpulse = 0.0f;
        public float m_accumDesiredLongitudinalImpulse = 0.0f;
        public float m_accumAvailableGripImpulse = 0.0f;
        public float m_accumGeneratedLateralImpulse = 0.0f;
        public float m_accumGeneratedLongitudinalImpulse = 0.0f;
        public float m_accumDT = 0.0f;
    }

    [System.Serializable]
    public class WheelState
    {
        //TODO: 轮胎超过一定角度后不再提供抓地（抓地力 vs 角度 曲线？）
        public float m_currentTorque = 0.0f;
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
        public float m_antiRollTorque = 0;
    };
}