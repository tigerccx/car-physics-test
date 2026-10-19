using Box3D.Hybrid;
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
        //因为轮胎非刚体，所以其最大静态摩擦力与压力的关系并非定值系数，而是曲线
        public NormalizedCurveAsset m_slipAngleToGripCurve;
        public float m_staticFrictionCoeff = 0.7f;
        public float m_laterlCounterScale = 0.9f;

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
                                        //steering
                                        //gas
                                        //brake
    }

    [System.Serializable]
    public class WheelState
    {
        //TODO: 轮胎超过一定角度后不再提供抓地（抓地力 vs 角度 曲线？）

        public Vector3 m_currentDirFoward_ls = Vector3.zero;
        public Vector3 m_currentDirFoward_ws = Vector3.zero;
        public Vector3 m_currentDirUp_ws = Vector3.zero;
        //public Vector3 m_currentDirRight_ws = Vector3.zero;
        public Vector3 m_currentDirLateral_ws = Vector3.zero;

        //public Vector3 m_currentPos_ls = Vector3.zero;
        public Vector3 m_currentPos_ws = Vector3.zero;

        public Vector3 m_currentVelocity_ws = Vector3.zero;
        public float m_lastVelocityLateral = 0;
        public float m_currentVelocityLateral = 0;
        public float m_currentSlipAngle = 0.0f;

        public bool m_isGrounded = false;
        public Vector3 m_currentGroundContact_ws = Vector3.zero;

        public float m_currentLateralImpulse = 0.0f;

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