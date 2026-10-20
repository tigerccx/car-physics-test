using Box3D;
using Box3D.Hybrid;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.Mathematics;
using UnityEngine;
using static Box3D.Hybrid.Box3DWorld;

namespace B3DVehicle
{
    public class VehiclePhysics
    {
        static float EPS = 1e-6f;

        #region Suspension
        private static void CalculateDamperForce(
            in SuspensionProperties suspensionProperties,
            ref SuspensionState suspensionState)
        {
            suspensionState.m_currentSuspensionDamperForce = -suspensionState.m_currentSuspensionVelocity * suspensionProperties.m_suspensionDamper;
        }

        public static void CalculateSuspensionForce(
            in SuspensionProperties suspensionProperties,
            ref SuspensionState suspensionState)
        {
            CalculateDamperForce(suspensionProperties, ref suspensionState);
            suspensionState.m_currentSuspensionForce = -suspensionProperties.m_suspensionHardness * suspensionState.m_currentSuspensionCompression + suspensionState.m_currentSuspensionDamperForce;
        }

        private static bool RaycastAgainstGround(
            in SuspensionState suspensionState,
            out QueryHit hit)
        {
            Vector3 rayStart = suspensionState.m_currentVehicleAttachPoint_ws;
            Vector3 rayDir = -suspensionState.m_currentDir_ws;

            int layerMaskNoVehicle = ~LayerMask.GetMask("Vehicle");

            return Box3DWorld.Instance.Query_CastRayClosest(rayStart, rayDir * suspensionState.m_maxDistanceToGround, layerMaskNoVehicle, out hit);
        }

        public static void InitSuspensionState(
            in SuspensionProperties suspensionProperties,
            ref SuspensionState suspensionState)
        {

        }

        public static void UpdateSuspensionState(
            float fixedDeltaTime,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            ref SuspensionState suspensionState)
        {
            float previousSuspensionLength = suspensionState.m_currentSuspensionLength;

            Vector3 posConnection_ws = vehicleState.m_b3dBody.transform.TransformPoint(suspensionProperties.m_posConnection_vehicle_ls);
            //TODO: 注意，suspension理论上并非永远垂直于车架，之后再复杂化
            Vector3 dirSuspension_ws = vehicleState.m_b3dBody.transform.TransformDirection(suspensionProperties.m_suspensionDirection_vehicle_ls);
            dirSuspension_ws.Normalize();
            suspensionState.m_currentVehicleAttachPoint_ws = posConnection_ws;
            suspensionState.m_currentDir_ws = dirSuspension_ws;

            // Max distance = suspension fully-relaxed length + wheel radius
            suspensionState.m_maxDistanceToGround = suspensionProperties.m_suspensionLength + wheelProperties.m_wheelRadius;

            QueryHit hit;
            suspensionState.m_isGrounded = RaycastAgainstGround(
                suspensionState,
                out hit);

            if (suspensionState.m_isGrounded)
            {
                float currentSuspensionLength = hit.distance - wheelProperties.m_wheelRadius;

                // Clamp to valid suspension travel.
                currentSuspensionLength = Mathf.Clamp(
                    currentSuspensionLength,
                    0.0f,
                    suspensionProperties.m_suspensionLength
                );

                suspensionState.m_currentSuspensionLength = currentSuspensionLength;
                suspensionState.m_currentSuspensionCompression = currentSuspensionLength - suspensionProperties.m_suspensionLength;
                suspensionState.m_dirGroundNormal_ws = hit.normal;
                suspensionState.m_posHitGround_ws = hit.point;
            }
            else
            {
                // 不在地上时默认先归零（fully-relaxed，暂不考虑轮胎重量）
                suspensionState.m_currentSuspensionCompression = 0;
                suspensionState.m_currentSuspensionLength = suspensionProperties.m_suspensionLength;
            }

            float suspensionVelocity = (suspensionState.m_currentSuspensionLength - previousSuspensionLength) / fixedDeltaTime;
            suspensionState.m_currentSuspensionVelocity = suspensionVelocity;
            suspensionState.m_currentWheelAttachPoint_ws = suspensionState.m_currentVehicleAttachPoint_ws - suspensionState.m_currentDir_ws * suspensionState.m_currentSuspensionLength;
        }

        public static bool CutOffSuspensionEndStatesIfSettled(
            ref SuspensionState suspensionState)
        {
            if (!suspensionState.m_isGrounded ||
                (
                    Mathf.Abs(suspensionState.m_currentSuspensionForce) < EPS &&
                    Mathf.Abs(suspensionState.m_currentSuspensionVelocity) < EPS &&
                    Mathf.Abs(suspensionState.m_currentSuspensionCompression) < EPS)
                )
            {
                suspensionState.m_currentSuspensionForce = 0.0f;
                suspensionState.m_currentSuspensionDamperForce = 0.0f;
                suspensionState.m_currentSuspensionVelocity = 0.0f;
                suspensionState.m_currentSuspensionCompression = 0.0f;
                return true;
            }
            return false;
        }

        public static void UpdateVehicleStateFromSuspension(
            in SuspensionState suspensionState,
            ref VehicleState vehicleState)
        {
            Vector3 force = suspensionState.m_currentDir_ws * suspensionState.m_currentSuspensionForce;
            vehicleState.m_b3dBody.ApplyForce(force, suspensionState.m_currentVehicleAttachPoint_ws, true);
        }

        public static void UpdateSuspensionAndApplyToVehicle(
            float fixedDeltaTime,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            ref SuspensionState suspensionState,
            ref VehicleState vehicleState)
        {
            UpdateSuspensionState(fixedDeltaTime, suspensionProperties, wheelProperties, vehicleState, ref suspensionState);

            CalculateSuspensionForce(suspensionProperties, ref suspensionState);

            // Eps以下记为已经settle
            if (CutOffSuspensionEndStatesIfSettled(ref suspensionState))
            {
                return;
            }

            UpdateVehicleStateFromSuspension(suspensionState, ref vehicleState);
        }
        #endregion

        #region Wheel

        private static float ComputeEffectiveMass(
            Box3DBody vehicleB3dBody,
            Vector3 wheelPosRelativeToCentreOfMass,
            Vector3 wheelRight_ws)
        {
            float invMass = 1.0f / vehicleB3dBody.mass;

            Matrix4x4 invInertiaWorld = vehicleB3dBody.GetWorldInverseRotationalInertia();

            Vector3 rCrossDir = Vector3.Cross(wheelPosRelativeToCentreOfMass, wheelRight_ws);
            Vector3 angularResponse = invInertiaWorld.MultiplyVector(rCrossDir);
            Vector3 pointVelocityResponse = Vector3.Cross(angularResponse, wheelPosRelativeToCentreOfMass);

            float angularFactor = Vector3.Dot(pointVelocityResponse, wheelRight_ws);

            float inverseEffectiveMass = invMass + angularFactor;

            if (inverseEffectiveMass <= EPS)
                return vehicleB3dBody.mass;

            return 1.0f / inverseEffectiveMass;
        }

        private static Vector3 GetVelocityAtPoint(
            in Vector3 velocity,
            in Vector3 angularVelocity,
            in Vector3 center_ws,
            in Vector3 point_ws)
        {
            Vector3 offset_ws = point_ws - center_ws;
            Vector3 predictedVelocity = velocity + Vector3.Cross(angularVelocity, offset_ws);

            return predictedVelocity;
        }


        public static void CalculateWheelLateralForce(
            ref Vector3 velocity, ref Vector3 angularVelocity,
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            //全量抵消轮胎侧向滑动所需的力
            Vector3 centerPos_ws = vehicleState.m_b3dBody.transform.position;

            Vector3 velocityContact_ws = GetVelocityAtPoint(velocity, angularVelocity, centerPos_ws, wheelState.m_currentGroundContact_ws);
            float velocityLateral = Vector3.Dot(velocityContact_ws, wheelState.m_currentDirLateral_ws);

            Vector3 wheelPosRelativeToCentreOfMass = wheelState.m_currentGroundContact_ws - vehicleState.m_b3dBody.worldCenterOfMass;
            float effectiveMass = ComputeEffectiveMass(vehicleState.m_b3dBody, wheelPosRelativeToCentreOfMass, wheelState.m_currentDirLateral_ws);
            float desiredLateralImpulse = -velocityLateral * effectiveMass;
            desiredLateralImpulse *= wheelProperties.m_laterlCounterScale;

            float load = Mathf.Max(suspensionState.m_currentSuspensionForce, 0.0f);
            float friction = wheelProperties.m_slipAngleToGripCurve.Evaluate(Mathf.Abs(wheelState.m_currentSlipAngle));
            friction = Mathf.Max(friction, 0.0f);
            friction *= wheelProperties.m_gripScale;
            friction *= context.m_surfaceFrictionCoeff;
            //TODO: 这里之后换成一个曲线
            float maxLateralImpulse = friction * load;
            maxLateralImpulse = Mathf.Max(maxLateralImpulse, 0.0f);

            float lateralImpulse = Mathf.Clamp(desiredLateralImpulse, -maxLateralImpulse, maxLateralImpulse);

            //Debug.Log($"{friction} | {desiredLateralImpulse} | {maxLateralImpulse}");

            wheelState.m_currentLateralImpulse = lateralImpulse;
        }

        public static void UpdateWheelState(
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in SuspensionProperties suspensionProperties,
            in VehicleState vehicleState,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            wheelState.m_isGrounded = suspensionState.m_isGrounded;
            wheelState.m_currentGroundContact_ws = suspensionState.m_posHitGround_ws;

            float turningAngle = wheelProperties.m_allowedAngle * context.m_inputs.m_steering;
            Vector3 dirWheelFwd_ls = Quaternion.AngleAxis(turningAngle, Vector3.up) * Vector3.forward;
            dirWheelFwd_ls.Normalize();
            Vector3 dirWheelFwd_ws = vehicleState.m_b3dBody.transform.TransformDirection(dirWheelFwd_ls);
            dirWheelFwd_ws.Normalize();
            Vector3 dirWheelFwdGround_ws = Vector3.ProjectOnPlane(dirWheelFwd_ws, suspensionState.m_dirGroundNormal_ws);
            dirWheelFwdGround_ws.Normalize();

            //Vector3 dirWheelUp_ls = suspensionProperties.m_suspensionDirection_vehicle_ls;
            //Vector3 wheelOffset_ls = -suspensionState.m_currentSuspensionLength * dirWheelUp_ls;
            //Vector3 posWheel_ls = wheelOffset_ls + suspensionProperties.m_posConnection_vehicle_ls;

            Vector3 dirWheelUp_ws = suspensionState.m_currentDir_ws;
            dirWheelUp_ws.Normalize();
            //Vector3 dirWheelRight_ws = Vector3.Cross(dirWheelFwd_ws, dirWheelUp_ws);
            //dirWheelRight_ws.Normalize();
            Vector3 dirLateral_ws = Vector3.Cross(dirWheelFwd_ws, suspensionState.m_dirGroundNormal_ws);
            dirLateral_ws.Normalize();
            Vector3 wheelVelocity_ws = vehicleState.m_b3dBody.GetPointVelocity(wheelState.m_currentGroundContact_ws);

            float wheelV_Fwd_Ground = Vector3.Dot(wheelVelocity_ws, dirWheelFwdGround_ws);
            float wheelV_Lateral_Ground = Vector3.Dot(wheelVelocity_ws, dirLateral_ws);
            //单纯算出一个偏移，无向
            float slipAngle = Mathf.Atan2(Mathf.Abs(wheelV_Lateral_Ground), Mathf.Max(Mathf.Abs(wheelV_Fwd_Ground), EPS));
            slipAngle = Mathf.Rad2Deg * slipAngle;

            wheelState.m_currentDirFoward_ls = dirWheelFwd_ls;
            wheelState.m_currentDirFoward_ws = dirWheelFwd_ws;
            wheelState.m_currentDirUp_ws = dirWheelUp_ws;
            //wheelState.m_currentDirRight_ws = dirWheelRight_ws;
            wheelState.m_currentDirLateral_ws = dirLateral_ws;
            wheelState.m_currentPos_ws = suspensionState.m_currentWheelAttachPoint_ws;
            wheelState.m_currentVelocity_ws = wheelVelocity_ws;
            wheelState.m_currentSlipAngle = slipAngle;
        }

        public static void UpdateVehicleStateFromWheel(
            in WheelState wheelState,
            ref VehicleState vehicleState)
        {
            Vector3 lateralImpulse = wheelState.m_currentDirLateral_ws * wheelState.m_currentLateralImpulse;
            vehicleState.m_b3dBody.AddForceAtPosition(lateralImpulse, wheelState.m_currentPos_ws, ForceMode.Impulse, false);
        }

        public static void UpdateWheelAndApplyToVehicle_Main(
            float fixedDeltaTime,
            in VehicleContext context,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in SuspensionState suspensionState,
            ref WheelState wheelState,
            ref VehicleState vehicleState)
        {
            UpdateWheelState(
                context,
                wheelProperties,
                suspensionProperties,
                vehicleState,
                suspensionState,
                ref wheelState);

            if (!wheelState.m_isGrounded)
            {
                wheelState.m_currentLateralImpulse = 0.0f;
                return;
            }
        }

        public static void UpdateWheelAndApplyToVehicle_SubStep_PrePosIntegration(
            in Vector3 deltaPosition, in Quaternion deltaRotation, float subStepDeltaTime,
            in VehicleContext context,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in SuspensionState suspensionState,
            ref WheelState wheelState,
            ref VehicleState vehicleState)
        {
            Vector3 velocity = vehicleState.m_b3dBody.linearVelocity;
            Vector3 angularVelocity = vehicleState.m_b3dBody.angularVelocity;
            CalculateWheelLateralForce(
                ref velocity, ref angularVelocity, 
                context, wheelProperties, vehicleState, suspensionState, ref wheelState);

            if (Mathf.Abs(wheelState.m_currentLateralImpulse) < EPS)
            {
                wheelState.m_currentLateralImpulse = 0.0f;
                return;
            }

            UpdateVehicleStateFromWheel(wheelState, ref vehicleState);
        }
        #endregion

        public static void ApplyAntiRollBar(
            in VehicleProperties vehicleProperties,
            ref VehicleState vehicleState)
        {
            Box3DBody b3dBody = vehicleState.m_b3dBody;
            Transform transform = b3dBody.transform;
            Vector3 angularVelocity_ls = transform.InverseTransformDirection(b3dBody.angularVelocity);
            float rollSpeed = angularVelocity_ls.z;
            float antiRollTorque = -rollSpeed * vehicleProperties.m_antiRollBarScale;
            vehicleState.m_antiRollTorque = antiRollTorque;

            b3dBody.AddRelativeTorque(new Vector3(0, 0, vehicleState.m_antiRollTorque));
        }

        public static void UpdateVehicle(
            in VehicleProperties vehicleProperties,
            ref VehicleState vehicleState)
        {
            ApplyAntiRollBar(vehicleProperties, ref vehicleState);
        }
    }

}