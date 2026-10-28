using Box3D;
using Box3D.Hybrid;
using CCXUnityVehicle;
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

        #region Utility
        private static float ComputeEffectiveMass(
            Box3DBody vehicleB3dBody,
            Vector3 posRelativeToCentreOfMass_ws,
            Vector3 intendedImpulseDirection_ws)
        {
            float invMass = 1.0f / vehicleB3dBody.mass;

            Matrix4x4 invInertiaWorld = vehicleB3dBody.GetWorldInverseRotationalInertia();

            Vector3 rCrossDir = Vector3.Cross(posRelativeToCentreOfMass_ws, intendedImpulseDirection_ws);
            Vector3 angularResponse = invInertiaWorld.MultiplyVector(rCrossDir);
            Vector3 pointVelocityResponse = Vector3.Cross(angularResponse, posRelativeToCentreOfMass_ws);

            float angularFactor = Vector3.Dot(pointVelocityResponse, intendedImpulseDirection_ws);

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
            Vector3 velocityAtPoint = velocity + Vector3.Cross(angularVelocity, offset_ws);

            return velocityAtPoint;
        }
        #endregion

        #region Suspension
        private static bool CutOffSuspensionEndStatesIfSettled(
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

            CutOffSuspensionEndStatesIfSettled(ref suspensionState);
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

        public static void UpdateSuspensionStateAndForce(
            float fixedDeltaTime,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            ref SuspensionState suspensionState)
        {
            UpdateSuspensionState(fixedDeltaTime, suspensionProperties, wheelProperties, vehicleState, ref suspensionState);

            CalculateSuspensionForce(suspensionProperties, ref suspensionState);
        }

        public static void UpdateVehicleStateFromSuspension(
            in SuspensionState suspensionState,
            ref VehicleState vehicleState)
        {
            Vector3 force = suspensionState.m_currentDir_ws * suspensionState.m_currentSuspensionForce;
            vehicleState.m_b3dBody.ApplyForce(force, suspensionState.m_currentVehicleAttachPoint_ws, true);
        }
        #endregion

        #region Wheel
        private static float CalculateSlipRatio(float wheelV, float groundV)
        {
            float slipRatio = (wheelV - groundV) / (Mathf.Sign(groundV) * Mathf.Max(Mathf.Abs(groundV), EPS));
            // =0.0 -> no slip;
            // <0.0 -> vehicle faster than wheel. Backward ground friction. (speed up wheel)
            // >0.0 -> vehicle slower than wheel. Forward ground friction. (slow down wheel)
            return slipRatio;
        }

        private static void InitWheelTransitionalStates(ref WheelState wheelState)
        {
            wheelState.m_transitional.m_velocityContact_ws = Vector3.zero;
            wheelState.m_transitional.m_velocityLateral = 0.0f;
            wheelState.m_transitional.m_velocityLongitudinal = 0.0f;
            wheelState.m_transitional.m_slipAngle = 0.0f;
            wheelState.m_transitional.m_slipRatio = 0.0f;

            wheelState.m_transitional.m_angularVelocity = wheelState.m_currentAngularVelocity;

            wheelState.m_transitional.m_substepAccumImpulse = Vector2.zero;

            wheelState.m_transitional.m_desiredDeltaLateralImpulse = 0.0f;
            wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse = 0.0f;
            wheelState.m_transitional.m_maxGripImpulse = 0.0f;
            wheelState.m_transitional.m_currentDeltaLateralImpulse = 0.0f;
            wheelState.m_transitional.m_currentDeltaLongitudinalImpulse = 0.0f;

            wheelState.m_transitional.m_accumDesiredLateralImpulse = 0.0f;
            wheelState.m_transitional.m_accumDesiredLongitudinalImpulse = 0.0f;
            wheelState.m_transitional.m_accumAvailableGripImpulse = 0.0f;
            wheelState.m_transitional.m_accumGeneratedLateralImpulse = 0.0f;
            wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse = 0.0f;
        }

        private static void IntegrateWheelTransitionalStates(in WheelProperties wheelProperties, ref WheelState wheelState)
        {
            wheelState.m_desiredLateralImpulse = wheelState.m_transitional.m_accumDesiredLateralImpulse;
            wheelState.m_desiredLongitudinalImpulse = wheelState.m_transitional.m_accumDesiredLongitudinalImpulse;

            wheelState.m_availableGripImpulse = wheelState.m_transitional.m_accumAvailableGripImpulse;

            wheelState.m_currentLateralImpulse = wheelState.m_transitional.m_accumGeneratedLateralImpulse;
            wheelState.m_currentLongitudinalImpulse = wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse;

            wheelState.m_currentAngularVelocity = wheelState.m_transitional.m_angularVelocity;
        }

        private static void UpdateWheelEffectiveMassAtContact(
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            //计算等效质量
            Vector3 wheelPosRelativeToCentreOfMass = wheelState.m_currentGroundContact_ws - vehicleState.m_b3dBody.worldCenterOfMass;
            float effectiveMassLateral = ComputeEffectiveMass(vehicleState.m_b3dBody, wheelPosRelativeToCentreOfMass, wheelState.m_currentDirLateral_ws);
            float effectiveMassLongitudinal = ComputeEffectiveMass(vehicleState.m_b3dBody, wheelPosRelativeToCentreOfMass, wheelState.m_currentDirLongitudinal_ws);

            wheelState.m_effectiveMassAtContactLateral = effectiveMassLateral;
            wheelState.m_effectiveMassAtContactLongitudinal = effectiveMassLongitudinal;
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

            float turningAngle = wheelState.m_currentSteerAngle;
            Vector3 dirWheelFwd_ls = Quaternion.AngleAxis(turningAngle, Vector3.up) * Vector3.forward;
            dirWheelFwd_ls.Normalize();
            Vector3 dirWheelFwd_ws = vehicleState.m_b3dBody.transform.TransformDirection(dirWheelFwd_ls);
            dirWheelFwd_ws.Normalize();
            Vector3 dirWheelFwdGround_ws = Vector3.ProjectOnPlane(dirWheelFwd_ws, suspensionState.m_dirGroundNormal_ws);
            dirWheelFwdGround_ws.Normalize();

            Vector3 dirWheelUp_ws = suspensionState.m_currentDir_ws;
            dirWheelUp_ws.Normalize();
            Vector3 dirLateral_ws = Vector3.Cross(dirWheelFwd_ws, suspensionState.m_dirGroundNormal_ws);
            dirLateral_ws.Normalize();
            Vector3 wheelVelocity_ws = vehicleState.m_b3dBody.GetPointVelocity(wheelState.m_currentGroundContact_ws);

            float wheelV_Fwd_Ground = Vector3.Dot(wheelVelocity_ws, dirWheelFwdGround_ws);
            float wheelV_Lateral_Ground = Vector3.Dot(wheelVelocity_ws, dirLateral_ws);
            //单纯算出一个偏移，无向
            float slipAngle = Mathf.Atan2(Mathf.Abs(wheelV_Lateral_Ground), Mathf.Max(Mathf.Abs(wheelV_Fwd_Ground), EPS));
            slipAngle = Mathf.Rad2Deg * slipAngle;

            float wheelLinearV = wheelState.m_currentAngularVelocity * wheelProperties.m_wheelRadius;
            float slipRatio = CalculateSlipRatio(wheelLinearV, wheelV_Fwd_Ground);

            wheelState.m_currentDirFoward_ls = dirWheelFwd_ls;
            wheelState.m_currentDirFoward_ws = dirWheelFwd_ws;
            wheelState.m_currentDirUp_ws = dirWheelUp_ws;
            wheelState.m_currentDirLateral_ws = dirLateral_ws;
            wheelState.m_currentDirLongitudinal_ws = dirWheelFwdGround_ws;
            wheelState.m_currentPos_ws = suspensionState.m_currentWheelAttachPoint_ws;
            wheelState.m_currentVelocity_ws = wheelVelocity_ws;
            wheelState.m_currentVelocityLateral = wheelV_Lateral_Ground;
            wheelState.m_currentVelocityLongitudinal = wheelV_Fwd_Ground;
            wheelState.m_currentSlipAngle = slipAngle;
            wheelState.m_currentSlipRatio = slipRatio;

            UpdateWheelEffectiveMassAtContact(vehicleState, ref wheelState);
        }

        public static void PrepareWheelSubsteps(
            float fixedDeltaTime,
            in VehicleContext context,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in SuspensionState suspensionState,
            ref WheelState wheelState,
            ref VehicleState vehicleState)
        {
            InitWheelTransitionalStates(ref wheelState);
        }

        private static void CalculateWheelTransitionalVelocityStates(
            in WheelProperties wheelProperty,
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            Vector3 centerPos_ws = vehicleState.m_b3dBody.transform.position;

            Vector3 velocity = vehicleState.m_b3dBody.velocity;
            Vector3 angularVelocity = vehicleState.m_b3dBody.angularVelocity;

            Vector3 velocityContact_ws = GetVelocityAtPoint(velocity, angularVelocity, centerPos_ws, wheelState.m_currentGroundContact_ws);
            float velocityLateral = Vector3.Dot(velocityContact_ws, wheelState.m_currentDirLateral_ws);
            float velocityLongitudinal = Vector3.Dot(velocityContact_ws, wheelState.m_currentDirLongitudinal_ws);

            //Re-calculate slip
            float slipAngle = Mathf.Atan2(Mathf.Abs(velocityLateral), Mathf.Max(Mathf.Abs(velocityLongitudinal), EPS));
            slipAngle = Mathf.Rad2Deg * slipAngle;

            float wheelLinearV = wheelState.m_transitional.m_angularVelocity * wheelProperty.m_wheelRadius;
            float slipRatio = CalculateSlipRatio(wheelLinearV, velocityLongitudinal);

            wheelState.m_transitional.m_velocityContact_ws = velocityContact_ws;
            wheelState.m_transitional.m_velocityLateral = velocityLateral;
            wheelState.m_transitional.m_velocityLongitudinal = velocityLongitudinal;
            wheelState.m_transitional.m_slipAngle = slipAngle;
            wheelState.m_transitional.m_slipRatio = slipRatio;
        }

        private static float CalculateWheelLateralResponseForce(
            float deltaTime,
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            float responseImpulse = 0.0f;
            float load = Mathf.Max(suspensionState.m_currentSuspensionForce, 0.0f);

            if (wheelProperties.m_slipAngleToLateralForceCurve)
            {
                responseImpulse = wheelProperties.m_slipAngleToLateralForceCurve.Evaluate(Mathf.Abs(wheelState.m_transitional.m_slipAngle));
                responseImpulse = Mathf.Max(0, responseImpulse);
                responseImpulse *= load;
                responseImpulse *= wheelProperties.m_staticFrictionCoeff;
                responseImpulse *= context.m_surfaceFrictionCoeff;
                responseImpulse *= deltaTime;
            }

            return responseImpulse;
        }

        private static void CalculateWheelDesiredLateralCounterSlipForce(
            float deltaTime,
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            //Impulse required to eliminate all lateral motion. This is calculated as a constraint. 
            float velocityLateral = wheelState.m_transitional.m_velocityLateral;
            float dirCounterLateral = -Mathf.Sign(velocityLateral);
            float effectiveMass = wheelState.m_effectiveMassAtContactLateral;
            float lateralImpulseConstraint = Mathf.Abs(velocityLateral) * effectiveMass;
            lateralImpulseConstraint = Mathf.Max(0.0f, lateralImpulseConstraint);

            float responseImpulse = CalculateWheelLateralResponseForce(
                deltaTime, context, wheelProperties, vehicleState, suspensionState,
                ref wheelState);
            responseImpulse = Mathf.Max(0.0f, responseImpulse);

            float desiredLateralImpulse = dirCounterLateral * Mathf.Min(lateralImpulseConstraint, responseImpulse);

            wheelState.m_transitional.m_desiredDeltaLateralImpulse = desiredLateralImpulse;
        }

        private static void CalculateWheelDesiredLongitudinalCounterSlipForce(
            in VehicleContext context,
            in WheelProperties wheelProperty,
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            float contactPointVelocity = wheelState.m_transitional.m_velocityLongitudinal;
            float wheelLinearVelocity = wheelState.m_transitional.m_angularVelocity * wheelProperty.m_wheelRadius;
            float effectiveMassAtContact = wheelState.m_effectiveMassAtContactLongitudinal;
            float radWheel = wheelProperty.m_wheelRadius;
            float wheelInertia = wheelProperty.m_inertia;

            float forwardSlipVelocity = wheelLinearVelocity - contactPointVelocity; //How much wheel is faster than vehicle. 
            //阻止Slip所需的impulse
            float impulseRequiredToCounterSlip = forwardSlipVelocity / (1 / effectiveMassAtContact + radWheel * radWheel / wheelInertia);

            if(wheelProperty.m_slipRatioToGripCurve)
            {
                float scale = wheelProperty.m_slipRatioToGripCurve.Evaluate(Mathf.Abs(wheelState.m_transitional.m_slipRatio));
                scale = Mathf.Max(0, scale);
                impulseRequiredToCounterSlip *= scale;
            }

            wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse = impulseRequiredToCounterSlip;
        }

        private static void CalculateMaxGrip(
            float subStepDeltaTime,
            in VehicleContext context,
            in WheelProperties wheelProperty,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            //计算可用的Grip
            float load = Mathf.Max(suspensionState.m_currentSuspensionForce, 0.0f);

            float friction = load;
            friction *= wheelProperty.m_gripScale;
            friction *= context.m_surfaceFrictionCoeff;
            friction *= wheelProperty.m_staticFrictionCoeff;
            float maxGripImpulse = friction * subStepDeltaTime;

            //TODO: 这里之后换成一个曲线。Load和Friction不是线性关系
            maxGripImpulse = Mathf.Max(maxGripImpulse, 0.0f);

            wheelState.m_transitional.m_maxGripImpulse = maxGripImpulse;
        }

        public static void UpdateWheelSubstep(
            in Vector3 deltaPosition, in Quaternion deltaRotation,
            float subStepDeltaTime,
            in VehicleContext context,
            int wheelCount, int iterCount,
            in List<Suspension> suspensions,
            in List<Wheel> wheels,
            ref VehicleState vehicleState)
        {
            // Initialize for iterations
            for (int i = 0; i < wheelCount; ++i)
            {
                // Spin the wheel
                WheelState wheelState = wheels[i].m_wheelState;
                WheelProperties wheelProperty = wheels[i].m_wheelProperties;
                SuspensionState suspensionState = suspensions[i].m_suspensionState;

                //Update wheel torque
                float angularImpulse = wheelState.m_currentTorque * subStepDeltaTime;
                wheelState.m_transitional.m_angularVelocity += angularImpulse / wheelProperty.m_inertia;

                wheelState.m_transitional.m_substepAccumImpulse = Vector2.zero;

                CalculateMaxGrip(subStepDeltaTime, context, wheelProperty, suspensionState, ref wheelState);
            }

            //Debug.Log($"----------- STEP -------------");
            for (int iter = 0; iter < iterCount; ++iter)
            {
                //Debug.Log($"Iter{iter} -------------");

                for (int i = 0; i < wheelCount; ++i)
                {
                    WheelState wheelState = wheels[i].m_wheelState;
                    WheelProperties wheelProperty = wheels[i].m_wheelProperties;
                    SuspensionState suspensionState = suspensions[i].m_suspensionState;

                    if (!wheelState.m_isGrounded)
                    {
                        wheelState.m_transitional.m_velocityContact_ws = Vector3.zero;
                        wheelState.m_transitional.m_velocityLateral = 0.0f;
                        wheelState.m_transitional.m_velocityLongitudinal = 0.0f;
                        wheelState.m_transitional.m_slipAngle = 0.0f;
                        wheelState.m_transitional.m_slipRatio = 0.0f;

                        wheelState.m_transitional.m_desiredDeltaLateralImpulse = 0.0f;
                        wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse = 0.0f;

                        wheelState.m_transitional.m_currentDeltaLateralImpulse = 0.0f;
                        wheelState.m_transitional.m_currentDeltaLongitudinalImpulse = 0.0f;
                    }
                    else
                    {
                        CalculateWheelTransitionalVelocityStates(wheelProperty, vehicleState, ref wheelState);
                        CalculateWheelDesiredLateralCounterSlipForce(
                            subStepDeltaTime, context, wheelProperty, vehicleState, suspensionState, ref wheelState);
                        CalculateWheelDesiredLongitudinalCounterSlipForce(context, wheelProperty, vehicleState, ref wheelState);

                        // Clamp desired impulse
                        Vector2 newAccumImpulse = wheelState.m_transitional.m_substepAccumImpulse + new Vector2(
                            wheelState.m_transitional.m_desiredDeltaLateralImpulse, 
                            wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse);
                        newAccumImpulse = Vector2.ClampMagnitude(newAccumImpulse, wheelState.m_transitional.m_maxGripImpulse);

                        Vector2 clampedCorrectionImpulse = newAccumImpulse - wheelState.m_transitional.m_substepAccumImpulse;
                        float deltaLateralImpulse = clampedCorrectionImpulse.x;
                        float deltaLongitudinalImpulse = clampedCorrectionImpulse.y;
                        wheelState.m_transitional.m_substepAccumImpulse = newAccumImpulse;

                        // Apply impulses
                        Vector3 lateralImpulse = wheelState.m_currentDirLateral_ws * deltaLateralImpulse;
                        vehicleState.m_b3dBody.AddForceAtPosition(lateralImpulse, wheelState.m_currentPos_ws, ForceMode.Impulse, false);

                        Vector3 longitudinalImpulse = wheelState.m_currentDirLongitudinal_ws * deltaLongitudinalImpulse;
                        vehicleState.m_b3dBody.AddForceAtPosition(longitudinalImpulse, wheelState.m_currentPos_ws, ForceMode.Impulse, false);

                        // Slowdown wheel
                        wheelState.m_transitional.m_angularVelocity -= deltaLongitudinalImpulse * wheelProperty.m_wheelRadius / wheelProperty.m_inertia;

                        
                        // Accumulations
                        wheelState.m_transitional.m_accumDesiredLateralImpulse += wheelState.m_transitional.m_desiredDeltaLateralImpulse;
                        wheelState.m_transitional.m_accumDesiredLongitudinalImpulse += wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse;

                        wheelState.m_transitional.m_accumGeneratedLateralImpulse += deltaLateralImpulse;
                        wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse += deltaLongitudinalImpulse;
                    }
                }
            }

            // Accumulations
            for (int i = 0; i < wheelCount; ++i)
            {
                WheelState wheelState = wheels[i].m_wheelState;
                wheelState.m_transitional.m_accumAvailableGripImpulse += wheelState.m_transitional.m_maxGripImpulse;
            }
        }

        public static void FinishWheelSubsteps(
            float fixedDeltaTime,
            in VehicleContext context,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in SuspensionState suspensionState,
            ref WheelState wheelState,
            ref VehicleState vehicleState)
        {
            IntegrateWheelTransitionalStates(wheelProperties, ref wheelState);
        }
        #endregion

        #region Vehicle
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
        #endregion
    }

}