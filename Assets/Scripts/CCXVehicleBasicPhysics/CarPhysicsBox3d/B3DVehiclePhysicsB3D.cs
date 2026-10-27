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
            Vector3 wheelPosRelativeToCentreOfMass_ws,
            Vector3 intendedImpulseDirection_ws)
        {
            float invMass = 1.0f / vehicleB3dBody.mass;

            Matrix4x4 invInertiaWorld = vehicleB3dBody.GetWorldInverseRotationalInertia();

            Vector3 rCrossDir = Vector3.Cross(wheelPosRelativeToCentreOfMass_ws, intendedImpulseDirection_ws);
            Vector3 angularResponse = invInertiaWorld.MultiplyVector(rCrossDir);
            Vector3 pointVelocityResponse = Vector3.Cross(angularResponse, wheelPosRelativeToCentreOfMass_ws);

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

        private static float CalculateSlipRatio(float wheelV, float groundV)
        {
            float slipRatio = (wheelV - groundV) / (Mathf.Sign(groundV) * Mathf.Max(Mathf.Abs(groundV), EPS));
            // =0.0 -> no slip;
            // <0.0 -> vehicle faster than wheel. Backward ground friction. (speed up wheel)
            // >0.0 -> vehicle slower than wheel. Forward ground friction. (slow down wheel)
            return slipRatio;
        }

        public static void InitWheelTransitionalStates(ref WheelState wheelState)
        {
            wheelState.m_transitional.m_velocityContact_ws = Vector3.zero;
            wheelState.m_transitional.m_velocityLateral = 0.0f;
            wheelState.m_transitional.m_velocityLongitudinal = 0.0f;
            wheelState.m_transitional.m_angularVelocity = wheelState.m_currentAngularVelocity;
            
            wheelState.m_transitional.m_desiredLateralImpulse = 0.0f;
            wheelState.m_transitional.m_desiredLongitudinalImpulse = 0.0f;

            wheelState.m_transitional.m_currentLateralImpulse = 0.0f;
            wheelState.m_transitional.m_currentLongitudinalImpulse = 0.0f;

            wheelState.m_transitional.m_accumDesiredLateralImpulse = 0.0f;
            wheelState.m_transitional.m_accumDesiredLongitudinalImpulse = 0.0f;
            wheelState.m_transitional.m_accumAvailableGripImpulse = 0.0f;
            wheelState.m_transitional.m_accumGeneratedLateralImpulse = 0.0f;
            wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse = 0.0f;
            wheelState.m_transitional.m_accumDT = 0.0f;
        }

        public static void CalculateWheelTransitionalVelocityStates(
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            Vector3 centerPos_ws = vehicleState.m_b3dBody.transform.position;

            Vector3 velocity = vehicleState.m_b3dBody.velocity;
            Vector3 angularVelocity = vehicleState.m_b3dBody.angularVelocity;

            Vector3 velocityContact_ws = GetVelocityAtPoint(velocity, angularVelocity, centerPos_ws, wheelState.m_currentGroundContact_ws);
            float velocityLateral = Vector3.Dot(velocityContact_ws, wheelState.m_currentDirLateral_ws);
            float velocityLongitudinal = Vector3.Dot(velocityContact_ws, wheelState.m_currentDirLongitudinal_ws);

            wheelState.m_transitional.m_velocityContact_ws = velocityContact_ws;
            wheelState.m_transitional.m_velocityLateral = velocityLateral;
            wheelState.m_transitional.m_velocityLongitudinal = velocityLongitudinal;
        }

        public static void CalculateWheelDesiredLateralForce(
            float subStepDeltaTime,
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            //全量抵消轮胎侧向滑动所需的力
            float velocityLateral = wheelState.m_transitional.m_velocityLateral;

            Vector3 wheelPosRelativeToCentreOfMass = wheelState.m_currentGroundContact_ws - vehicleState.m_b3dBody.worldCenterOfMass;
            float effectiveMass = wheelState.m_effectiveMassAtContactLateral;
            float desiredLateralImpulse = -velocityLateral * effectiveMass;

            wheelState.m_transitional.m_desiredLateralImpulse = desiredLateralImpulse;
        }

        public static void CalculateWheelDesiredLongitudinalForce(
            float subStepDeltaTime, 
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            ref WheelState wheelState)
        {
            float contactPointVelocity = wheelState.m_transitional.m_velocityLongitudinal;
            float wheelLinearVelocity = wheelState.m_transitional.m_angularVelocity * wheelProperties.m_wheelRadius;
            float effectiveMassAtContact = wheelState.m_effectiveMassAtContactLongitudinal;
            float angularImpulse = wheelState.m_currentTorque * subStepDeltaTime;
            float radWheel = wheelProperties.m_wheelRadius;
            float wheelInertia = wheelProperties.m_inertia;

            //float slipRatio = CalculateSlipRatio(wheelLinearVelocity, contactPointVelocity);

            float forwardSlipVelocity = wheelLinearVelocity - contactPointVelocity; //How much wheel is faster than vehicle. 
            //阻止Slip所需的impulse
            float impulseRequiredToCounterSlip = forwardSlipVelocity / (1 / effectiveMassAtContact + radWheel * radWheel / wheelInertia);

            //Non-slip情况下，维持non-slip所需要的impulse
            float impulseDrive = angularImpulse / (radWheel + (wheelInertia / effectiveMassAtContact));

            //两者合并为non slip所需的impulse
            float desiredLongitudinalImpulse = impulseDrive + impulseRequiredToCounterSlip;

            wheelState.m_transitional.m_desiredLongitudinalImpulse = desiredLongitudinalImpulse;

            //Debug.Log($"slipRatio{slipRatio}(whl{wheelLinearVelocity} vs grnd{contactPointVelocity}), slipVelocity{forwardSlipVelocity}, impulseCntSlip{impulseRequiredToCounterSlip}, impulseDrive{impulseDrive}");
        }

        public static void CalculateWheelLateralAndLongitudinalForce(
            float subStepDeltaTime,
            in VehicleContext context,
            in WheelProperties wheelProperties,
            in VehicleState vehicleState,
            in SuspensionState suspensionState,
            ref WheelState wheelState)
        {
            float desiredLateralImpulse = wheelState.m_transitional.m_desiredLateralImpulse;
            float desiredLongitudinalImpulse = wheelState.m_transitional.m_desiredLongitudinalImpulse;

            float load = Mathf.Max(suspensionState.m_currentSuspensionForce, 0.0f);
            float friction = wheelProperties.m_slipAngleToGripCurve.Evaluate(Mathf.Abs(wheelState.m_currentSlipAngle));
            friction = Mathf.Max(friction, 0.0f);
            friction *= wheelProperties.m_gripScale;
            friction *= context.m_surfaceFrictionCoeff;
            //TODO: 这里之后换成一个曲线。Load和Friction不是线性关系
            float maxGripImpulse = friction * load * subStepDeltaTime;
            maxGripImpulse = Mathf.Max(maxGripImpulse, 0.0f);

            //Clamp impulses by available friction circle
            float lateralImpulse = 0.0f;
            float longitudinalImpulse = 0.0f;
            Vector2 desiredImpulses = new Vector2(desiredLateralImpulse, desiredLongitudinalImpulse);
            if (desiredImpulses.sqrMagnitude <= maxGripImpulse * maxGripImpulse) //Enough grip for everything! 
            {
                //Lateral and Longitudinal stays the same
                lateralImpulse = desiredImpulses.x;
                longitudinalImpulse = desiredImpulses.y;
            }
            else //Not enough grip... 
            {
                desiredImpulses.Normalize();
                lateralImpulse = desiredImpulses.x * maxGripImpulse;
                longitudinalImpulse = desiredImpulses.y * maxGripImpulse;
            }

            wheelState.m_transitional.m_currentLateralImpulse = lateralImpulse;
            wheelState.m_transitional.m_currentLongitudinalImpulse = longitudinalImpulse;
            wheelState.m_transitional.m_availableGripImpulse = maxGripImpulse;
        }

        public static void IntegrateWheelTransitionalStates(in WheelProperties wheelProperties, ref WheelState wheelState)
        {
            wheelState.m_desiredLateralImpulse = wheelState.m_transitional.m_accumDesiredLateralImpulse / wheelState.m_transitional.m_accumDT;
            wheelState.m_desiredLongitudinalImpulse = wheelState.m_transitional.m_accumDesiredLongitudinalImpulse / wheelState.m_transitional.m_accumDT;

            wheelState.m_availableGripImpulse = wheelState.m_transitional.m_accumAvailableGripImpulse / wheelState.m_transitional.m_accumDT;

            wheelState.m_currentLateralImpulse = wheelState.m_transitional.m_accumGeneratedLateralImpulse / wheelState.m_transitional.m_accumDT;
            wheelState.m_currentLongitudinalImpulse = wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse / wheelState.m_transitional.m_accumDT;

            wheelState.m_currentAngularVelocity = wheelState.m_transitional.m_angularVelocity;
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

            Vector3 wheelVelocityLateral_ws = wheelV_Lateral_Ground * dirLateral_ws;

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

        public static void UpdateWheelEffectiveMassAtContact(
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

        public static void UpdateWheelAngularVelocityFromLateralAndLongitudinalForce(
            float subStepDeltaTime,
            in WheelProperties wheelProperties,
            ref WheelState wheelState)
        {
            float angularImpulse = wheelState.m_currentTorque * subStepDeltaTime;
            wheelState.m_transitional.m_angularVelocity += angularImpulse / wheelProperties.m_inertia;
            wheelState.m_transitional.m_angularVelocity -= wheelState.m_transitional.m_currentLongitudinalImpulse * wheelProperties.m_wheelRadius / wheelProperties.m_inertia;
        }

        public static void UpdateVehicleStateFromWheel(
            in WheelState wheelState,
            ref VehicleState vehicleState)
        {
            if (Mathf.Abs(wheelState.m_transitional.m_currentLateralImpulse) >= EPS)
            {
                Vector3 lateralImpulse = wheelState.m_currentDirLateral_ws * wheelState.m_transitional.m_currentLateralImpulse;
                vehicleState.m_b3dBody.AddForceAtPosition(lateralImpulse, wheelState.m_currentPos_ws, ForceMode.Impulse, false);
            }
            if (Mathf.Abs(wheelState.m_transitional.m_currentLongitudinalImpulse) >= EPS)
            {
                Vector3 longitudinalImpulse = wheelState.m_currentDirLongitudinal_ws * wheelState.m_transitional.m_currentLongitudinalImpulse;
                vehicleState.m_b3dBody.AddForceAtPosition(longitudinalImpulse, wheelState.m_currentPos_ws, ForceMode.Impulse, false);
            }
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
        }

        public static void UpdateWheelAndApplyToVehicle_PrePhysics(
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

        public static void UpdateWheelAndApplyToVehicle_SubStep_PrePosIntegration(
            in Vector3 deltaPosition, in Quaternion deltaRotation, float subStepDeltaTime,
            in VehicleContext context,
            in SuspensionProperties suspensionProperties,
            in WheelProperties wheelProperties,
            in SuspensionState suspensionState,
            ref WheelState wheelState,
            ref VehicleState vehicleState)
        {
            wheelState.m_transitional.m_accumDT += subStepDeltaTime;

            if (!wheelState.m_isGrounded)
            {
                wheelState.m_transitional.m_velocityContact_ws = Vector3.zero;
                wheelState.m_transitional.m_velocityLateral = 0.0f;
                wheelState.m_transitional.m_velocityLongitudinal = 0.0f;

                wheelState.m_transitional.m_desiredLateralImpulse = 0.0f;
                wheelState.m_transitional.m_desiredLongitudinalImpulse = 0.0f;

                wheelState.m_transitional.m_availableGripImpulse = 0.0f;

                wheelState.m_transitional.m_currentLateralImpulse = 0.0f;
                wheelState.m_transitional.m_currentLongitudinalImpulse = 0.0f;

                UpdateWheelAngularVelocityFromLateralAndLongitudinalForce(subStepDeltaTime, wheelProperties, ref wheelState);

                return;
            }

            CalculateWheelTransitionalVelocityStates(vehicleState, ref wheelState);
            CalculateWheelDesiredLateralForce(
                subStepDeltaTime, context, wheelProperties, vehicleState, ref wheelState);
            CalculateWheelDesiredLongitudinalForce(
                subStepDeltaTime, context, wheelProperties, vehicleState, ref wheelState);

            wheelState.m_transitional.m_accumDesiredLateralImpulse += wheelState.m_transitional.m_desiredLateralImpulse;
            wheelState.m_transitional.m_accumDesiredLongitudinalImpulse += wheelState.m_transitional.m_desiredLongitudinalImpulse;

            CalculateWheelLateralAndLongitudinalForce(
                subStepDeltaTime, context, wheelProperties, vehicleState, suspensionState, ref wheelState);

            wheelState.m_transitional.m_accumAvailableGripImpulse += wheelState.m_transitional.m_availableGripImpulse;

            UpdateWheelAngularVelocityFromLateralAndLongitudinalForce(subStepDeltaTime, wheelProperties, ref wheelState);

            //Calculate accumulative statistics
            if (Mathf.Abs(wheelState.m_transitional.m_currentLateralImpulse) < EPS)
            {
                wheelState.m_transitional.m_currentLateralImpulse = 0.0f;
            }
            if (Mathf.Abs(wheelState.m_transitional.m_currentLongitudinalImpulse) < EPS)
            {
                wheelState.m_transitional.m_currentLongitudinalImpulse = 0.0f;
            }

            wheelState.m_transitional.m_accumGeneratedLateralImpulse += wheelState.m_transitional.m_currentLateralImpulse;
            wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse += wheelState.m_transitional.m_currentLongitudinalImpulse;

            UpdateVehicleStateFromWheel(wheelState, ref vehicleState);
        }

        public static void UpdateWheelAndApplyToVehicle_PostPhysics(
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