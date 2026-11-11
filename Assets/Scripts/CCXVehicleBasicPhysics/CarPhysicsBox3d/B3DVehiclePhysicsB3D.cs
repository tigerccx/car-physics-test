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
        public static float EPS = 1e-6f;
        static float RPM_TO_AV = Mathf.PI / 30.0f;
        static float AV_TO_RPM = 1.0f / RPM_TO_AV;

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

        public static float RPMToAngularV(float rpm)
        {
            return rpm * RPM_TO_AV;
        }
        public static float AngularVToRPM(float angularVelocity)
        {
            return angularVelocity * AV_TO_RPM;
        }
        private static float GetAverageDrivingWheelRad(in List<Wheel> wheels)
        {
            float avgWheelRad = 0.0f;
            int countDrivingWheels = 0;
            foreach (Wheel wheel in wheels)
            {
                if (!wheel.m_wheelProperties.m_isConnectedToDriveTrain)
                    continue;
                avgWheelRad += wheel.m_wheelProperties.m_wheelRadius;
                ++countDrivingWheels;
            }
            avgWheelRad /= countDrivingWheels;
            return avgWheelRad;

        }
        private static float AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(float speed, float wheelRad, float differentialGearRatio)
        {
            float drivingAxleRPMThres = speed / (2 * wheelRad * Mathf.PI);
            return drivingAxleRPMThres * differentialGearRatio;
        }

        private static float SmoothStep(float x)
        {
            x = Mathf.Clamp01(x);
            return x * x * (3f - 2f * x);
        }

        private static float ExponentialMovingAverage(float step, float filterScale)
        {
            return 1 - Mathf.Exp(-step * filterScale);
        }

        public static float GetSignedAngle(Quaternion q, Vector3 axis)
        {
            axis.Normalize();

            Vector3 v = new Vector3(q.x, q.y, q.z);

            float angle = 2f * Mathf.Atan2(
                Vector3.Dot(v, axis),
                q.w
            ) * Mathf.Rad2Deg;

            return Mathf.DeltaAngle(0f, angle);
        }

        public static void GetWheelPose(
            in WheelState wheelState, bool isRightFacing,
            out Vector3 pos, out Quaternion rot)
        {
            pos = wheelState.m_currentPos_ws;
            Quaternion wheelStanding = Quaternion.FromToRotation(Vector3.up, Vector3.right * (isRightFacing ? 1f : -1f));
            Vector3 dirRight = Vector3.Cross(wheelState.m_currentDirFoward_ws, wheelState.m_currentDirUp_ws);
            Vector3 dirFacing = isRightFacing ? dirRight : -dirRight;
            Quaternion wheelPose = Quaternion.LookRotation(wheelState.m_currentDirFoward_ws, wheelState.m_currentDirUp_ws);
            Quaternion wheelRot = Quaternion.AngleAxis(-wheelState.m_currentRotAngle, dirRight);
            rot = wheelRot * wheelPose * wheelStanding;
        }
        #endregion


        #region Vehicle
        public static void AutoConfigATGearSettings(
            ref VehicleProperties vehicleProperty)
        {
            if (vehicleProperty.m_gearBoxProperties.m_gears.Count <= 0)
            {
                return;
            }

            float gearUpRPM = vehicleProperty.m_engineProperty.m_engineMaxRPM * vehicleProperty.m_gearBoxProperties.m_ATGearUpRPMScale;
            float targetRPM = vehicleProperty.m_engineProperty.m_engineMaxRPM * vehicleProperty.m_gearBoxProperties.m_ATGearDownTargetRPMScale;

            DrivingGear gear1 = vehicleProperty.m_gearBoxProperties.m_gears[0];
            gear1.m_ATGearUpRPM = gearUpRPM;
            gear1.m_ATGearDownRPM = -10000;

            for (int iGear = 1; iGear < vehicleProperty.m_gearBoxProperties.m_gears.Count; ++iGear)
            {
                DrivingGear prevGear = vehicleProperty.m_gearBoxProperties.m_gears[iGear - 1];
                DrivingGear curGear = vehicleProperty.m_gearBoxProperties.m_gears[iGear];

                float gearRatioScale = prevGear.m_gearRatio / curGear.m_gearRatio;
                float safeTargetRPM = Mathf.Min(vehicleProperty.m_engineProperty.m_engineMaxRPM, targetRPM);
                float gearDownRPM = safeTargetRPM / gearRatioScale;

                curGear.m_ATGearUpRPM = gearUpRPM;
                curGear.m_ATGearDownRPM = gearDownRPM;
            }
        }

        public static DrivingGear GetCurrentGear(
            in VehicleProperties vehicleProperty, in VehicleState vehicleState)
        {
            int idxGear = vehicleState.m_idxCurrentGear;
            if (idxGear >= 0)
            {
                if(idxGear < vehicleProperty.m_gearBoxProperties.m_gears.Count)
                {
                    return vehicleProperty.m_gearBoxProperties.m_gears[idxGear];
                }
                else
                {
                    return null;
                }
            }
            else
            {
                return vehicleProperty.m_gearBoxProperties.m_reverseGear;
            }
        }

        public static bool GetCurrentGearIsReverseGear(in VehicleState vehicleState)
        {
            return vehicleState.m_idxCurrentGear < 0;
        }

        public static float GetCurrentGearBoxGearRatio(
            in VehicleProperties vehicleProperty, in VehicleState vehicleState)
        {
            DrivingGear gear = GetCurrentGear(vehicleProperty, vehicleState);
            bool isReverse = GetCurrentGearIsReverseGear(vehicleState);
            float gearRatio = isReverse ? -1.0f: 1.0f;
            if (gear != null)
            {
                gearRatio *= gear.m_gearRatio;
            }
            return gearRatio;
        }

        public static float GetCurrentGearRatio(
            in VehicleProperties vehicleProperty, in VehicleState vehicleState)
        {
            float gearRatio = GetCurrentGearBoxGearRatio(vehicleProperty, vehicleState);
            gearRatio *= vehicleProperty.m_differentialGearRatio;
            return gearRatio;
        }

        public static bool IsInForwardGear(in VehicleState vehicleState) 
        { 
            return vehicleState.m_idxCurrentGear >= 0; 
        }
        public static bool IsInReverseGear(in VehicleState vehicleState) 
        { 
            return vehicleState.m_idxCurrentGear < 0; 
        }

        public static float GetVehicleSpeed(in List<Wheel> wheels)
        {
            // Using non-driving wheels avg speed as vehicle speed
            int countNonDrivingWheels = 0;
            float avgAngularVelocityNonDriving = 0.0f;
            float avgRadiusNonDriving = 0.0f;
            foreach (Wheel wheel in wheels)
            {
                if (!wheel.m_wheelProperties.m_isConnectedToDriveTrain)
                {
                    ++countNonDrivingWheels;
                    avgAngularVelocityNonDriving += wheel.m_wheelState.m_currentAngularVelocity;
                    avgRadiusNonDriving += wheel.m_wheelProperties.m_wheelRadius;
                }
            }

            avgAngularVelocityNonDriving /= countNonDrivingWheels;
            avgRadiusNonDriving /= countNonDrivingWheels;
            float vehicleSpeed = avgAngularVelocityNonDriving * avgRadiusNonDriving;


            return vehicleSpeed;
        }

        public static float GetSmoothedVehicleSpeed(in List<Wheel> wheels, float currentVehicleSpeed, float timeStep, float filterTime = 0.3f)
        {
            float rawVehicleSpeed = GetVehicleSpeed(wheels);
            float newVehicleSpeed = ExponentialMovingAverage(timeStep, 1f/filterTime)*(rawVehicleSpeed - currentVehicleSpeed) + currentVehicleSpeed;
            return newVehicleSpeed;
        }

        private static bool CanShiftToReverse(
            in VehicleProperties vehicleProperty,
            in VehicleState vehicleState)
        {
            float limit = AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(
                vehicleProperty.m_gearBoxProperties.m_forwardToReverseSpeedThreshold,
                vehicleProperty.m_avgDrivingWheelRad,
                vehicleProperty.m_differentialGearRatio);
            return vehicleState.m_idxCurrentGear == 0 && vehicleState.m_driveOutputShaftRotationSpeed < limit;
        }

        private static bool CanShiftToForward(
            in VehicleProperties vehicleProperty,
            in VehicleState vehicleState)
        {
            float limit = AverageDrivingWheelLinearSpeedToDriveShaftRotationSpeed(
                vehicleProperty.m_gearBoxProperties.m_reverseToForwardSpeedThreshold,
                vehicleProperty.m_avgDrivingWheelRad,
                vehicleProperty.m_differentialGearRatio);
            return vehicleState.m_idxCurrentGear == -1 && vehicleState.m_driveOutputShaftRotationSpeed > -limit;
        }

        private static void ChangeGear(
            int gearChange,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            if (gearChange == 0)
            {
                return;
            }

            int maxGear = vehicleProperty.m_gearBoxProperties.m_gears.Count - 1;
            int newGear = math.clamp(vehicleState.m_idxCurrentGear + gearChange, -1, maxGear);

            // Shift to reverse gear
            if (vehicleState.m_idxCurrentGear == 0 && newGear == -1)
            {
                // Forward speed is too high to shift into reverse gear
                if (!CanShiftToReverse(vehicleProperty, vehicleState))
                {
                    newGear = 0;
                }
            }
            // Shift to forward gear
            else if (vehicleState.m_idxCurrentGear == -1 && newGear == 1)
            {
                // Reverse speed is too high to shift into forward gear
                if (!CanShiftToForward(vehicleProperty, vehicleState))
                {
                    newGear = -1;
                }
            }

            vehicleState.m_idxCurrentGear = newGear;
        }

        private static void ConsumeManualGearChange(
            ref VehicleContext vehicleContext,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            int gearChange = vehicleContext.m_inputs.m_gearChange;

            ChangeGear(gearChange, vehicleProperty, ref vehicleState);

            vehicleContext.m_inputs.m_gearChange = 0;
        }

        private static void CalculateATChangeForwardGear(
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            DrivingGear curGear = GetCurrentGear(vehicleProperty, vehicleState);
            if (vehicleState.m_engineRPM >= curGear.m_ATGearUpRPM)
            {
                ChangeGear(1, vehicleProperty, ref vehicleState);
                DrivingGear newCurGear = GetCurrentGear(vehicleProperty, vehicleState);
                //Debug.Log($"AT Shift Up: {curGear.m_gearName}->{newCurGear.m_gearName} at RPM: {vehicleState.m_engineRPM}");
            }
            else if (vehicleState.m_engineRPM <= curGear.m_ATGearDownRPM)
            {
                ChangeGear(-1, vehicleProperty, ref vehicleState);
                DrivingGear newCurGear = GetCurrentGear(vehicleProperty, vehicleState);
                //Debug.Log($"AT Shift Down: {curGear.m_gearName}->{newCurGear.m_gearName} at RPM: {vehicleState.m_engineRPM}");
            }
        }

        private static void CalculateATSwitchForwardReverse(
            in VehicleContext vehicleContext,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            if (IsInForwardGear(vehicleState))
            {
                if (vehicleContext.m_inputs.m_throttleIntention < 0 && CanShiftToReverse(vehicleProperty, vehicleState))
                {
                    ChangeGear(-1, vehicleProperty, ref vehicleState);
                }
                else
                {
                    CalculateATChangeForwardGear(vehicleProperty, ref vehicleState);
                }
            }
            //TODO: 实现空挡时这里需要加空挡
            else if (IsInReverseGear(vehicleState))
            {
                if (vehicleContext.m_inputs.m_throttleIntention > 0 && CanShiftToForward(vehicleProperty, vehicleState))
                {
                    ChangeGear(1, vehicleProperty, ref vehicleState);
                }
            }
        }

        private static bool ShouldRevLimiterEngage(in VehicleProperties vehicleProperty, in VehicleState vehicleState)
        {
            return vehicleState.m_engineRPM > vehicleProperty.m_engineProperty.m_engineMaxRPM;
        }

        private static bool ShouldSpeedLimiterEngage(in VehicleProperties vehicleProperty, in VehicleState vehicleState)
        {
            bool shouldEngage = false;
            if (vehicleProperty.m_engineProperty.m_forwardSpeedLimit >= 0)
            {
                shouldEngage |= vehicleState.m_measuredVehicleSpeed > vehicleProperty.m_engineProperty.m_forwardSpeedLimit;
            }
            if (vehicleProperty.m_engineProperty.m_backwardSpeedLimit >= 0)
            {
                shouldEngage |= vehicleState.m_measuredVehicleSpeed < -vehicleProperty.m_engineProperty.m_backwardSpeedLimit;
            }
            return shouldEngage;
        }

        private static void CalculateLimiterState(
            float deltaTime, in VehicleProperties vehicleProperty, ref VehicleState vehicleState)
        {
            if (!vehicleState.m_limiterEngaged)
            {
                if (ShouldRevLimiterEngage(vehicleProperty, vehicleState) || 
                    ShouldSpeedLimiterEngage(vehicleProperty, vehicleState))
                {
                    vehicleState.m_limitEngageTimer += deltaTime;
                    if (vehicleState.m_limitEngageTimer >= vehicleProperty.m_engineProperty.m_limiterReactionTime)
                    {
                        vehicleState.m_limiterEngaged = true;
                        vehicleState.m_limitEngageTimer = 0.0f;
                    }
                }
                else
                {
                    vehicleState.m_limitEngageTimer = 0.0f;
                }
            }
            else
            {
                if (! ShouldRevLimiterEngage(vehicleProperty, vehicleState) && 
                    !ShouldSpeedLimiterEngage(vehicleProperty, vehicleState))
                {
                    vehicleState.m_limitEngageTimer += deltaTime;
                    if (vehicleState.m_limitEngageTimer >= vehicleProperty.m_engineProperty.m_limiterReactionTime)
                    {
                        vehicleState.m_limiterEngaged = false;
                        vehicleState.m_limitEngageTimer = 0.0f;
                    }
                }
                else
                {
                    vehicleState.m_limitEngageTimer = 0.0f;
                }
            }
        }

        public static void InitVehicleState(
            ref VehicleProperties vehicleProperty,
            in List<Wheel> wheels,
            ref VehicleState vehicleState)
        {
            vehicleProperty.m_avgDrivingWheelRad = GetAverageDrivingWheelRad(wheels);
        }

        public static void UpdateVehicleInternalStates(
            float deltaTime,
            ref VehicleContext vehicleContext,
            in VehicleProperties vehicleProperty, 
            in List<Wheel> wheels,
            ref VehicleState vehicleState)
        {
            //TODO: 当轴和引擎断开时，单独计算引擎本身的情况(m_bClutchEngaged需要处理)

            // Calculate Engine State and Drive Train State
            int wheelCount = wheels.Count;

            //TODO: 引入轴，轴和轴可以通过differential链接
            //Conventional Differential: Drive Shaft Speed = Avg Wheel Speed
            float combinedAngularVelocity = 0.0f;
            int countDrivingWheels = 0;
            foreach (Wheel wheel in wheels)
            {
                if(!wheel.m_wheelProperties.m_isConnectedToDriveTrain)
                {
                    continue;
                }

                ++countDrivingWheels;
                combinedAngularVelocity += wheel.m_wheelState.m_currentAngularVelocity;
            }

            float engineIdleAngularVelocity = RPMToAngularV(vehicleProperty.m_engineProperty.m_engineIdleRPM);

            if (countDrivingWheels != 0)
            {

                //TODO: 目前先走从驱动轮倒推的EngineRPM
                float diffCarrierRotationSpeed = combinedAngularVelocity / countDrivingWheels;
                vehicleState.m_driveOutputShaftRotationSpeed = diffCarrierRotationSpeed * vehicleProperty.m_differentialGearRatio;
                float gearRatio = GetCurrentGearBoxGearRatio(vehicleProperty, vehicleState);
                vehicleState.m_driveInputShaftRotationSpeed = vehicleState.m_driveOutputShaftRotationSpeed * gearRatio;

                float engineAngularVelocity = 0.0f;
                if (vehicleState.m_bClutchEngaged)
                {
                    engineAngularVelocity = vehicleState.m_driveInputShaftRotationSpeed;
                }
                else
                {
                    // TODO: 给Clutch disengage一个曲线，而非用01描述这个过程
                    float deltaEngineRotSpeed = vehicleState.m_engineOutputTorque / vehicleProperty.m_engineProperty.m_engineInertia * deltaTime;
                    engineAngularVelocity += deltaEngineRotSpeed;
                }

                //Debug.Log($"engineRPM {AngularVToRPM(engineAngularVelocity)}");

                engineAngularVelocity = MathF.Max(engineAngularVelocity, engineIdleAngularVelocity);

                vehicleState.m_engineAngularVelocity = engineAngularVelocity;
            }
            else
            {
                //TODO: 当轴和引擎断开时，单独计算引擎本身的情况
            }
            vehicleState.m_engineRPM = AngularVToRPM(vehicleState.m_engineAngularVelocity);
            vehicleState.m_measuredVehicleSpeed = GetSmoothedVehicleSpeed(wheels, vehicleState.m_measuredVehicleSpeed, deltaTime);

            // Change Gear
            if (vehicleProperty.m_gearBoxProperties.m_transmissionSystemType == TransmissionSystemType.ManualTransmission)
            {
                ConsumeManualGearChange(ref vehicleContext, vehicleProperty, ref vehicleState);
            }
            else
            {
                CalculateATSwitchForwardReverse(vehicleContext, vehicleProperty, ref vehicleState);
            }

            //Calculate limiter (revolve limiter and speed limiter)
            CalculateLimiterState(deltaTime, vehicleProperty, ref vehicleState);

            //Calculate Torque
            float sampledTorque = 0.0f;
            if(vehicleProperty.m_engineProperty.m_torqueCurve)
            {
                sampledTorque = vehicleProperty.m_engineProperty.m_torqueCurve.Evaluate(vehicleState.m_engineRPM);
            }

            if (!vehicleState.m_limiterEngaged)
            {
                vehicleState.m_engineOutputTorque = sampledTorque;
                vehicleState.m_engineOutputTorque *= Mathf.Abs(vehicleContext.m_inputs.m_throttle);
            }
            else
            {
                // Rev limiter
                float overRedlineRPM = Mathf.Max(vehicleState.m_engineRPM - vehicleProperty.m_engineProperty.m_engineMaxRPM, 0.0f);
                float sampledRedlineTorque = 0.0f;
                if (vehicleProperty.m_engineProperty.m_revLimiterEngagedRPMToTorqueCurve)
                {
                    sampledRedlineTorque = vehicleProperty.m_engineProperty.m_revLimiterEngagedRPMToTorqueCurve.Evaluate(overRedlineRPM);
                }
                
                // Speed limiter
                float overForwardLimitSpeed = Mathf.Max(vehicleState.m_measuredVehicleSpeed - vehicleProperty.m_engineProperty.m_forwardSpeedLimit, 0.0f);
                float overBackwardLimitSpeed = Mathf.Max(vehicleProperty.m_engineProperty.m_backwardSpeedLimit - vehicleState.m_measuredVehicleSpeed, 0.0f);
                float overLimitSpeed = Mathf.Max(overForwardLimitSpeed, overBackwardLimitSpeed);
                float sampledSpeedLimitTorque = 0.0f;
                if (vehicleProperty.m_engineProperty.m_speedLimiterEngagedSpeedToTorqueCurve)
                {
                    sampledSpeedLimitTorque = vehicleProperty.m_engineProperty.m_speedLimiterEngagedSpeedToTorqueCurve.Evaluate(overLimitSpeed);
                }

                float resistenceTorque = Mathf.Max(sampledRedlineTorque, sampledSpeedLimitTorque);
                //Debug.Log($"resistenceTorque {resistenceTorque} | Redline{sampledRedlineTorque} SpeedLimit{sampledSpeedLimitTorque}");

                vehicleState.m_engineOutputTorque = -resistenceTorque;
            }

            float loadTorque = vehicleProperty.m_engineProperty.m_engineInternalLoadTorque;
            float maxLoadTorque = Mathf.Max(0.0f, vehicleState.m_engineAngularVelocity - engineIdleAngularVelocity) * vehicleProperty.m_engineProperty.m_engineInertia / deltaTime;
            loadTorque = Mathf.Min(maxLoadTorque, loadTorque);
            //Debug.Log($"loadTorque {loadTorque}");

            vehicleState.m_engineNetOutputTorque = vehicleState.m_engineOutputTorque - loadTorque;

            //Debug.Log($"engineNetOutputTorque {vehicleState.m_engineNetOutputTorque}");
        }

        private static void CalculateDriftState(
            in List<Wheel> wheels,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehilcleState)
        {
            bool areRearsOverwhelmed = false;
            foreach (Wheel wheel in wheels)
            {
                if (wheel.m_wheelProperties.m_isRear)
                {
                    areRearsOverwhelmed |= wheel.m_wheelState.m_transitional.m_desireExceedsMaxGrip;
                }
            }

            if (!areRearsOverwhelmed)
            {
                vehilcleState.m_driftAmount = 0.0f;
                vehilcleState.m_fullyDrifting = false;
                return;
            }

            float driftAmountSmoothed = 0.0f;
            float absSlipAngle = Mathf.Abs(vehilcleState.m_slipAngle);
            if (vehilcleState.m_fullyDrifting)
            {
                float a = vehicleProperty.m_driftMeasurer.m_angleSideSlipReturn;
                float b = vehicleProperty.m_driftMeasurer.m_angleSideSlipReturn + vehicleProperty.m_driftMeasurer.m_angleSideSlipSmooth;
                float t = absSlipAngle - vehicleProperty.m_driftMeasurer.m_angleSideSlipReturn;
                driftAmountSmoothed = Mathf.Clamp01(t / (b - a));

                if (absSlipAngle <= a)
                {
                    vehilcleState.m_fullyDrifting = false;
                }
            }
            else
            {
                float a = vehicleProperty.m_driftMeasurer.m_angleSideSlipStart;
                float b = vehicleProperty.m_driftMeasurer.m_angleSideSlipStart + vehicleProperty.m_driftMeasurer.m_angleSideSlipSmooth;
                float t = absSlipAngle - vehicleProperty.m_driftMeasurer.m_angleSideSlipStart;
                driftAmountSmoothed = Mathf.Clamp01(t / (b - a));

                if (absSlipAngle >= b)
                {
                    vehilcleState.m_fullyDrifting = true;
                }

                //Debug.Log($"driftAmountSmoothed{driftAmountSmoothed} = absSlipAngle{absSlipAngle} vs {a}-end{b}");
            }

            vehilcleState.m_driftAmount = driftAmountSmoothed;
        }

        //TODO: Seperate drive assist against wheel assist
        private static void CalculateAndApplyDriveAssist(
            float subStepDeltaTime,
            in List<Wheel> wheels,
            in VehicleContext vehicleContext,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            Vector3 up = Vector3.zero;
            float steeringAngle = 0.0f;
            bool toApply = true;
            // Use average of wheels' up as correction plane
            foreach (Wheel wheel in wheels)
            {
                up += wheel.m_wheelState.m_currentDirUp_ws;
                steeringAngle += wheel.m_wheelState.m_currentSteerAngle;
                toApply &= wheel.m_wheelState.m_isGrounded;
            }
            if (!toApply)
            {
                vehicleState.m_transitionalVelocity = vehicleState.m_b3dBody.velocity;
                vehicleState.m_driftAssistAngularImpulse = Vector3.zero;
                return;
            }

            up.Normalize();
            steeringAngle /= wheels.Count;

            Vector3 lastVelocity = vehicleState.m_transitionalVelocity;
            
            Vector3 lastPlanarVelocity = Vector3.ProjectOnPlane(lastVelocity, up);
            Vector3 currentPlanarVelocity = Vector3.ProjectOnPlane(vehicleState.m_b3dBody.velocity, up);

            Vector3 dirForward = Vector3.ProjectOnPlane(vehicleState.m_b3dBody.transform.forward, up);
            Vector3 dirRight = Vector3.ProjectOnPlane(vehicleState.m_b3dBody.transform.right, up);
            float currentPlanarVelocityLong = Vector3.Dot(currentPlanarVelocity, dirForward);
            float currentPlanarVelocityLat = Vector3.Dot(currentPlanarVelocity, dirRight);

            if (currentPlanarVelocityLong < EPS)
            {
                vehicleState.m_transitionalVelocity = vehicleState.m_b3dBody.velocity;
                vehicleState.m_driftAssistAngularImpulse = Vector3.zero;
                return;
            }

            //TODO: Slip Angle等状态计算给它提出来，成UpdateVehicleSubstepState
            float slipAngle = Mathf.Atan2(currentPlanarVelocityLat, Mathf.Max(Mathf.Abs(currentPlanarVelocityLong), EPS)); //Positive -> right, Negative -> left
            slipAngle = Mathf.Rad2Deg * slipAngle;
            vehicleState.m_slipAngle = slipAngle;

            // Check if the vehicle is drifting
            CalculateDriftState(wheels, vehicleProperty, ref vehicleState);

            float driftAmount = vehicleState.m_driftAmount;

            float pendingVelocityRotAngle = Vector3.SignedAngle(lastPlanarVelocity, currentPlanarVelocity, up);

            // Only when drifting
            // To make vehicle rotation controllable during drifting
            if (driftAmount > EPS)
            {
                // 计算操控和带来的辅助的速度转向速度
                float throttle = vehicleContext.m_inputs.m_throttle;
                float steering = vehicleContext.m_inputs.m_steering;
                float standardTargetVelocity = vehicleProperty.m_driftControlAssistVelocity * subStepDeltaTime; //angle
                float driftDirection = Mathf.Clamp(-currentPlanarVelocityLat / 1f, -1f, 1f);
                float targetVelocityThrottleScale = Mathf.Max(throttle, 0f) * driftDirection * vehicleProperty.m_driftControlAssistThrottleScale;
                float targetVelocitySteeringScale = steering * vehicleProperty.m_driftControlAssistSteeringScale;
                float velocityTurnAssistScale = targetVelocityThrottleScale + targetVelocitySteeringScale;

                //Debug.Log($"Assist Angle {velocityTurnAssistScale} = t{targetVelocityThrottleScale} + s{targetVelocitySteeringScale}");

                float targetAssistVelocityTurningVelocityAngle = velocityTurnAssistScale * standardTargetVelocity;
                pendingVelocityRotAngle = pendingVelocityRotAngle + targetAssistVelocityTurningVelocityAngle * driftAmount;

                //Vector3 targetPlanarVelocity = Quaternion.AngleAxis(targetAssistVelocityTurningVelocityAngle, up) * currentPlanarVelocity;
                //Vector3 velocityRotAssitImpulse = (targetPlanarVelocity - currentPlanarVelocity) * vehicleState.m_b3dBody.mass;
            }

            //Debug.Log($"Total Assist Angle {pendingVelocityRotAngle} = t{pendingVelocityRotAngle} + s{targetAssistVelocityTurningVelocityAngle}");

            pendingVelocityRotAngle = pendingVelocityRotAngle * Mathf.Deg2Rad / subStepDeltaTime;
            Vector3 pendingVelocityRotVelocity = pendingVelocityRotAngle * up;

            Vector3 pendingBodyRotVelocity = Vector3.Project(vehicleState.m_b3dBody.angularVelocity, up);

            Vector3 deltaDriftStablizerAngularVelocity = pendingVelocityRotVelocity - pendingBodyRotVelocity;

            //// Limit maximum change by stablizer
            //float deltaAngularVelocityLimit = vehicleProperty.testMaxDriftDeltaAngularVelocityAngle * subStepDeltaTime;
            //Debug.Log($"Unlimited stablier accl {deltaAngularVelocity.magnitude}/{deltaAngularVelocityLimit}");

            Matrix4x4 inverseWorldInertia = vehicleState.m_b3dBody.GetWorldInverseRotationalInertia();
            float effectiveInverseInertia = Vector3.Dot(up, inverseWorldInertia.MultiplyVector(up));

            if (effectiveInverseInertia < EPS)
            {
                vehicleState.m_transitionalVelocity = vehicleState.m_b3dBody.velocity;
                vehicleState.m_driftAssistAngularImpulse = Vector3.zero;
                return;
            }

            // Calculate angular impulse to assist stablizing drifting
            Vector3 driftStablizeAngularImpulse = deltaDriftStablizerAngularVelocity / effectiveInverseInertia;
            float driftAssistScale = 0.0f;
            // Stablize drifting
            if (vehicleProperty.m_driftAssistVehicleSlipAngleToAssistScaleCurve)
            {
                driftAssistScale = vehicleProperty.m_driftAssistVehicleSlipAngleToAssistScaleCurve.Evaluate(Mathf.Abs(slipAngle));
                driftAssistScale = Mathf.Clamp01(driftAssistScale);
                driftAssistScale *= driftAmount;

                //Debug.Log($"driftAssistScale {driftAssistScale}");
            }
            float driftStablizerSpeedScale = SmoothStep((currentPlanarVelocityLong - vehicleProperty.m_driftAssistEnableThreshold) / vehicleProperty.m_driftAssistLowSpeedFadeIn);
            float driftGain = ExponentialMovingAverage(vehicleProperty.m_driftControlAssistScale, subStepDeltaTime);
            driftStablizeAngularImpulse *= driftAssistScale * driftStablizerSpeedScale * driftGain;
            //Debug.Log($"final driftAssistScale {driftAssistScale * driftStablizerSpeedScale * driftGain}");

            // Stablize straightline steering
            // How much to turn the vehicle to face current velocity
            float driveStablizerRotationAngle = Vector3.SignedAngle(dirForward, currentPlanarVelocity, up) * Mathf.Deg2Rad;
            float bodyRotVelocity = Vector3.Dot(pendingBodyRotVelocity, up);
            float targetDriveRotationVelocity = driveStablizerRotationAngle; //per 1s

            float deltaRotationVelocity = (targetDriveRotationVelocity - bodyRotVelocity);

            Vector3 driveStablizerAngularVelocity = deltaRotationVelocity * up;
            Vector3 driveStablizeAngularImpulse = driveStablizerAngularVelocity / effectiveInverseInertia;

            float driveAssistScale = 0.0f;
            if (vehicleProperty.m_driveAssistVehicleSlipAngleToAssistScaleCurve)
            {
                driveAssistScale = vehicleProperty.m_driveAssistVehicleSlipAngleToAssistScaleCurve.Evaluate(Mathf.Abs(slipAngle));
                driveAssistScale = Mathf.Clamp01(driveAssistScale);
                driveAssistScale *= (1 - Mathf.Abs(vehicleContext.m_inputs.m_steering)); //Reduce drive assist when steering

                driveAssistScale *= (1- driftAmount);

                //Debug.Log($"driveAssistScale {driveAssistScale}");
            }
            float driveStablizerSpeedScale = SmoothStep((currentPlanarVelocityLong - vehicleProperty.m_driveAssistEnableThreshold) / vehicleProperty.m_driveAssistLowSpeedFadeIn);
            float driveGain = ExponentialMovingAverage(vehicleProperty.m_driveControlAssistScale, subStepDeltaTime);
            driveStablizeAngularImpulse *= driveAssistScale * driveStablizerSpeedScale * driveGain;
            //Debug.Log($"final driveAssistScale {driveAssistScale * driveStablizerSpeedScale * driveGain}");

            //velocityRotAssitImpulse *= speedScale;

            //float steeringImpulse = steering * vehicleProperty.m_steeringAssistForce * subStepDeltaTime;
            //angularImpulse += steeringImpulse * up;

            Vector3 assistCombinedAngularImpulse = driftStablizeAngularImpulse + driveStablizeAngularImpulse;
            vehicleState.m_driftAssistAngularImpulse = assistCombinedAngularImpulse;
            if (assistCombinedAngularImpulse.sqrMagnitude > EPS * EPS)
            {
                // Assist vehicle rotation
                vehicleState.m_b3dBody.ApplyAngularImpulse(assistCombinedAngularImpulse);
            }

            //TODO: 漂移的轨迹辅助有问题。。。

            //vehicleState.m_driftAssistVelocityImpulse = velocityRotAssitImpulse;
            //if (velocityRotAssitImpulse.sqrMagnitude > EPS * EPS)
            //{
            //    // Assist the turn
            //    vehicleState.m_b3dBody.ApplyLinearImpulseToCenter(velocityRotAssitImpulse);
            //}

            //Debug.Log($"velocityRotV {pendingVelocityRotVelocity} | bodyRotV {vehicleState.m_b3dBody.angularVelocity} | delta {deltaAngularVelocity} | angularImpulse{angularImpulse.magnitude}");

            vehicleState.m_transitionalVelocity = vehicleState.m_b3dBody.velocity;
        }

        public static void UpdateVehicleSubstep(
            float subStepDeltaTime,
            in VehicleContext vehicleContext,
            in List<Wheel> wheels,
            in VehicleProperties vehicleProperty,
            ref VehicleState vehicleState)
        {
            if(vehicleProperty.m_enableDriveAssist)
            {
                CalculateAndApplyDriveAssist(subStepDeltaTime, wheels, vehicleContext, vehicleProperty, ref vehicleState);
            }
        }

        public static void UpdateWheelStatesFromVehicle(
            float deltaTime,
            in VehicleContext vehicleContext,
            in VehicleProperties vehicleProperty,
            in VehicleState vehicleState,
            List<Wheel> wheels)
        {
            //TODO: 引入轴的概念，用轴来计算每个轮胎的torque输入
            int countDrivingWheels = 0;
            foreach (Wheel wheel in wheels)
            {
                WheelState wheelState = wheel.m_wheelState;
                WheelProperties wheelProperty = wheel.m_wheelProperties;

                if (wheelProperty.m_isSteerable)
                {
                    wheelState.m_currentSteerAngle = vehicleContext.m_inputs.m_steering * wheelProperty.m_allowedAngle;
                }
                else
                {
                    wheelState.m_currentSteerAngle = 0;
                }

                if (wheelProperty.m_isConnectedToDriveTrain)
                {
                    ++countDrivingWheels;
                }
                else
                {
                    wheelState.m_currentTorque = 0.0f;
                }

                if (wheelProperty.m_allowHandbrake && vehicleContext.m_inputs.m_handbrake)
                {
                    wheelState.m_currentBrakeTorque = wheelProperty.m_handbrakeTorque;
                    wheelState.m_brakeType = WheelBrakeType.HandBrake;
                }
                else
                {
                    wheelState.m_currentBrakeTorque = wheelProperty.m_brakeTorque * vehicleContext.m_inputs.m_brake;
                    wheelState.m_brakeType = WheelBrakeType.Brake;
                }
            }

            float gearRatio = GetCurrentGearRatio(vehicleProperty, vehicleState);

            //TODO: Currently using open differential. Consider other types of differential
            float wheelTorque = vehicleState.m_engineNetOutputTorque * gearRatio / countDrivingWheels;
            foreach (Wheel wheel in wheels)
            {
                WheelState wheelState = wheel.m_wheelState;
                WheelProperties wheelProperty = wheel.m_wheelProperties;

                if (wheelProperty.m_isConnectedToDriveTrain)
                {
                    wheelState.m_currentTorque = wheelTorque;
                }
            }
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

        public static void InitSuspension(
            in VehicleState vehicleState,
            ref Suspension suspension)
        {
            Box3DBody b3dBody = vehicleState.m_b3dBody;
            Transform bodyTransform = b3dBody.transform;
            suspension.m_suspensionProperties.m_posConnection_vehicle_ls =
                bodyTransform.InverseTransformPoint(suspension.transform.position);
            suspension.m_suspensionProperties.m_suspensionDirection_vehicle_ls =
                bodyTransform.InverseTransformDirection(suspension.transform.up);
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
            wheelState.m_transitional.m_desireExceedsMaxGrip = false;

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

            wheelState.m_transitional.m_ABSEngaged = false;
            wheelState.m_transitional.m_accumBrakeTorque = 0.0f;
            
            
            wheelState.m_transitional.m_subStepCount = 0;
        }

        private static void IntegrateWheelTransitionalStates(in WheelProperties wheelProperties, ref WheelState wheelState)
        {
            wheelState.m_desiredLateralImpulse = wheelState.m_transitional.m_accumDesiredLateralImpulse;
            wheelState.m_desiredLongitudinalImpulse = wheelState.m_transitional.m_accumDesiredLongitudinalImpulse;

            wheelState.m_availableGripImpulse = wheelState.m_transitional.m_accumAvailableGripImpulse;

            wheelState.m_currentLateralImpulse = wheelState.m_transitional.m_accumGeneratedLateralImpulse;
            wheelState.m_currentLongitudinalImpulse = wheelState.m_transitional.m_accumGeneratedLongitudinalImpulse;

            wheelState.m_currentAngularVelocity = wheelState.m_transitional.m_angularVelocity;

            wheelState.m_isABSEngaged = wheelState.m_transitional.m_ABSEngaged;
            wheelState.m_currentBrakeTorque = wheelState.m_transitional.m_accumBrakeTorque / wheelState.m_transitional.m_subStepCount;
            
            wheelState.m_currentRotAngle += wheelState.m_currentAngularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime;
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
            vehicleState.m_transitionalVelocity = vehicleState.m_b3dBody.velocity;
        }

        private static float CalculateABSBrakeTorque(
            in VehicleProperties vehicleProperties,
            in WheelProperties wheelProperty,
            ref WheelState wheelState,
            float brakeTorque)
        {
            float currentSlipRatio = wheelState.m_transitional.m_slipRatio;
            float diffSlipRatio = -vehicleProperties.m_ABSOptimalSlipRatio - currentSlipRatio;

            if (diffSlipRatio < 0f)
            // Wheel faster than optimal slip ratio -> ABS disengage
            {
                wheelState.m_transitional.m_ABSEngaged |= false;
                return brakeTorque;
            }
            else
            // Wheel slower than optimal slip ratio
            {
                wheelState.m_transitional.m_ABSEngaged |= true;

                float newBrakeTorque = 0.0f;
                if (vehicleProperties.m_ABSSlipToTorqueScaleCurve)
                {
                    float scale = vehicleProperties.m_ABSSlipToTorqueScaleCurve.Evaluate(diffSlipRatio);
                    scale = Mathf.Clamp01(scale);
                    newBrakeTorque = brakeTorque * scale;
                }

                return newBrakeTorque;
            }
        }

        private static void CalculateWheelTransitionalVelocityStates(
            float deltaTime,
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
            in VehicleContext context, in VehicleProperties vehicleProperty,
            int iterCount,
            in List<Suspension> suspensions,
            in List<Wheel> wheels,
            ref VehicleState vehicleState)
        {
            int wheelCount = wheels.Count;

            // Initialize for iterations
            for (int i = 0; i < wheelCount; ++i)
            {
                // Rotate the wheel (With drive and brake)
                WheelState wheelState = wheels[i].m_wheelState;
                WheelProperties wheelProperty = wheels[i].m_wheelProperties;
                SuspensionState suspensionState = suspensions[i].m_suspensionState;

                wheelState.m_transitional.m_subStepCount += 1;

                float brakeTorque = wheelState.m_currentBrakeTorque;

                if (brakeTorque > EPS && vehicleProperty.m_enableABS && wheelState.m_isGrounded)
                {
                    CalculateWheelTransitionalVelocityStates(
                        subStepDeltaTime, wheelProperty, vehicleState, ref wheelState);

                    brakeTorque = CalculateABSBrakeTorque(vehicleProperty, wheelProperty, ref wheelState, brakeTorque);
                }
                wheelState.m_transitional.m_accumBrakeTorque += brakeTorque;

                float angularImpulseDrive = wheelState.m_currentTorque * subStepDeltaTime;
                float angularImpulseBrake = Mathf.Abs(brakeTorque) * subStepDeltaTime;

                float deltaAngularDriveImpulse = angularImpulseDrive / wheelProperty.m_inertia;
                float deltaAngularBrakeImpulse = angularImpulseBrake / wheelProperty.m_inertia;
                float deltaAngularResistenceImpulse = deltaAngularBrakeImpulse;

               // Only apply drag when there is no control
                if (angularImpulseDrive < EPS && angularImpulseBrake < EPS)
                {
                    float angularImpulseDrag = Mathf.Abs(wheelProperty.m_extraDragTorque) * subStepDeltaTime;
                    float deltaAngularDragImpulse = angularImpulseDrag / wheelProperty.m_inertia;

                    deltaAngularResistenceImpulse += deltaAngularDragImpulse;
                }

                wheelState.m_transitional.m_angularVelocity += deltaAngularDriveImpulse;

                deltaAngularResistenceImpulse = Mathf.Min(deltaAngularResistenceImpulse, Mathf.Abs(wheelState.m_transitional.m_angularVelocity));
                wheelState.m_transitional.m_angularVelocity -= Mathf.Sign(wheelState.m_transitional.m_angularVelocity) * deltaAngularResistenceImpulse; ;

                wheelState.m_transitional.m_substepAccumImpulse = Vector2.zero;

                CalculateMaxGrip(subStepDeltaTime, context, wheelProperty, suspensionState, ref wheelState);
            }

            //Debug.Log($"----------- STEP -------------");
            for (int iter = 0; iter < iterCount; ++iter)
            {
                //Debug.Log($"Iter{iter} -------------");
                for (int innerIter = 0; innerIter < wheelCount; ++innerIter)
                {
                    int i = (iter % 2 == 0) ? innerIter : wheelCount - 1 - innerIter;

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
                        CalculateWheelTransitionalVelocityStates(
                            subStepDeltaTime, wheelProperty, vehicleState, ref wheelState);
                        CalculateWheelDesiredLateralCounterSlipForce(
                            subStepDeltaTime, context, wheelProperty, vehicleState, suspensionState, ref wheelState);
                        CalculateWheelDesiredLongitudinalCounterSlipForce(context, wheelProperty, vehicleState, ref wheelState);

                        // Clamp desired impulse
                        Vector2 newAccumImpulse = wheelState.m_transitional.m_substepAccumImpulse + new Vector2(
                            wheelState.m_transitional.m_desiredDeltaLateralImpulse, 
                            wheelState.m_transitional.m_desiredDeltaLongitudinalImpulse);
                        float maxGripImpulse = wheelState.m_transitional.m_maxGripImpulse;
                        newAccumImpulse = Vector2.ClampMagnitude(newAccumImpulse, maxGripImpulse);

                        wheelState.m_transitional.m_desireExceedsMaxGrip = newAccumImpulse.sqrMagnitude > maxGripImpulse * maxGripImpulse;

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