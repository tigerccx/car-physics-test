using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.InputSystem;

public class B3DVehicleController : MonoBehaviour
{
    [SerializeField] private InputActionAsset inputAsset;
    public B3DSimpleVehicle m_vehicle;

    [Header("Throttle")]
    [Description("Only available for keyboard")]
    public float m_throttleSmoothSpeedIncrease = 3f;
    [Description("Only available for keyboard")]
    public float m_throttleSmoothSpeedReturn = 4f;
    [Description("Only available for keyboard")]
    public NormalizedCurveAsset m_throttleSmoothInputOutputCurve;

    [Header("Steering")]
    public float m_steeringSmoothSpeedIncrease = 2f;
    public float m_steeringSmoothSpeedReturn = 3f;
    public NormalizedCurveAsset m_steeringSmoothInputOutputCurve;

    private InputAction m_throttle;
    private InputAction m_steering;
    private InputAction m_brake;
    private InputAction m_handbrake;
    private InputAction m_gearUp;
    private InputAction m_gearDown;

    private bool m_throttleUseKeyboard = false; //TODO: 之后切换为InputScheme管理
    private float m_throttleInternal = 0.0f;
    private float m_steeringInternal = 0.0f;

    private void OnEnable()
    {
        InputActionMap driving = inputAsset.FindActionMap("Driving");

        m_throttle = driving.FindAction("Throttle");
        m_steering = driving.FindAction("Steering");
        m_brake = driving.FindAction("Brake");
        m_handbrake = driving.FindAction("HandBrake");
        m_gearUp = driving.FindAction("GearUp");
        m_gearDown = driving.FindAction("GearDown");

        driving.Enable();
    }

    private void OnDisable()
    {
        inputAsset.FindActionMap("Driving").Disable();
    }

    private void Update()
    {
        // Throttle
        // Throttle curve is only for keyboard input
        float throttleInputRead = m_throttle.ReadValue<float>();
        float throttleOutput = 0.0f;
        if (m_throttle.activeControl != null)
        {
            m_throttleUseKeyboard = m_throttle.activeControl.device is Keyboard;
        }
        if (m_throttleUseKeyboard)
        {
            m_throttleInternal = Smooth(m_throttleInternal, throttleInputRead, 
                IsIncrease(m_throttleInternal, throttleInputRead) ? m_throttleSmoothSpeedIncrease: m_throttleSmoothSpeedReturn);
            throttleOutput = Mathf.Sign(m_throttleInternal) * m_throttleSmoothInputOutputCurve.Evaluate(Mathf.Abs(m_throttleInternal));
        }
        else
        {
            m_throttleInternal = throttleInputRead;
            throttleOutput = m_throttleInternal;
        }
        m_vehicle.m_throttleInput = throttleOutput;

        // Steering
        // Steering curve is for both keyboard and gamepad
        float steeringInputRead = m_steering.ReadValue<float>();
        float steeringOutput = 0.0f;
        m_steeringInternal = Smooth(m_steeringInternal, steeringInputRead,
            IsIncrease(m_steeringInternal, steeringInputRead) ? m_steeringSmoothSpeedIncrease : m_steeringSmoothSpeedReturn);
        steeringOutput = Mathf.Sign(m_steeringInternal) * m_steeringSmoothInputOutputCurve.Evaluate(Mathf.Abs(m_steeringInternal));
        m_vehicle.m_steeringInput = steeringOutput;

        // Others
        m_vehicle.m_brakeInput = m_brake.IsPressed(); //Brake input is only for keyboard. Gamepad use throttle negative. 
        m_vehicle.m_handbrakeInput = m_handbrake.IsPressed();
        if (m_gearUp.WasPerformedThisFrame())
        {
            m_vehicle.m_gearChangeInput += 1;
        }
        if (m_gearDown.WasPerformedThisFrame())
        {
            m_vehicle.m_gearChangeInput -= 1;
        }
    }

    private float Smooth(float currentInput, float targetInput, float smoothingSpeed)
    {
        float diffInput = targetInput - currentInput;
        float diffSmoothed = smoothingSpeed * Time.deltaTime;
        diffSmoothed = Mathf.Clamp(diffSmoothed, 0, Mathf.Abs(diffInput));
        float newInput = currentInput + Mathf.Sign(diffInput) * diffSmoothed;
        return newInput;
    }

    private bool IsIncrease(float currentInput, float targetInput)
    {
        float diffInput = targetInput - currentInput;
        if (diffInput > 0f && currentInput >= 0f)
        {
            return true;
        }
        if (diffInput < 0f && currentInput <= 0f)
        {
            return true;
        }
        return false;
    }
}
