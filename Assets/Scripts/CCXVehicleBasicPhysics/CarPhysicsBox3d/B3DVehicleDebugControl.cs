using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(B3DVehicleController))]
public class B3DVehicleDebugControl : MonoBehaviour
{
    public B3DVehicleController m_controller;
    public TextMeshProUGUI m_debugSpeedShower;

    private void Awake()
    {
        if(!m_controller)
        {
            m_controller = GetComponent<B3DVehicleController>();
        }
    }

    private void Update()
    {
        if (Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame &&
            m_controller.m_vehicle != null)
        {
            m_controller.m_vehicle.ResetPhysics(new Vector3(0f, 5f, 0f), Quaternion.identity);
        }

        if (Keyboard.current != null && Keyboard.current.eKey.wasPressedThisFrame &&
            m_controller.m_vehicle != null)
        {
            m_controller.m_vehicle.m_vehicleProperties.m_enableDriveAssist = !m_controller.m_vehicle.m_vehicleProperties.m_enableDriveAssist;
        }

        if (Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame &&
            m_controller.m_vehicle != null)
        {
            Application.Quit();
        }

        if (m_debugSpeedShower)
        {
            float speedMS = m_controller.m_vehicle.m_vehicleState.m_measuredVehicleSpeed;
            float speedKmH = speedMS * 3.6f;
            m_debugSpeedShower.text = speedKmH.ToString("000"); 
        }
    }
}
