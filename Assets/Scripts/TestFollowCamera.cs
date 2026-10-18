using UnityEngine;

public class TestFollowCamera : MonoBehaviour
{
    [Header("Camera")]
    public Camera camera;

    [Header("Target")]
    public Transform target;

    [Header("Camera Settings")]
    public Vector3 offset = new Vector3(0f, 3f, -6f);
    public float cameraXRotation = -5f;

    [Header("Camera Movement")]
    public float moveScale = 1f;
    public float durationExitMoveMode = 1f;
    public float rotationClampX = 60f;

    //[Header("Optional Smoothing")]
    //public float positionSmoothTime = 0.1f;
    //public float rotationSmoothSpeed = 10f;

    //private Vector3 velocity;
    private float m_rotMoveY = 0.0f;
    private float m_rotMoveX = 0.0f;
    private float m_exieMoveModeCountdown = 0.0f;
    private bool m_inMoveMode = false;

    private void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    private void CalculateCameraMovement()
    {
        float axisX = Input.GetAxis("Look X");
        float axisY = Input.GetAxis("Look Y");

        //Debug.Log($"{axisX}, {axisY}");

        if (Mathf.Abs(axisX) < float.Epsilon && Mathf.Abs(axisY) < float.Epsilon)
        {
            m_exieMoveModeCountdown -= Time.deltaTime;
            if (m_exieMoveModeCountdown <= 0.0f)
            {
                m_exieMoveModeCountdown = 0.0f;
                m_inMoveMode = false;
                m_rotMoveX = 0.0f;
                m_rotMoveY = 0.0f;
            }
            return;
        }
        else
        {
            m_exieMoveModeCountdown = durationExitMoveMode;
            m_inMoveMode = true;
        }

        m_rotMoveX += axisY * moveScale;
        m_rotMoveX = Mathf.Clamp(m_rotMoveX, -rotationClampX, rotationClampX);
        m_rotMoveY += axisX * moveScale;
    }

    void LateUpdate()
    {
        if (target == null)
            return;

        CalculateCameraMovement();

        float cameraRotX = cameraXRotation; //target.eulerAngles.x;
        float cameraRotY = target.eulerAngles.y;
        if (m_inMoveMode)
        {
            cameraRotX -= m_rotMoveX;
            cameraRotY += m_rotMoveY;
        }

        //Debug.Log($"Cam {cameraRotX}, {cameraRotY}");

        // Follow only the target's Y rotation, ignoring X/Z tilt.
        Quaternion rotation = Quaternion.Euler(0, cameraRotY, 0f);

        // Offset is applied relative to the target's XZ orientation.
        Vector3 desiredPosition = target.position + rotation * offset;
        //Vector3 finalPosition = Vector3.Lerp(camera.transform.position, desiredPosition, Time.deltaTime);
        Vector3 finalPosition = desiredPosition;

        camera.transform.position = finalPosition;

        // Camera keeps its own tunable X rotation, but follows target yaw.
        Quaternion desiredRotation = Quaternion.Euler(
            cameraRotX,
            cameraRotY,
            0f
        );
        //Quaternion finalRotation = Quaternion.Lerp(camera.transform.rotation, desiredRotation, Time.deltaTime);
        Quaternion finalRotation = desiredRotation;

        camera.transform.rotation = finalRotation;
    }
}