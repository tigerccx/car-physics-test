using UnityEngine;

using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CapsuleCollider))]
public class SimplePlayer : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Camera playerCamera;

    [Header("Movement")]
    [SerializeField] private float moveSpeed = 6.0f;
    [SerializeField] private float sprintSpeed = 9.0f;
    [SerializeField] private float groundAcceleration = 40.0f;
    [SerializeField] private float airAcceleration = 10.0f;
    [SerializeField] private float jumpHeight = 1.5f;
    [SerializeField] private float groundCheckDistance = 0.15f;
    [SerializeField] private LayerMask groundMask = ~0;

    [Header("Look")]
    [SerializeField] private float mouseSensitivity = 2.0f;
    [SerializeField] private float maxPitch = 85.0f;

    private Rigidbody rb;
    private CapsuleCollider capsule;

    private Vector2 moveInput;
    private bool jumpPressed;
    private float yaw;
    private float pitch;
    private Quaternion rotOriginal;
    private bool grounded;
    public Vector3 originalCameraPos_ls = new Vector3(0.0f, 0.0f, -15.0f);

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        capsule = GetComponent<CapsuleCollider>();

        if (playerCamera == null)
            playerCamera = GetComponentInChildren<Camera>();

        rb.freezeRotation = true;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        yaw = transform.eulerAngles.y;

        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;

        playerCamera.transform.position = transform.TransformPoint(originalCameraPos_ls);
        playerCamera.transform.LookAt(transform, Vector3.up);
    }

    private void Update()
    {
        ReadInput();

        HandleLook();
    }

    private void FixedUpdate()
    {
        grounded = CheckGrounded();

        ApplyRotation();
        ApplyMovement();

        if (jumpPressed)
        {
            TryJump();
            jumpPressed = false;
        }
    }

    private void ReadInput()
    {
        moveInput = new Vector2(
            Input.GetAxisRaw("Horizontal"),
            Input.GetAxisRaw("Vertical"));

        moveInput = Vector2.ClampMagnitude(moveInput, 1.0f);

        if (Input.GetButtonDown("Jump"))
            jumpPressed = true;
    }

    private void HandleLook()
    {
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;

        yaw += mouseX;

        pitch -= mouseY;
        pitch = Mathf.Clamp(pitch, -maxPitch, maxPitch);

        if (playerCamera != null)
        {
            Quaternion rot = Quaternion.Euler(pitch, yaw, 0.0f);

            playerCamera.transform.rotation = rot;
            Vector3 newPos = rot * originalCameraPos_ls + transform.position;
            playerCamera.transform.position = newPos;
        }
    }

    private void ApplyRotation()
    {
        Quaternion targetRotation = Quaternion.Euler(0.0f, yaw, 0.0f);
        rb.MoveRotation(targetRotation);
    }

    private void ApplyMovement()
    {
        Vector3 inputDir =
            transform.right * moveInput.x +
            transform.forward * moveInput.y;

        inputDir = Vector3.ClampMagnitude(inputDir, 1.0f);

        float targetSpeed = Input.GetKey(KeyCode.LeftShift) ? sprintSpeed : moveSpeed;
        Vector3 targetHorizontalVelocity = inputDir * targetSpeed;

        Vector3 velocity = GetVelocity();
        Vector3 currentHorizontalVelocity = new Vector3(velocity.x, 0.0f, velocity.z);

        Vector3 velocityDelta = targetHorizontalVelocity - currentHorizontalVelocity;

        float acceleration = grounded ? groundAcceleration : airAcceleration;
        Vector3 maxVelocityChange = Vector3.ClampMagnitude(
            velocityDelta,
            acceleration * Time.fixedDeltaTime);

        rb.AddForce(maxVelocityChange, ForceMode.VelocityChange);
    }

    private void TryJump()
    {
        if (!grounded)
            return;

        Vector3 velocity = GetVelocity();

        // Remove downward velocity before jumping.
        if (velocity.y < 0.0f)
        {
            velocity.y = 0.0f;
            SetVelocity(velocity);
        }

        float jumpSpeed = Mathf.Sqrt(2.0f * -Physics.gravity.y * jumpHeight);

        rb.AddForce(Vector3.up * jumpSpeed, ForceMode.VelocityChange);
    }

    private bool CheckGrounded()
    {
        float radius = capsule.radius * 0.95f;
        float halfHeight = Mathf.Max(capsule.height * 0.5f, radius);

        Vector3 center = transform.TransformPoint(capsule.center);

        Vector3 bottomSphereCenter =
            center + Vector3.down * (halfHeight - radius);

        return Physics.SphereCast(
            bottomSphereCenter + Vector3.up * 0.05f,
            radius,
            Vector3.down,
            out _,
            groundCheckDistance + 0.05f,
            groundMask,
            QueryTriggerInteraction.Ignore);
    }

    private Vector3 GetVelocity()
    {
#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity;
#else
        return rb.velocity;
#endif
    }

    private void SetVelocity(Vector3 velocity)
    {
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = velocity;
#else
        rb.velocity = velocity;
#endif
    }
}