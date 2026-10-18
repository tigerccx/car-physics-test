using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BasicCamera : MonoBehaviour
{
    [Header("Camera")]
    public Camera camera;

    [Header("Target")]
    public Transform target;

    [Header("Follow Settings")]
    public Vector3 localOffset = new Vector3(0f, 3f, -7f);
    public float followSmoothness = 8f;
    public float rotationSmoothness = 8f;

    [Header("Look Settings")]
    public Vector3 lookOffset = new Vector3(0f, 1.2f, 0f);

    private void LateUpdate()
    {
        if (target == null)
            return;

        // Desired camera position relative to the car's rotation
        Vector3 desiredPosition = target.TransformPoint(localOffset);

        camera.transform.position = Vector3.Lerp(
            camera.transform.position,
            desiredPosition,
            followSmoothness * Time.deltaTime
        );

        Vector3 lookTarget = target.position + lookOffset;
        Vector3 lookDirection = lookTarget - camera.transform.position;

        if (lookDirection.sqrMagnitude > 0.001f)
        {
            Quaternion desiredRotation = Quaternion.LookRotation(lookDirection, Vector3.up);

            camera.transform.rotation = Quaternion.Slerp(
                camera.transform.rotation,
                desiredRotation,
                rotationSmoothness * Time.deltaTime
            );
        }
    }
}
