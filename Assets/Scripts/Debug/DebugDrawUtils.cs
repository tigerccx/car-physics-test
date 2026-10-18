using UnityEngine;

public static class DebugDraw
{
    public static void DrawArrow(Vector3 start, Vector3 direction, Color color, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20f)
    {
        Gizmos.color = color;

        Vector3 end = start + direction;
        Gizmos.DrawLine(start, end);

        if (direction.sqrMagnitude < 0.0001f)
            return;

        Vector3 right = Quaternion.LookRotation(direction) *
                        Quaternion.Euler(0, 180 + arrowHeadAngle, 0) *
                        Vector3.forward;

        Vector3 left = Quaternion.LookRotation(direction) *
                       Quaternion.Euler(0, 180 - arrowHeadAngle, 0) *
                       Vector3.forward;

        Gizmos.DrawLine(end, end + right * arrowHeadLength);
        Gizmos.DrawLine(end, end + left * arrowHeadLength);
    }

    public static void DrawCylinder(
        Vector3 center,
        Quaternion rotation,
        float radius,
        float height,
        Color color,
        float duration = 0f,
        bool depthTest = true,
        int segments = 24)
    {
        segments = Mathf.Max(segments, 3);

        Vector3 up = rotation * Vector3.up;
        Vector3 right = rotation * Vector3.right;
        Vector3 forward = rotation * Vector3.forward;

        Vector3 topCenter = center + up * height * 0.5f;
        Vector3 bottomCenter = center - up * height * 0.5f;

        Vector3 prevTop = topCenter + right * radius;
        Vector3 prevBottom = bottomCenter + right * radius;

        for (int i = 1; i <= segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;

            Vector3 offset =
                (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;

            Vector3 currTop = topCenter + offset;
            Vector3 currBottom = bottomCenter + offset;

            // Top circle
            Debug.DrawLine(prevTop, currTop, color, duration, depthTest);

            // Bottom circle
            Debug.DrawLine(prevBottom, currBottom, color, duration, depthTest);

            if (height >= 0.0001f)
            {
                // Side line
                Debug.DrawLine(currTop, currBottom, color, duration, depthTest);
            }

            prevTop = currTop;
            prevBottom = currBottom;
        }
    }

    public static void DrawCylinder(
        Vector3 start,
        Vector3 end,
        float radius,
        Color color,
        float duration = 0f,
        bool depthTest = true,
        int segments = 24)
    {
        segments = Mathf.Max(segments, 3);

        Vector3 axis = end - start;
        float height = axis.magnitude;

        if (height <= Mathf.Epsilon)
            return;

        Vector3 up = axis / height;
        Vector3 arbitrary = Mathf.Abs(Vector3.Dot(up, Vector3.up)) > 0.99f
            ? Vector3.right
            : Vector3.up;

        Vector3 right = Vector3.Cross(up, arbitrary).normalized;
        Vector3 forward = Vector3.Cross(right, up).normalized;

        Vector3 prevStart = start + right * radius;
        Vector3 prevEnd = end + right * radius;

        for (int i = 1; i <= segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;

            Vector3 offset =
                (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;

            Vector3 currStart = start + offset;
            Vector3 currEnd = end + offset;

            // Start circle
            Debug.DrawLine(prevStart, currStart, color, duration, depthTest);

            // End circle
            Debug.DrawLine(prevEnd, currEnd, color, duration, depthTest);

            if (height >= 0.0001f)
            {
                // Side line
                Debug.DrawLine(currStart, currEnd, color, duration, depthTest);
            }

            prevStart = currStart;
            prevEnd = currEnd;
        }
    }

    public static void DrawSuspension(Vector3 start, Vector3 end, Color color)
    {
        Gizmos.color = color;
        float radius = 0.1f;

        DrawCylinder(start, end, radius, color);
    }

    public static void DrawWheel(Vector3 pos, Quaternion rot, float rad, Color color)
    {
        Gizmos.color = color;
        float height = 0.05f;
        DrawCylinder(pos, rot, rad, height, color);
    }
}