using System.Globalization;
using B3DVehicle;
using Box3D.Hybrid;
using UnityEditor;
using UnityEngine;

public class VehicleDebugWindow : EditorWindow
{
    [SerializeField] private B3DSimpleVehicle selectedVehicle;
    private Vector2 scroll;
    private double nextRepaintTime;
    private DebugData data = new DebugData();
    private static readonly string[] WheelNames = { "Left Front", "Right Front", "Left Rear", "Right Rear" };
    private static readonly Color NorthColor = new Color(0.45f, 0.8f, 1f);
    private static readonly Color WheelColor = new Color(1f, 0.8f, 0.25f);
    private static readonly Color VelocityColor = new Color(0.3f, 1f, 0.6f);
    private static readonly Color YawColor = new Color(1f, 0.5f, 0.75f);

    [MenuItem("Tools/Vehicle Debug")]
    public static void Open()
    {
        GetWindow<VehicleDebugWindow>("Vehicle Debug");
    }

    private void OnEnable()
    {
        minSize = new Vector2(760f, 650f);
        EditorApplication.update += OnEditorUpdate;
    }

    private void OnDisable()
    {
        EditorApplication.update -= OnEditorUpdate;
    }

    private void OnEditorUpdate()
    {
        // Keep the diagram current in both Edit mode and Play mode.
        if (EditorApplication.timeSinceStartup < nextRepaintTime) return;
        nextRepaintTime = EditorApplication.timeSinceStartup + 0.05;
        Repaint();
    }

    private void OnGUI()
    {
        DrawVehicleSelector();
        if (!selectedVehicle)
        {
            EditorGUILayout.HelpBox("Select a B3DSimpleVehicle in a loaded scene to inspect it.", MessageType.Info);
            return;
        }

        PopulateDebugData(selectedVehicle, data);
        scroll = EditorGUILayout.BeginScrollView(scroll);
        EditorGUILayout.HelpBox(data.HasPhysicsData
            ? "Live physics data. Arrows show direction only. -- means unavailable. Wheels map by local mount positions."
            : "Enter Play mode with an initialized Box3D body. -- means unavailable; wheel poses are schematic.",
            MessageType.Info);
        Rect diagram = GUILayoutUtility.GetRect(720f, 310f, GUILayout.ExpandWidth(true));
        DrawDiagram(diagram);
        EditorGUILayout.BeginHorizontal();
        DrawVehicleData();
        EditorGUILayout.BeginVertical(GUILayout.ExpandWidth(true));
        EditorGUILayout.BeginHorizontal();
        DrawWheelData(0);
        DrawWheelData(1);
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        DrawWheelData(2);
        DrawWheelData(3);
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.EndVertical();
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.EndScrollView();
    }

    private void DrawVehicleSelector()
    {
        EditorGUILayout.BeginHorizontal(EditorStyles.toolbar);
        GUILayout.Label("Select Vehicle", GUILayout.Width(90f));
        string title = selectedVehicle ? selectedVehicle.name : "Select a scene vehicle...";
        if (EditorGUILayout.DropdownButton(new GUIContent(title), FocusType.Keyboard, EditorStyles.toolbarDropDown))
        {
            GenericMenu menu = new GenericMenu();
            menu.AddItem(new GUIContent("None"), !selectedVehicle, () => selectedVehicle = null);
            B3DSimpleVehicle[] vehicles = Object.FindObjectsByType<B3DSimpleVehicle>(FindObjectsInactive.Include, FindObjectsSortMode.InstanceID);
            foreach (B3DSimpleVehicle vehicle in vehicles)
            {
                if (!vehicle.gameObject.scene.IsValid() || EditorUtility.IsPersistent(vehicle)) continue;
                B3DSimpleVehicle candidate = vehicle;
                string path = candidate.name;
                for (Transform parent = candidate.transform.parent; parent; parent = parent.parent)
                    path = parent.name + "/" + path;
                string label = candidate.gameObject.scene.name + "/" + path + " [" + candidate.GetInstanceID() + "]";
                menu.AddItem(new GUIContent(label), selectedVehicle == candidate, () => selectedVehicle = candidate);
            }
            menu.DropDown(GUILayoutUtility.GetLastRect());
        }
        if (GUILayout.Button("Ping", EditorStyles.toolbarButton, GUILayout.Width(42f)) && selectedVehicle)
            EditorGUIUtility.PingObject(selectedVehicle);
        EditorGUILayout.EndHorizontal();
    }

    // All data acquisition belongs here. Drawing code only consumes this snapshot.
    private static void PopulateDebugData(B3DSimpleVehicle vehicle, DebugData target)
    {
        target.HasPhysicsData = false;
        target.LocalNorth = vehicle.transform.InverseTransformDirection(Vector3.forward);
        foreach (WheelDebugData wheel in target.Wheels)
        {
            wheel.HasPhysicsData = false;
            wheel.LocalForward = Vector3.forward;
            wheel.MountCount = 0;
            wheel.Status = "Missing or ambiguous mount";
        }

        // Native velocity getters require a live body; do not call them in Edit mode.
        Box3DBody body = vehicle.GetComponent<Box3DBody>();
        if (!Application.isPlaying || !body || body.Body.Id.IsNull || !body.Body.IsValid) return;

        Transform vehicleTransform = vehicle.transform;
        Vector3 velocity = body.linearVelocity;
        Vector3 angularVelocity = body.angularVelocity;
        target.Speed = velocity.magnitude;
        target.WorldDirection = velocity.normalized;
        target.LocalVelocity = vehicleTransform.InverseTransformDirection(velocity);
        target.AngularSpeed = angularVelocity.magnitude;
        target.WorldRotationAxis = angularVelocity.normalized;
        target.LocalYawSpeed = vehicleTransform.InverseTransformDirection(angularVelocity).y;
        target.HasPhysicsData = true;

        Suspension[] suspensions = vehicle.GetComponentsInChildren<Suspension>();
        Vector3 min = new Vector3(float.PositiveInfinity, 0f, float.PositiveInfinity);
        Vector3 max = new Vector3(float.NegativeInfinity, 0f, float.NegativeInfinity);
        int mountCount = 0;
        foreach (Suspension suspension in suspensions)
        {
            if (suspension.GetComponentInParent<B3DSimpleVehicle>() != vehicle) continue;
            Vector3 mount = vehicleTransform.InverseTransformPoint(suspension.transform.position);
            min = Vector3.Min(min, mount);
            max = Vector3.Max(max, mount);
            mountCount++;
        }
        if (mountCount == 0) return;
        // Bounds-based mapping supports an offset vehicle origin; +Z is front, +X right.
        Vector3 midpoint = (min + max) * 0.5f;
        foreach (Suspension suspension in suspensions)
        {
            if (suspension.GetComponentInParent<B3DSimpleVehicle>() != vehicle) continue;
            Vector3 mount = vehicleTransform.InverseTransformPoint(suspension.transform.position);
            if (Mathf.Abs(mount.x - midpoint.x) < 0.00001f ||
                Mathf.Abs(mount.z - midpoint.z) < 0.00001f) continue;
            int index = (mount.z > midpoint.z ? 0 : 2) + (mount.x < midpoint.x ? 0 : 1);
            WheelDebugData result = target.Wheels[index];
            if (++result.MountCount > 1)
            {
                result.HasPhysicsData = false;
                result.LocalForward = Vector3.forward;
                result.Status = "Multiple mounts in this corner";
                continue;
            }
            Wheel wheel = suspension.GetComponent<Wheel>();
            SuspensionState suspensionState = suspension.m_suspensionState;
            WheelState wheelState = wheel ? wheel.m_wheelState : null;
            if (suspensionState == null || wheelState == null ||
                wheelState.m_currentDirFoward_ws.sqrMagnitude < 0.000001f ||
                suspensionState.m_currentDir_ws.sqrMagnitude < 0.000001f)
            {
                result.Status = "Wheel / suspension not initialized";
                continue;
            }

            result.LocalForward = vehicleTransform.InverseTransformDirection(wheelState.m_currentDirFoward_ws).normalized;
            result.SlipAngle = wheelState.m_currentSlipAngle; // Already in degrees.
            result.TurnAngle = Mathf.Atan2(result.LocalForward.x, result.LocalForward.z) * Mathf.Rad2Deg;
            result.LateralImpulse = wheelState.m_currentLateralImpulse;
            result.AxialVelocity = suspensionState.m_currentSuspensionVelocity;
            result.Length = suspensionState.m_currentSuspensionLength;
            result.Grounded = suspensionState.m_isGrounded;
            // Actual ray distance from suspension attachment, not clamped travel + radius.
            // The stored hit point is stale when airborne; hide distance in that case.
            result.GroundDistance = result.Grounded
                ? Vector3.Distance(suspensionState.m_currentVehicleAttachPoint_ws, suspensionState.m_posHitGround_ws)
                : 0f;
            result.DamperForce = suspensionState.m_currentSuspensionDamperForce;
            result.SuspensionForce = suspensionState.m_currentSuspensionForce;
            result.HasPhysicsData = true;
        }
    }

    private void DrawVehicleData()
    {
        EditorGUILayout.BeginVertical(EditorStyles.helpBox, GUILayout.Width(270f), GUILayout.MinHeight(480f));
        GUILayout.Label("Vehicle", EditorStyles.boldLabel);
        Value("Speed (m/s)", data.Speed);
        VectorValue("Direction (world, unit vector)", data.WorldDirection);
        Value("Local XZ speed (m/s)", new Vector2(data.LocalVelocity.x, data.LocalVelocity.z).magnitude);
        Value("Angular speed (rad/s)", data.AngularSpeed);
        VectorValue("Rotation axis (world, unit vector)", data.WorldRotationAxis);
        Value("Local Y angular speed (rad/s)", data.LocalYawSpeed);
        GUILayout.Space(12f);
        GUILayout.Label("Diagram", EditorStyles.boldLabel);
        GUILayout.Label("Top: local +Z    Right: local +X");
        Legend("North (world +Z)", NorthColor);
        Legend("Wheel forward", WheelColor);
        Legend("Local XZ velocity", VelocityColor);
        Legend("Local Y rotation", YawColor);
        GUILayout.Label("Arrow lengths are normalized.", EditorStyles.miniLabel);
        EditorGUILayout.EndVertical();
    }

    private void DrawWheelData(int index)
    {
        WheelDebugData wheel = data.Wheels[index];
        EditorGUILayout.BeginVertical(EditorStyles.helpBox, GUILayout.MinWidth(230f), GUILayout.ExpandWidth(true));
        GUILayout.Label(WheelNames[index], EditorStyles.boldLabel);
        if (!wheel.HasPhysicsData)
            GUILayout.Label(data.HasPhysicsData ? wheel.Status : "Physics unavailable", EditorStyles.miniLabel);
        GUILayout.Label("Wheel Data", EditorStyles.miniBoldLabel);
        Value("Slip angle (deg)", wheel.SlipAngle, wheel.HasPhysicsData);
        Value("Lateral impulse (N·s)", wheel.LateralImpulse, wheel.HasPhysicsData);
        Value("Turn angle (deg)", wheel.TurnAngle, wheel.HasPhysicsData);
        GUILayout.Space(5f);
        GUILayout.Label("Suspension Data", EditorStyles.miniBoldLabel);
        Value("Axial velocity (m/s)", wheel.AxialVelocity, wheel.HasPhysicsData);
        Value("Length (m)", wheel.Length, wheel.HasPhysicsData);
        Value("Mount to ground (m)", wheel.GroundDistance, wheel.HasPhysicsData && wheel.Grounded);
        Value("Damper force (N)", wheel.DamperForce, wheel.HasPhysicsData);
        Value("Suspension force (N)", wheel.SuspensionForce, wheel.HasPhysicsData);
        Row("Grounded", wheel.HasPhysicsData ? (wheel.Grounded ? "Yes" : "No") : "--");
        EditorGUILayout.EndVertical();
    }

    private void Value(string label, float value, bool available = true)
    {
        Row(label, data.HasPhysicsData && available ? Format(value) : "--");
    }

    private void VectorValue(string label, Vector3 value)
    {
        GUILayout.Label(label);
        GUILayout.Label(data.HasPhysicsData
            ? $"({Format(value.x)}, {Format(value.y)}, {Format(value.z)})" : "--", EditorStyles.miniLabel);
    }

    private static string Format(float value) => value.ToString("F5", CultureInfo.InvariantCulture);

    private static void Row(string label, string value)
    {
        EditorGUILayout.BeginHorizontal();
        GUILayout.Label(label);
        GUILayout.FlexibleSpace();
        GUILayout.Label(value);
        EditorGUILayout.EndHorizontal();
    }

    private static void Legend(string label, Color color)
    {
        Rect rect = GUILayoutUtility.GetRect(12f, 20f);
        EditorGUI.DrawRect(new Rect(rect.x, rect.y + 5f, 9f, 9f), color);
        GUI.Label(new Rect(rect.x + 16f, rect.y, rect.width - 16f, rect.height), label);
    }

    private void DrawDiagram(Rect rect)
    {
        if (Event.current.type != EventType.Repaint) return;
        EditorGUI.DrawRect(rect, new Color(0.12f, 0.13f, 0.15f));
        Vector2 center = rect.center;
        Rect body = new Rect(center.x - 57f, center.y - 108f, 114f, 216f);
        GUI.Box(body, GUIContent.none, EditorStyles.helpBox);
        GUI.Label(new Rect(center.x - 28f, body.y + 8f, 80f, 20f), "FRONT", EditorStyles.whiteMiniLabel);
        GUI.Label(new Rect(rect.x + 12f, rect.y + 8f, 400f, 20f), "Vehicle local XZ plane (direction only)", EditorStyles.whiteMiniLabel);
        Handles.BeginGUI();
        Color previous = Handles.color;
        try
        {
            Vector2 northOrigin = new Vector2(rect.x + 85f, center.y);
            Arrow(northOrigin, Project(data.LocalNorth), 60f, NorthColor);
            GUI.Label(new Rect(northOrigin.x - 65f, northOrigin.y + 68f, 180f, 32f),
                Project(data.LocalNorth).sqrMagnitude > 0.000001f ? "North (world +Z)" : "North: XZ projection is zero", EditorStyles.whiteMiniLabel);
            for (int i = 0; i < 4; i++)
            {
                Vector2 wheelCenter = center + new Vector2(i % 2 == 0 ? -59f : 59f, i < 2 ? -70f : 70f);
                Vector2 forward = Project(data.Wheels[i].LocalForward).normalized;
                if (forward.sqrMagnitude < 0.000001f) forward = Vector2.up * -1f;
                Vector2 side = new Vector2(-forward.y, forward.x) * 10f;
                Vector2 along = forward * 23f;
                Handles.color = WheelColor;
                Handles.DrawAAPolyLine(2f, wheelCenter - side - along, wheelCenter + side - along,
                    wheelCenter + side + along, wheelCenter - side + along, wheelCenter - side - along);
                Arrow(wheelCenter, forward, 52f, WheelColor);
                GUI.Label(new Rect(wheelCenter.x - 10f, wheelCenter.y + 25f, 40f, 20f),
                    new[] { "LF", "RF", "LR", "RR" }[i], EditorStyles.whiteMiniLabel);
            }
            if (data.HasPhysicsData)
            {
                Arrow(center, Project(data.LocalVelocity), 72f, VelocityColor);
                if (Mathf.Abs(data.LocalYawSpeed) > 0.000001f)
                {
                    float sign = Mathf.Sign(data.LocalYawSpeed);
                    Vector2 arcCenter = center + new Vector2(165f, 0f);
                    Vector3[] points = new Vector3[33];
                    for (int i = 0; i < points.Length; i++)
                    {
                        float angle = (-120f + sign * 240f * i / (points.Length - 1)) * Mathf.Deg2Rad;
                        points[i] = arcCenter + new Vector2(Mathf.Sin(angle), -Mathf.Cos(angle)) * 38f;
                    }
                    Handles.color = YawColor;
                    Handles.DrawAAPolyLine(3f, points);
                    Arrow(points[31], points[32] - points[31], Vector2.Distance(points[31], points[32]), YawColor);
                    GUI.Label(new Rect(arcCenter.x - 35f, arcCenter.y + 50f, 150f, 20f), "Local Y rotation", EditorStyles.whiteMiniLabel);
                }
            }
        }
        finally
        {
            Handles.color = previous;
            Handles.EndGUI();
        }
    }

    private static Vector2 Project(Vector3 local) => new Vector2(local.x, -local.z);

    private static void Arrow(Vector2 start, Vector2 direction, float length, Color color)
    {
        if (direction.sqrMagnitude < 0.000001f) return;
        Vector2 forward = direction.normalized;
        Vector2 tip = start + forward * length;
        Vector2 side = new Vector2(-forward.y, forward.x);
        Handles.color = color;
        Handles.DrawAAPolyLine(2.5f, start, tip);
        Handles.DrawAAPolyLine(2.5f, tip - forward * 10f + side * 5f, tip, tip - forward * 10f - side * 5f);
    }

    private sealed class DebugData
    {
        public bool HasPhysicsData;
        public Vector3 LocalNorth;
        public float Speed = 0f, AngularSpeed = 0f, LocalYawSpeed = 0f;
        public Vector3 WorldDirection = Vector3.zero, LocalVelocity = Vector3.zero, WorldRotationAxis = Vector3.zero;
        public readonly WheelDebugData[] Wheels =
        {
            new WheelDebugData(), new WheelDebugData(), new WheelDebugData(), new WheelDebugData()
        };
    }

    private sealed class WheelDebugData
    {
        public bool HasPhysicsData;
        public int MountCount;
        public string Status;
        public Vector3 LocalForward = Vector3.forward;
        public float SlipAngle = 0f, LateralImpulse = 0f, TurnAngle = 0f;
        public float AxialVelocity = 0f, Length = 0f, GroundDistance = 0f, DamperForce = 0f, SuspensionForce = 0f;
        public bool Grounded = false;
    }
}


