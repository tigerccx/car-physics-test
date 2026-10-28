using System.Globalization;
using B3DVehicle;
using Box3D.Hybrid;
using UnityEditor;
using UnityEngine;

public class VehicleDebugWindow : EditorWindow
{
    [SerializeField] private B3DSimpleVehicle selectedVehicle;
    [SerializeField] private float defaultTractionImpulse = 100f;
    private GUISkin compactSkin;
    private GUIStyle compactBold, compactBoldTitle, compactMini, compactMiniBold, compactMiniBoldTitle, compactWhite, compactPanel;
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
        minSize = new Vector2(660f, 520f);
        EditorApplication.update += OnEditorUpdate;
    }

    private void OnDisable()
    {
        EditorApplication.update -= OnEditorUpdate;
        if (compactSkin) DestroyImmediate(compactSkin);
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
        if (!compactSkin)
        {
            compactSkin = Instantiate(GUI.skin);
            compactSkin.hideFlags = HideFlags.HideAndDontSave;
            compactSkin.label = CompactStyle(GUI.skin.label);
            compactBold = CompactStyle(EditorStyles.boldLabel);
            compactBoldTitle = CompactStyle(EditorStyles.boldLabel);
            compactBoldTitle.alignment = TextAnchor.MiddleCenter;
            compactMini = CompactStyle(EditorStyles.miniLabel);
            compactMiniBold = CompactStyle(EditorStyles.miniBoldLabel);
            compactMiniBoldTitle = CompactStyle(EditorStyles.miniBoldLabel);
            compactMiniBoldTitle.alignment = TextAnchor.MiddleCenter;
            compactWhite = CompactStyle(EditorStyles.whiteMiniLabel);
            compactPanel = new GUIStyle(EditorStyles.helpBox)
            {
                padding = new RectOffset(4, 4, 3, 3),
                margin = new RectOffset(2, 2, 2, 2)
            };
        }
        GUISkin previous = GUI.skin;
        try
        {
            GUI.skin = compactSkin;
            DrawWindowGUI();
        }
        finally { GUI.skin = previous; }
    }

    private static GUIStyle CompactStyle(GUIStyle source)
    {
        return new GUIStyle(source)
        {
            fontSize = 10,
            fixedHeight = 13f,
            margin = new RectOffset(1, 1, 0, 0),
            padding = new RectOffset(0, 0, 0, 0)
        };
    }

    private void DrawWindowGUI()
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
        Rect diagram = GUILayoutUtility.GetRect(630f, 220f, GUILayout.ExpandWidth(true));
        // Keep the diagram near the vehicle rather than distributing it across a wide window.
        float vehicleWidth = Mathf.Min(360f, diagram.width - 240f);
        Rect vehicleRect = new Rect(diagram.x, diagram.y, vehicleWidth, diagram.height);
        DrawDiagram(vehicleRect);
        DrawTractionCircles(new Rect(vehicleRect.xMax + 8f, diagram.y, 232f, diagram.height));
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.BeginVertical(GUILayout.Width(210f));
        DrawVehicleData();
        EditorGUILayout.BeginVertical(compactPanel, GUILayout.ExpandHeight(false));
        GUILayout.Label("---Debug Config---", compactBoldTitle);
        GUILayout.Label("Default traction impulse (N·s)");
        float configured = EditorGUILayout.FloatField(defaultTractionImpulse);
        defaultTractionImpulse = float.IsNaN(configured) || float.IsInfinity(configured) ? 100f : Mathf.Max(0.00001f, configured);
        GUILayout.Label("Display reference only", compactMini);
        GUILayout.Space(4f);
        EditorGUI.BeginChangeCheck();
        bool useLockedSteering = EditorGUILayout.ToggleLeft("Use Locked Steering", selectedVehicle.m_debugUseLockedSteering);
        GUILayout.Label("Locked Steering");
        float lockedSteering = EditorGUILayout.Slider(selectedVehicle.m_debugLockedSteering, -1f, 1f);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(selectedVehicle, "Change debug steering");
            selectedVehicle.m_debugUseLockedSteering = useLockedSteering;
            selectedVehicle.m_debugLockedSteering = lockedSteering;
            EditorUtility.SetDirty(selectedVehicle);
            PrefabUtility.RecordPrefabInstancePropertyModifications(selectedVehicle);
        }
        EditorGUILayout.EndVertical();
        // Absorb the wheel columns' extra height outside the Config panel.
        GUILayout.FlexibleSpace();
        EditorGUILayout.EndVertical();
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
        target.SpeedKmH = target.Speed * 3.6f;
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
            result.DesiredLateral = wheelState.m_desiredLateralImpulse;
            result.Longitudinal = wheelState.m_currentLongitudinalImpulse;
            result.DesiredLongitudinal = wheelState.m_desiredLongitudinalImpulse;
            result.AvailableGrip = Mathf.Max(0f, wheelState.m_availableGripImpulse);
            result.UsedGrip = new Vector2(result.LateralImpulse, result.Longitudinal).magnitude;
            result.LinearSpeed = wheelState.m_currentAngularVelocity * wheel.m_wheelProperties.m_wheelRadius;
            result.GroundSpeed = wheelState.m_currentVelocityLongitudinal;
            result.SlipRatio = wheelState.m_currentSlipRatio;
            result.Torque = wheelState.m_currentTorque;
            result.Load = Mathf.Max(0f, suspensionState.m_currentSuspensionForce);
            result.LateralSign = Vector3.Dot(wheelState.m_currentDirLateral_ws,
                vehicleTransform.TransformDirection(Vector3.Cross(Vector3.up, result.LocalForward))) >= 0f ? 1f : -1f;
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
        EditorGUILayout.BeginVertical(compactPanel, GUILayout.Width(210f), GUILayout.MinHeight(250f));
        GUILayout.Label("---Vehicle---", compactBoldTitle);
        Value("Speed (m/s)", data.Speed);
        Value("Speed (km/h)", data.SpeedKmH);
        VectorValue("Direction (world, unit vector)", data.WorldDirection);
        Value("Local XZ speed (m/s)", new Vector2(data.LocalVelocity.x, data.LocalVelocity.z).magnitude);
        Value("Angular speed (rad/s)", data.AngularSpeed);
        VectorValue("Rotation axis (world, unit vector)", data.WorldRotationAxis);
        Value("Local Y angular speed (rad/s)", data.LocalYawSpeed);
        GUILayout.Space(5f);
        GUILayout.Label("---Diagram---", compactBoldTitle);
        GUILayout.Label("Top: local +Z    Right: local +X");
        Legend("North (world +Z)", NorthColor);
        Legend("Wheel forward", WheelColor);
        Legend("Local XZ velocity", VelocityColor);
        Legend("Local Y rotation", YawColor);
        GUILayout.Label("Arrow lengths are normalized.", compactMini);
        EditorGUILayout.EndVertical();
    }

    private void DrawWheelData(int index)
    {
        WheelDebugData wheel = data.Wheels[index];
        EditorGUILayout.BeginVertical(compactPanel, GUILayout.MinWidth(205f), GUILayout.ExpandWidth(true));
        GUILayout.Label($"---{WheelNames[index]}---", compactBoldTitle);
        if (!wheel.HasPhysicsData)
            GUILayout.Label(data.HasPhysicsData ? wheel.Status : "Physics unavailable", compactMini);
        GUILayout.Label("Wheel Data", compactMiniBoldTitle);
        Value("Turn angle (deg)", wheel.TurnAngle, wheel.HasPhysicsData);
        Value("Slip angle (deg)", wheel.SlipAngle, wheel.HasPhysicsData);
        DrawImpulsePair("Lateral impulse (N·s)", wheel.LateralImpulse, wheel.DesiredLateral, wheel.HasPhysicsData);
        GUILayout.Space(2f);
        Value("Linear V (m/s)", wheel.LinearSpeed, wheel.HasPhysicsData);
        Value("Ground V (m/s)", wheel.GroundSpeed, wheel.HasPhysicsData);
        Value("Slip ratio (unitless)", wheel.SlipRatio, wheel.HasPhysicsData);
        Value("Torque (N·m)", wheel.Torque, wheel.HasPhysicsData);
        DrawImpulsePair("Longitudinal impulse (N·s)", wheel.Longitudinal, wheel.DesiredLongitudinal, wheel.HasPhysicsData);
        GUILayout.Space(2f);
        Value("Load (N)", wheel.Load, wheel.HasPhysicsData);
        Value("Available grip (N·s)", wheel.AvailableGrip, wheel.HasPhysicsData);
        Value("Used grip (N·s)", wheel.UsedGrip, wheel.HasPhysicsData);
        GUILayout.Space(2f);
        GUILayout.Label("Suspension Data", compactMiniBoldTitle);
        Value("Axial velocity (m/s)", wheel.AxialVelocity, wheel.HasPhysicsData);
        Value("Length (m)", wheel.Length, wheel.HasPhysicsData);
        Value("Mount to ground (m)", wheel.GroundDistance, wheel.HasPhysicsData && wheel.Grounded);
        Value("Damper force (N)", wheel.DamperForce, wheel.HasPhysicsData);
        Value("Suspension force (N)", wheel.SuspensionForce, wheel.HasPhysicsData);
        Row("Grounded", wheel.HasPhysicsData ? (wheel.Grounded ? "Yes" : "No") : "--");
        EditorGUILayout.EndVertical();
    }

    private void DrawImpulsePair(string label, float actual, float desired, bool available)
    {
        GUILayout.Label(label + " (actual / desired)");
        GUILayout.Label(data.HasPhysicsData && available
            ? Format(actual) + " / " + Format(desired) : "--");
    }

    private void Value(string label, float value, bool available = true)
    {
        Row(label, data.HasPhysicsData && available ? Format(value) : "--");
    }

    private void VectorValue(string label, Vector3 value)
    {
        GUILayout.Label(label);
        GUILayout.Label(data.HasPhysicsData
            ? $"({Format(value.x)}, {Format(value.y)}, {Format(value.z)})" : "--", compactMini);
    }

    private static string Format(float value) => value.ToString("F3", CultureInfo.InvariantCulture);

    private static void Row(string label, string value)
    {
        EditorGUILayout.BeginHorizontal();
        GUILayout.Label(label);
        GUILayout.FlexibleSpace();
        GUILayout.Label(value);
        EditorGUILayout.EndHorizontal();
    }

    private void Legend(string label, Color color)
    {
        Rect rect = GUILayoutUtility.GetRect(12f, 14f);
        EditorGUI.DrawRect(new Rect(rect.x, rect.y + 3f, 7f, 7f), color);
        GUI.Label(new Rect(rect.x + 16f, rect.y, rect.width - 16f, rect.height), label);
    }

    private void DrawDiagram(Rect rect)
    {
        if (Event.current.type != EventType.Repaint) return;
        EditorGUI.DrawRect(rect, new Color(0.12f, 0.13f, 0.15f));
        Vector2 center = rect.center;
        Rect body = new Rect(center.x - 50f, center.y - 75f, 100f, 150f);
        GUI.Box(body, GUIContent.none, compactPanel);
        GUI.Label(new Rect(center.x - 16f, body.y + 8f, 80f, 20f), "FRONT", compactWhite);
        GUI.Label(new Rect(rect.x + 12f, rect.y + 8f, 400f, 20f), "Vehicle local XZ plane (direction only)", compactWhite);
        Handles.BeginGUI();
        Color previous = Handles.color;
        try
        {
            Vector2 northOrigin = new Vector2(rect.x + 60f, center.y);
            Arrow(northOrigin, Project(data.LocalNorth), 50f, NorthColor);
            GUI.Label(new Rect(northOrigin.x - 45f, northOrigin.y + 68f, 180f, 32f),
                Project(data.LocalNorth).sqrMagnitude > 0.000001f ? "North (world +Z)" : "North: XZ projection is zero", compactWhite);
            for (int i = 0; i < 4; i++)
            {
                Vector2 wheelCenter = center + new Vector2(i % 2 == 0 ? -45f : 45f, i < 2 ? -50f : 50f);
                Vector2 forward = Project(data.Wheels[i].LocalForward).normalized;
                if (forward.sqrMagnitude < 0.000001f) forward = Vector2.up * -1f;
                Vector2 side = new Vector2(-forward.y, forward.x) * 10f;
                Vector2 along = forward * 17f;
                Handles.color = WheelColor;
                Handles.DrawAAPolyLine(2f, wheelCenter - side - along, wheelCenter + side - along,
                    wheelCenter + side + along, wheelCenter - side + along, wheelCenter - side - along);
                Arrow(wheelCenter, forward, 38f, WheelColor);
                GUI.Label(new Rect(wheelCenter.x - 5f, wheelCenter.y + 25f, 40f, 20f),
                    new[] { "LF", "RF", "LR", "RR" }[i], compactWhite);
            }
            if (data.HasPhysicsData)
            {
                Arrow(center, Project(data.LocalVelocity), 55f, VelocityColor);
                if (Mathf.Abs(data.LocalYawSpeed) > 0.001f)
                {
                    // Share the velocity origin; positive local yaw turns the nose right.
                    Arrow(center, Vector2.right * Mathf.Sign(data.LocalYawSpeed), 42f, YawColor);
                }
            }
        }
        finally
        {
            Handles.color = previous;
            Handles.EndGUI();
        }
    }

    private void DrawTractionCircles(Rect rect)
    {
        if (Event.current.type != EventType.Repaint) return;
        Handles.BeginGUI();
        Color previous = Handles.color;
        try
        {
            for (int i = 0; i < 4; i++)
            {
                WheelDebugData wheel = data.Wheels[i];
                Rect cell = new Rect(rect.x + (i % 2) * rect.width * 0.5f,
                    rect.y + (i / 2) * rect.height * 0.5f, rect.width * 0.5f, rect.height * 0.5f);
                // Reserve separate bands for the title, F label, circle, and B label.
                Vector2 center = new Vector2(cell.center.x, cell.y + 60f);
                float limit = Mathf.Min(cell.width * 0.22f, 24f);
                float available = wheel.HasPhysicsData ? wheel.AvailableGrip : 0f;
                // Both circles share a scale; zoom out together if grip exceeds the reference.
                float scale = limit / Mathf.Max(defaultTractionImpulse, available, 0.00001f);
                Handles.color = Color.gray;
                Handles.DrawLine(center - Vector2.right * (limit + 8f), center + Vector2.right * (limit + 8f));
                Handles.DrawLine(center - Vector2.up * (limit + 8f), center + Vector2.up * (limit + 8f));
                Handles.DrawWireDisc(center, Vector3.forward, defaultTractionImpulse * scale);
                GUI.Label(new Rect(cell.x + 8f, cell.y + 2f, cell.width - 16f, 13f), WheelNames[i], compactMini);
                GUI.Label(new Rect(center.x - 4f, center.y - limit - 18f, 16f, 13f), "F");
                GUI.Label(new Rect(center.x - 4f, center.y + limit + 9f, 16f, 13f), "B");
                GUI.Label(new Rect(center.x - limit - 20f, center.y - 9f, 20f, 18f), "L");
                GUI.Label(new Rect(center.x + limit + 9f, center.y - 9f, 20f, 18f), "R");
                if (!wheel.HasPhysicsData) continue;
                Color blue = new Color(0.2f, 0.6f, 1f);
                Handles.color = blue;
                Handles.DrawWireDisc(center, Vector3.forward, available * scale);
                Vector2 actual = Vector2.ClampMagnitude(new Vector2(wheel.LateralImpulse * wheel.LateralSign, -wheel.Longitudinal), available);
                Vector2 desired = new Vector2(wheel.DesiredLateral * wheel.LateralSign, -wheel.DesiredLongitudinal);
                Arrow(center, desired, Mathf.Min(desired.magnitude * scale, limit + 15f), desired.magnitude > available ? Color.red : blue);
                Handles.color = blue;
                Handles.DrawSolidDisc(center + actual * scale, Vector3.forward, 3f);
            }
        }
        finally { Handles.color = previous; Handles.EndGUI(); }
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
        public float Speed = 0f, SpeedKmH = 0f, AngularSpeed = 0f, LocalYawSpeed = 0f;
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
        public float DesiredLateral, Longitudinal, DesiredLongitudinal, AvailableGrip, UsedGrip;
        public float LinearSpeed, GroundSpeed, SlipRatio, Torque, Load, LateralSign;
        public bool Grounded = false;
    }
}


