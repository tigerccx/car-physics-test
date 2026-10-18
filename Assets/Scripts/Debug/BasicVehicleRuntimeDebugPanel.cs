using System;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.Rendering.DebugUI;

[DefaultExecutionOrder(1000)]
public class BasicVehicleRuntimeDebugPanel : MonoBehaviour
{
    [Header("Target")]
    [SerializeField] public BasicVehicle m_vehicle;

    [Header("Panel")]
    [SerializeField] private KeyCode m_toggleKey = KeyCode.F8;
    [SerializeField] private float m_panelWidth = 560f;
    [SerializeField] private float m_topMargin = 20f;
    [SerializeField] private float m_rightMargin = 20f;
    [SerializeField] private float m_bottomMargin = 20f;
    [SerializeField] private float m_graphHeight = 90f;
    private Rect m_panelRect;
    private CursorLockMode m_previousLockState;
    private bool m_previousCursorVisible;

    [Header("Sampling")]
    [SerializeField] private int m_sampleCapacity = 600;
    [SerializeField] private bool m_useUnscaledTime = true;

    private readonly List<TrackedValue> m_values = new();
    private readonly RingBuffer<float> m_timeSamples = new(1);

    private bool m_isOpen = false;
    private Vector2 m_scroll;

    [Header("Vehicle Selection")]
    [SerializeField] private bool autoRefreshVehicles = true;
    [SerializeField] private float vehicleRefreshInterval = 1.0f;
    [SerializeField] private bool clearHistoryWhenVehicleChanges = true;

    private readonly List<BasicVehicle> _availableVehicles = new();
    private string[] _vehicleNames = Array.Empty<string>();

    private bool _vehicleDropdownOpen;

    [Header("Tracked Values")]
    private DrivingGear m_currentGear = null;
    //private readonly TrackedVector3 m_worldPosition =
    //    new("Transform / World Position", "m");
    private readonly TrackedFloat m_vehicleSpeed =
        new("Vehicle / Speed", "km/h");
    private readonly TrackedFloat m_engineRPM =
        new("Vehicle / EngineRPM", "r/s");
    private readonly TrackedFloat m_engineTorque =
        new("Vehicle / EngineTorque", "N*m");
    private readonly TrackedFloat m_enginePower =
        new("Vehicle / EnginePower", "kW");
    private readonly TrackedFloat m_driveRPM =
        new("Engine / DriveRPM", "r/s");
    private readonly TrackedFloat m_engineNetTorque =
        new("Engine / NetTorque", "N*m");

    private readonly TrackedFloat m_wheelSlipL =
        new("DrivingWheel / Slip_L", "?");
    private readonly TrackedFloat m_wheelTorqueL =
        new("DrivingWheel / Torque_L", "N*m");
    private readonly TrackedFloat m_wheelSlipR =
        new("DrivingWheel / Slip_R", "?");
    private readonly TrackedFloat m_wheelTorqueR =
        new("DrivingWheel / Torque_R", "N*m");

    // Add your own vehicle-specific tracked members here.
    //
    // private readonly TrackedFloat _speed =
    //     new("Vehicle / Speed", "m/s");
    //
    // private readonly TrackedInt _gear =
    //     new("Vehicle / Gear");
    //
    // private readonly TrackedVector3 _localVelocity =
    //     new("Vehicle / Local Velocity", "m/s");

    private void Awake()
    {
        m_timeSamples.EnsureCapacity(m_sampleCapacity);

        RegisterVehicleValues();

        foreach (TrackedValue value in m_values)
            value.EnsureCapacity(m_sampleCapacity);

        SetOpen(m_isOpen);
    }

    private void Update()
    {
        if (Input.GetKeyDown(m_toggleKey))
            SetOpen(!m_isOpen);

        if (m_vehicle == null)
            return;

        float t = m_useUnscaledTime
            ? Time.realtimeSinceStartup
            : Time.time;

        m_timeSamples.Add(t);

        SampleVehicle(m_vehicle);
    }

    protected virtual void RegisterVehicleValues()
    {
        //Register(m_worldPosition);
        Register(m_vehicleSpeed);
        Register(m_engineRPM);
        Register(m_engineTorque);
        Register(m_enginePower);
        Register(m_driveRPM);
        Register(m_engineNetTorque);
        Register(m_wheelSlipL);
        Register(m_wheelTorqueL);
        Register(m_wheelSlipR);
        Register(m_wheelTorqueR);
    }

    protected virtual void SampleVehicle(BasicVehicle v)
    {
        //m_worldPosition.Sample(v.transform.position);

        m_currentGear = v.CurrentGear;

        m_vehicleSpeed.Sample(v.VehicleSpeedKmsPerHour);
        m_engineRPM.Sample(v.EngineRPM);
        m_engineTorque.Sample(v.EngineOutputTorque);
        m_enginePower.Sample(v.EngineOutputPower / 1000.0f);
        m_driveRPM.Sample(v.DriveOutputShaftRPM);
        m_engineNetTorque.Sample(v.EngineNetTorque);
        m_wheelSlipL.Sample(v.SlipRL);
        m_wheelTorqueL.Sample(v.m_rearLeft.motorTorque);
        m_wheelSlipR.Sample(v.SlipRR);
        m_wheelTorqueR.Sample(v.m_rearRight.motorTorque);
    }

    private void Register(TrackedValue value)
    {
        if (value == null)
            return;

        value.EnsureCapacity(m_sampleCapacity);
        m_values.Add(value);
    }

    private void ClearBuffers()
    {
        m_timeSamples.Clear();

        foreach (TrackedValue value in m_values)
            value.Clear();
    }

    private void SetOpen(bool newOpen)
    {
        if (m_isOpen == newOpen)
            return;

        m_isOpen = newOpen;

        if (m_isOpen)
        {
            m_previousLockState = Cursor.lockState;
            m_previousCursorVisible = Cursor.visible;

            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
        else
        {
            Cursor.lockState = m_previousLockState;
            Cursor.visible = m_previousCursorVisible;
        }
    }

    public void SetVehicle(BasicVehicle newVehicle, bool clearHistory = true)
    {
        m_vehicle = newVehicle;

        if (clearHistory)
            ClearBuffers();
    }

    private void OnGUI()
    {
        if (!m_isOpen)
            return;

        CalculateRightAnchoredPanelRect();

        GUILayout.BeginArea(m_panelRect, GUI.skin.box);

        GUILayout.BeginHorizontal();
        GUILayout.Label("BasicVehicleRuntimeDebugPanel");
        GUILayout.FlexibleSpace();

        if (GUILayout.Button("Clear", GUILayout.Width(70)))
            ClearBuffers();

        GUILayout.EndHorizontal();

        DrawVehicleSelector();

        GUILayout.Space(4);

        if (m_vehicle == null)
        {
            GUILayout.Label("No vehicle selected.");
            GUILayout.EndArea();
            return;
        }

        GUILayout.Label($"Vehicle: {m_vehicle.name}");
        GUILayout.Label($"Samples: {m_timeSamples.Count}/{m_sampleCapacity}");

        GUILayout.Space(6);

        m_scroll = GUILayout.BeginScrollView(m_scroll);

        DraGear(m_currentGear);
        foreach (TrackedValue value in m_values)
        {
            DrawValueRow(value);
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void DraGear(DrivingGear gear)
    {
        if (gear is null)
        {
            return;
        }

        GUILayout.BeginVertical(GUI.skin.box);

        GUILayout.BeginHorizontal();

        GUILayout.Label("Gear | Ratio", GUILayout.Width(240));

        GUILayout.FlexibleSpace();

        GUILayout.Label(
            $"{gear.m_gearName} | {gear.m_gearRatio.ToString()}",
            GUILayout.Width(190));

        GUILayout.EndHorizontal();

        GUILayout.EndVertical();
    }

    private void DrawValueRow(TrackedValue value)
    {
        GUILayout.BeginVertical(GUI.skin.box);

        GUILayout.BeginHorizontal();

        value.ShowGraph = GUILayout.Toggle(
            value.ShowGraph,
            "Graph",
            GUILayout.Width(70));

        GUILayout.Label(value.Name, GUILayout.Width(240));

        GUILayout.FlexibleSpace();

        GUILayout.Label(
            value.LatestText,
            GUILayout.Width(190));

        GUILayout.EndHorizontal();

        if (value.ShowGraph)
        {
            Rect rect = GUILayoutUtility.GetRect(
                m_panelRect.width - 40f,
                m_graphHeight);

            value.DrawGraph(rect, m_timeSamples);
        }

        GUILayout.EndVertical();
    }

    private void DrawVehicleSelector()
    {
        GUILayout.BeginVertical(GUI.skin.box);

        GUILayout.BeginHorizontal();

        string currentName = m_vehicle != null
            ? m_vehicle.name
            : "No vehicle selected";

        GUILayout.Label("Vehicle", GUILayout.Width(70));

        if (GUILayout.Button(currentName, GUILayout.Width(260)))
        {
            _vehicleDropdownOpen = !_vehicleDropdownOpen;
            if(_vehicleDropdownOpen)
            {
                RefreshVehicleList();
            }
        }

        if (m_vehicle != null && GUILayout.Button("Clear", GUILayout.Width(70)))
        {
            SetVehicle(null, clearHistoryWhenVehicleChanges);
        }

        GUILayout.EndHorizontal();

        if (_vehicleDropdownOpen)
        {
            if (_availableVehicles.Count == 0)
            {
                GUILayout.Label("No BasicVehicle instances found.");
            }
            else
            {
                for (int i = 0; i < _availableVehicles.Count; i++)
                {
                    BasicVehicle candidate = _availableVehicles[i];

                    if (candidate == null)
                        continue;

                    bool isSelected = candidate == m_vehicle;
                    string label = isSelected
                        ? $"✓ {_vehicleNames[i]}"
                        : $"  {_vehicleNames[i]}";

                    if (GUILayout.Button(label))
                    {
                        SetVehicle(candidate, clearHistoryWhenVehicleChanges);
                        _vehicleDropdownOpen = false;
                    }
                }
            }
        }

        GUILayout.EndVertical();
    }

    // ---------------------------------------------------------------------
    // Vehicle choose list
    // ---------------------------------------------------------------------
    private void RefreshVehicleList()
    {
        _availableVehicles.Clear();

        BasicVehicle[] found = UnityEngine.Object.FindObjectsByType<BasicVehicle>(
        FindObjectsInactive.Exclude,
        FindObjectsSortMode.None);

        Array.Sort(found, (a, b) =>
            string.Compare(a.name, b.name, StringComparison.OrdinalIgnoreCase));

        _availableVehicles.AddRange(found);

        _vehicleNames = new string[_availableVehicles.Count];

        for (int i = 0; i < _availableVehicles.Count; i++)
        {
            BasicVehicle v = _availableVehicles[i];

            string sceneName = v.gameObject.scene.IsValid()
                ? v.gameObject.scene.name
                : "No Scene";

            _vehicleNames[i] = $"{v.name}  [{sceneName}]";
        }

        if (m_vehicle != null && !_availableVehicles.Contains(m_vehicle))
        {
            SetVehicle(null, clearHistoryWhenVehicleChanges);
        }
    }

    // ---------------------------------------------------------------------
    // Tracked value types
    // ---------------------------------------------------------------------

    public abstract class TrackedValue
    {
        public string Name { get; }
        public string Unit { get; }
        public bool ShowGraph { get; set; }

        public abstract string LatestText { get; }

        protected TrackedValue(string name, string unit = "")
        {
            Name = name;
            Unit = unit;
            ShowGraph = false; // default display is latest value only
        }

        internal abstract void EnsureCapacity(int capacity);
        internal abstract void Clear();
        internal abstract void DrawGraph(Rect rect, RingBuffer<float> timeSamples);
    }

    public sealed class TrackedFloat : TrackedValue
    {
        private readonly RingBuffer<float> _samples = new(1);

        public TrackedFloat(string name, string unit = "") : base(name, unit)
        {
        }

        public void Sample(float value)
        {
            _samples.Add(value);
        }

        public override string LatestText
        {
            get
            {
                if (_samples.Count == 0)
                    return "—";

                return FormatScalar(_samples.Latest, Unit);
            }
        }

        internal override void EnsureCapacity(int capacity)
        {
            _samples.EnsureCapacity(capacity);
        }

        internal override void Clear()
        {
            _samples.Clear();
        }

        internal override void DrawGraph(Rect rect, RingBuffer<float> timeSamples)
        {
            int count = Mathf.Min(_samples.Count, timeSamples.Count);

            if (count < 2)
            {
                GUI.Box(rect, "Not enough samples");
                return;
            }

            FindMinMax(
                count,
                i => _samples.GetRecent(i, count),
                out float min,
                out float max);

            DrawGraphFrame(rect, timeSamples, count, min, max, Unit);
            DrawSeries(
                rect,
                timeSamples,
                count,
                i => _samples.GetRecent(i, count),
                min,
                max,
                Color.cyan);
        }
    }

    public sealed class TrackedInt : TrackedValue
    {
        private readonly RingBuffer<int> _samples = new(1);

        public TrackedInt(string name, string unit = "") : base(name, unit)
        {
        }

        public void Sample(int value)
        {
            _samples.Add(value);
        }

        public override string LatestText
        {
            get
            {
                if (_samples.Count == 0)
                    return "—";

                return string.IsNullOrWhiteSpace(Unit)
                    ? _samples.Latest.ToString()
                    : $"{_samples.Latest} {Unit}";
            }
        }

        internal override void EnsureCapacity(int capacity)
        {
            _samples.EnsureCapacity(capacity);
        }

        internal override void Clear()
        {
            _samples.Clear();
        }

        internal override void DrawGraph(Rect rect, RingBuffer<float> timeSamples)
        {
            int count = Mathf.Min(_samples.Count, timeSamples.Count);

            if (count < 2)
            {
                GUI.Box(rect, "Not enough samples");
                return;
            }

            FindMinMax(
                count,
                i => _samples.GetRecent(i, count),
                out float min,
                out float max);

            DrawGraphFrame(rect, timeSamples, count, min, max, Unit);
            DrawSeries(
                rect,
                timeSamples,
                count,
                i => _samples.GetRecent(i, count),
                min,
                max,
                Color.yellow);
        }
    }

    public sealed class TrackedVector2 : TrackedValue
    {
        private readonly RingBuffer<Vector2> _samples = new(1);

        public TrackedVector2(string name, string unit = "") : base(name, unit)
        {
        }

        public void Sample(Vector2 value)
        {
            _samples.Add(value);
        }

        public override string LatestText
        {
            get
            {
                if (_samples.Count == 0)
                    return "—";

                Vector2 v = _samples.Latest;

                return string.IsNullOrWhiteSpace(Unit)
                    ? $"({v.x:0.###}, {v.y:0.###})"
                    : $"({v.x:0.###}, {v.y:0.###}) {Unit}";
            }
        }

        internal override void EnsureCapacity(int capacity)
        {
            _samples.EnsureCapacity(capacity);
        }

        internal override void Clear()
        {
            _samples.Clear();
        }

        internal override void DrawGraph(Rect rect, RingBuffer<float> timeSamples)
        {
            int count = Mathf.Min(_samples.Count, timeSamples.Count);

            if (count < 2)
            {
                GUI.Box(rect, "Not enough samples");
                return;
            }

            FindVector2MinMax(count, _samples, out float min, out float max);

            DrawGraphFrame(rect, timeSamples, count, min, max, Unit);

            DrawSeries(rect, timeSamples, count, i => _samples.GetRecent(i, count).x, min, max, Color.red);
            DrawSeries(rect, timeSamples, count, i => _samples.GetRecent(i, count).y, min, max, Color.green);

            DrawLegend(rect, "x", Color.red, 0);
            DrawLegend(rect, "y", Color.green, 1);
        }
    }

    public sealed class TrackedVector3 : TrackedValue
    {
        private readonly RingBuffer<Vector3> _samples = new(1);

        public TrackedVector3(string name, string unit = "") : base(name, unit)
        {
        }

        public void Sample(Vector3 value)
        {
            _samples.Add(value);
        }

        public override string LatestText
        {
            get
            {
                if (_samples.Count == 0)
                    return "—";

                Vector3 v = _samples.Latest;

                return string.IsNullOrWhiteSpace(Unit)
                    ? $"({v.x:0.###}, {v.y:0.###}, {v.z:0.###})"
                    : $"({v.x:0.###}, {v.y:0.###}, {v.z:0.###}) {Unit}";
            }
        }

        internal override void EnsureCapacity(int capacity)
        {
            _samples.EnsureCapacity(capacity);
        }

        internal override void Clear()
        {
            _samples.Clear();
        }

        internal override void DrawGraph(Rect rect, RingBuffer<float> timeSamples)
        {
            int count = Mathf.Min(_samples.Count, timeSamples.Count);

            if (count < 2)
            {
                GUI.Box(rect, "Not enough samples");
                return;
            }

            FindVector3MinMax(count, _samples, out float min, out float max);

            DrawGraphFrame(rect, timeSamples, count, min, max, Unit);

            DrawSeries(rect, timeSamples, count, i => _samples.GetRecent(i, count).x, min, max, Color.red);
            DrawSeries(rect, timeSamples, count, i => _samples.GetRecent(i, count).y, min, max, Color.green);
            DrawSeries(rect, timeSamples, count, i => _samples.GetRecent(i, count).z, min, max, Color.blue);

            DrawLegend(rect, "x", Color.red, 0);
            DrawLegend(rect, "y", Color.green, 1);
            DrawLegend(rect, "z", Color.blue, 2);
        }
    }

    // ---------------------------------------------------------------------
    // Ring buffer
    // ---------------------------------------------------------------------

    public sealed class RingBuffer<T>
    {
        private T[] _items;
        private int _nextIndex;

        public int Count { get; private set; }
        public int Capacity => _items.Length;

        public T Latest => Count == 0
            ? default
            : GetChronological(Count - 1);

        public RingBuffer(int capacity)
        {
            _items = new T[Mathf.Max(1, capacity)];
        }

        public void EnsureCapacity(int capacity)
        {
            capacity = Mathf.Max(1, capacity);

            if (_items != null && _items.Length == capacity)
                return;

            _items = new T[capacity];
            _nextIndex = 0;
            Count = 0;
        }

        public void Add(T value)
        {
            _items[_nextIndex] = value;

            _nextIndex = (_nextIndex + 1) % _items.Length;

            if (Count < _items.Length)
                Count++;
        }

        public void Clear()
        {
            Array.Clear(_items, 0, _items.Length);
            _nextIndex = 0;
            Count = 0;
        }

        public T GetChronological(int index)
        {
            if (index < 0 || index >= Count)
                throw new ArgumentOutOfRangeException(nameof(index));

            int oldestIndex = Count == _items.Length
                ? _nextIndex
                : 0;

            int actualIndex = (oldestIndex + index) % _items.Length;
            return _items[actualIndex];
        }

        public T GetRecent(int chronologicalIndex, int recentCount)
        {
            if (recentCount < 0 || recentCount > Count)
                throw new ArgumentOutOfRangeException(nameof(recentCount));

            if (chronologicalIndex < 0 || chronologicalIndex >= recentCount)
                throw new ArgumentOutOfRangeException(nameof(chronologicalIndex));

            int start = Count - recentCount;
            return GetChronological(start + chronologicalIndex);
        }
    }

    // ---------------------------------------------------------------------
    // Drawing helpers
    // ---------------------------------------------------------------------
    private void CalculateRightAnchoredPanelRect()
    {
        float height = Mathf.Max(100f, Screen.height - m_topMargin - m_bottomMargin);

        m_panelRect.x = Screen.width - m_panelWidth - m_rightMargin;
        m_panelRect.y = m_topMargin;
        m_panelRect.width = m_panelWidth;
        m_panelRect.height = height;
    }

    private static string FormatScalar(float value, string unit)
    {
        return string.IsNullOrWhiteSpace(unit)
            ? value.ToString("0.###")
            : $"{value:0.###} {unit}";
    }

    private static void DrawGraphFrame(
        Rect rect,
        RingBuffer<float> timeSamples,
        int count,
        float min,
        float max,
        string unit)
    {
        GUI.Box(rect, GUIContent.none);

        string unitSuffix = string.IsNullOrWhiteSpace(unit) ? "" : $" {unit}";

        GUI.Label(
            new Rect(rect.x + 4, rect.y + 2, 160, 18),
            $"{max:0.###}{unitSuffix}");

        GUI.Label(
            new Rect(rect.x + 4, rect.yMax - 20, 160, 18),
            $"{min:0.###}{unitSuffix}");

        float startTime = timeSamples.GetRecent(0, count);
        float endTime = timeSamples.GetRecent(count - 1, count);

        GUI.Label(
            new Rect(rect.x + 4, rect.yMax - 38, 120, 18),
            $"{startTime:0.0}s");

        GUI.Label(
            new Rect(rect.xMax - 80, rect.yMax - 38, 76, 18),
            $"{endTime:0.0}s");
    }

    private static void DrawSeries(
        Rect rect,
        RingBuffer<float> timeSamples,
        int count,
        Func<int, float> valueAt,
        float min,
        float max,
        Color color)
    {
        if (Mathf.Approximately(min, max))
        {
            min -= 1f;
            max += 1f;
        }

        float startTime = timeSamples.GetRecent(0, count);
        float endTime = timeSamples.GetRecent(count - 1, count);
        float timeRange = Mathf.Max(0.0001f, endTime - startTime);
        float valueRange = Mathf.Max(0.0001f, max - min);

        bool hasPrevious = false;
        Vector2 previous = default;

        for (int i = 0; i < count; i++)
        {
            float value = valueAt(i);

            if (!IsFinite(value))
            {
                hasPrevious = false;
                continue;
            }

            float t = timeSamples.GetRecent(i, count);

            float x01 = (t - startTime) / timeRange;
            float y01 = (value - min) / valueRange;

            Vector2 current = new(
                Mathf.Lerp(rect.x, rect.xMax, x01),
                Mathf.Lerp(rect.yMax, rect.y, y01));

            if (hasPrevious)
                DrawLine(previous, current, color, 2f);

            previous = current;
            hasPrevious = true;
        }
    }

    private static void DrawLine(Vector2 a, Vector2 b, Color color, float width)
    {
        Matrix4x4 oldMatrix = GUI.matrix;
        Color oldColor = GUI.color;

        Vector2 delta = b - a;
        float angle = Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg;
        float length = delta.magnitude;

        GUI.color = color;
        GUIUtility.RotateAroundPivot(angle, a);
        GUI.DrawTexture(
            new Rect(a.x, a.y - width * 0.5f, length, width),
            Texture2D.whiteTexture);

        GUI.matrix = oldMatrix;
        GUI.color = oldColor;
    }

    private static void DrawLegend(Rect rect, string label, Color color, int index)
    {
        Color oldColor = GUI.color;
        GUI.color = color;

        GUI.Label(
            new Rect(rect.xMax - 34, rect.y + 4 + index * 16, 30, 16),
            label);

        GUI.color = oldColor;
    }

    private static void FindMinMax(
        int count,
        Func<int, float> valueAt,
        out float min,
        out float max)
    {
        min = float.PositiveInfinity;
        max = float.NegativeInfinity;

        for (int i = 0; i < count; i++)
        {
            float value = valueAt(i);

            if (!IsFinite(value))
                continue;

            min = Mathf.Min(min, value);
            max = Mathf.Max(max, value);
        }

        if (!IsFinite(min) || !IsFinite(max))
        {
            min = 0f;
            max = 1f;
        }

        if (Mathf.Approximately(min, max))
        {
            min -= 1f;
            max += 1f;
        }
    }

    private static void FindVector2MinMax(
        int count,
        RingBuffer<Vector2> samples,
        out float min,
        out float max)
    {
        min = float.PositiveInfinity;
        max = float.NegativeInfinity;

        for (int i = 0; i < count; i++)
        {
            Vector2 v = samples.GetRecent(i, count);

            Include(v.x, ref min, ref max);
            Include(v.y, ref min, ref max);
        }

        ValidateMinMax(ref min, ref max);
    }

    private static void FindVector3MinMax(
        int count,
        RingBuffer<Vector3> samples,
        out float min,
        out float max)
    {
        min = float.PositiveInfinity;
        max = float.NegativeInfinity;

        for (int i = 0; i < count; i++)
        {
            Vector3 v = samples.GetRecent(i, count);

            Include(v.x, ref min, ref max);
            Include(v.y, ref min, ref max);
            Include(v.z, ref min, ref max);
        }

        ValidateMinMax(ref min, ref max);
    }

    private static void Include(float value, ref float min, ref float max)
    {
        if (!IsFinite(value))
            return;

        min = Mathf.Min(min, value);
        max = Mathf.Max(max, value);
    }

    private static void ValidateMinMax(ref float min, ref float max)
    {
        if (!IsFinite(min) || !IsFinite(max))
        {
            min = 0f;
            max = 1f;
        }

        if (Mathf.Approximately(min, max))
        {
            min -= 1f;
            max += 1f;
        }
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}