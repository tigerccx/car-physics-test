
using UnityEngine;
using UnityEditor;

public class RoadTurnGeneratorWindow : EditorWindow
{
    private GameObject roadTemplate;

    private float radius = 20f;
    private float roadWidth = 3.5f;
    private int segmentCount = 8;
    private int segmentIndex = 2;

    private bool turnRight = true;

    private static readonly int[] SegmentOptions =
    {
        2, 4, 8, 16
    };

    [MenuItem("Tools/Road Turn Generator")]
    public static void Open()
    {
        GetWindow<RoadTurnGeneratorWindow>(
            "Road Turn Generator");
    }

    private void OnGUI()
    {
        EditorGUILayout.LabelField(
            "Road Template",
            EditorStyles.boldLabel);

        roadTemplate = (GameObject)EditorGUILayout.ObjectField(
            "Template",
            roadTemplate,
            typeof(GameObject),
            true);

        EditorGUILayout.Space(10);

        EditorGUILayout.LabelField(
            "Turn Settings",
            EditorStyles.boldLabel);

        radius = EditorGUILayout.FloatField(
            "Radius (m)", radius);

        roadWidth = EditorGUILayout.FloatField(
            "Road Width (m)", roadWidth);

        segmentIndex = EditorGUILayout.Popup(
            "Segments",
            segmentIndex,
            new[] { "2", "4", "8", "16" });

        segmentCount = SegmentOptions[segmentIndex];

        turnRight = EditorGUILayout.Toggle(
            "Turn Right", turnRight);

        EditorGUILayout.Space(10);

        float angleStep = 90f / segmentCount;
        float outerRadius = radius + roadWidth * 0.5f;

        float length = 2f * outerRadius *
            Mathf.Sin(angleStep * Mathf.Deg2Rad * 0.5f);

        EditorGUILayout.LabelField(
            "Segment Length",
            $"{length:F3} m");

        EditorGUILayout.LabelField(
            "Rotation Increment",
            $"{angleStep:F3}бу");

        EditorGUILayout.Space(10);

        using (new EditorGUI.DisabledScope(
            roadTemplate == null || radius <= 0f || roadWidth <= 0f))
        {
            if (GUILayout.Button(
                "Generate Road",
                GUILayout.Height(32)))
            {
                GenerateRoad();
            }
        }
    }

    private void GenerateRoad()
    {
        int n = segmentCount;
        float angleStepRad = Mathf.PI * 0.5f / n;

        float outerRadius = radius + roadWidth * 0.5f;

        float segmentLength =
            2f * outerRadius * Mathf.Sin(angleStepRad * 0.5f);

        // Create a new root GameObject
        GameObject root = new GameObject(
            $"RoadTurn_R{radius:F0}_S{n}");

        Undo.RegisterCreatedObjectUndo(
            root, "Generate Road Turn");

        // Position the generated road at the selected
        // object's position, if one exists.
        Transform anchor = Selection.activeTransform;
        if (anchor != null)
        {
            root.transform.SetPositionAndRotation(
                anchor.position,
                anchor.rotation);
        }

        for (int i = 0; i < n; i++)
        {
            float a0 = i * angleStepRad;
            float a1 = (i + 1) * angleStepRad;

            Vector3 p0 = GetArcPoint(a0);
            Vector3 p1 = GetArcPoint(a1);

            Vector3 midpoint = (p0 + p1) * 0.5f;
            Vector3 direction = (p1 - p0).normalized;

            GameObject segment =
                PrefabUtility.IsPartOfPrefabAsset(roadTemplate)
                    ? (GameObject)PrefabUtility.InstantiatePrefab(
                        roadTemplate)
                    : Instantiate(roadTemplate);

            segment.name = $"Road_{i:D2}";

            Undo.RegisterCreatedObjectUndo(
                segment, "Generate Road Segment");

            segment.transform.SetParent(
                root.transform, false);

            segment.transform.localPosition = midpoint;
            segment.transform.localRotation =
                Quaternion.LookRotation(direction, Vector3.up);

            // The template is assumed to have
            // unit X/Z dimensions.
            Vector3 scale = roadTemplate.transform.localScale;
            scale.x = roadWidth;
            scale.z = segmentLength;

            segment.transform.localScale = scale;

            segment.SetActive(true);
        }

        Selection.activeGameObject = root;

        Debug.Log(
            $"Generated road: R={radius}, " +
            $"segments={n}, length={segmentLength:F3}m");
    }

    private Vector3 GetArcPoint(float angle)
    {
        float x = radius * (1f - Mathf.Cos(angle));
        float z = radius * Mathf.Sin(angle);

        if (!turnRight)
            x = -x;

        return new Vector3(x, 0f, z);
    }
}
