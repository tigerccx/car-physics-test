using System.Collections.Generic;
using UnityEngine;

public class CVarConsole : MonoBehaviour
{
    public static CVarConsole Instance { get; set; }

    public bool IsOpen { get { return open; } }

    private string input = "";
    private bool open;
    private Vector2 _scroll;
    private CursorLockMode _previousLockState;
    private bool _previousCursorVisible;
    private readonly Dictionary<string, string> _editBuffers = new();

    // Bootstrap to auto create on scene load
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void Bootstrap()
    {
        var go = new GameObject("[CVar Console]");
        go.AddComponent<CVarConsole>();
        DontDestroyOnLoad(go);

        Instance = go.GetComponent<CVarConsole>();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.BackQuote))
        {
            SetOpen(!open);
        }
    }

    private void SetOpen(bool newOpen)
    {
        if (open == newOpen)
            return;

        open = newOpen;

        if (open)
        {
            _previousLockState = Cursor.lockState;
            _previousCursorVisible = Cursor.visible;

            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
        else
        {
            Cursor.lockState = _previousLockState;
            Cursor.visible = _previousCursorVisible;
        }
    }

    private void OnGUI()
    {
        if (!open)
            return;

        GUILayout.BeginArea(new Rect(10, 10, 600, 300), GUI.skin.box);

        GUILayout.Label("CVar Console");

        input = GUILayout.TextField(input);

        if (GUILayout.Button("Execute"))
        {
            Execute(input);
            input = "";
        }

        _scroll = GUILayout.BeginScrollView(_scroll);

        foreach (ICVar cvar in CVarRegistry.All)
        {
            DrawCVar(cvar);
        }

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void DrawCVar(ICVar cvar)
    {
        GUILayout.BeginHorizontal();

        GUILayout.Label(cvar.Name, GUILayout.Width(240));

        if (cvar.ValueType == typeof(bool))
        {
            bool currentValue = cvar.GetString().ToLowerInvariant() is "true" or "1" or "on" or "yes";

            bool newValue = GUILayout.Toggle(currentValue, "", GUILayout.Width(30));

            if (newValue != currentValue)
            {
                cvar.SetString(newValue ? "true" : "false");
            }
        }
        else
        {
            if (!_editBuffers.TryGetValue(cvar.Name, out string text))
            {
                text = cvar.GetString();
                _editBuffers[cvar.Name] = text;
            }

            string newText = GUILayout.TextField(text, GUILayout.Width(160));
            _editBuffers[cvar.Name] = newText;

            if (GUILayout.Button("Apply", GUILayout.Width(60)))
            {
                if (cvar.SetString(newText))
                {
                    _editBuffers[cvar.Name] = cvar.GetString();
                }
            }

            if (GUILayout.Button("Reset Text", GUILayout.Width(80)))
            {
                _editBuffers[cvar.Name] = cvar.GetString();
            }
        }

        //GUILayout.Label(cvar.Description);

        GUILayout.EndHorizontal();
    }

    private static void Execute(string command)
    {
        string[] parts = command.Split(' ', 2);

        if (parts.Length != 2)
        {
            Debug.LogWarning("Expected format: <cvar_name> <value>");
            return;
        }

        CVarRegistry.Set(parts[0], parts[1]);
    }
}