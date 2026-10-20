using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class CVarDebugWindow : EditorWindow
{
    private string input = "";
    private Vector2 _scroll;
    private readonly Dictionary<string, string> _editBuffers = new();

    private void OnEnable()
    {
        CVarDiscovery.Discover();
        CVarConfig.Loaded += RefreshValues;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private void OnDisable()
    {
        CVarConfig.Loaded -= RefreshValues;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
    }

    private void OnPlayModeStateChanged(PlayModeStateChange state) => RefreshValues();

    private void RefreshValues()
    {
        _editBuffers.Clear();
        Repaint();
    }

    [MenuItem("Tools/CVar/Open Window")]
    public static void Open()
    {
        GetWindow<CVarDebugWindow>("CVar");
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 600, 300), GUI.skin.box);

        GUILayout.Label("CVar Console");

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Save Config"))
            CVarConfig.Save();
        if (GUILayout.Button("Load Config"))
            CVarConfig.Load();
        GUILayout.EndHorizontal();

        input = GUILayout.TextField(input);

        if (GUILayout.Button("Execute"))
        {
            Execute(input);
            RefreshValues();
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
        string[] parts = command.Split(new[] { ' ' }, 2, StringSplitOptions.None);

        if (parts.Length != 2)
        {
            Debug.LogWarning("Expected format: <cvar_name> <value>");
            return;
        }

        CVarRegistry.Set(parts[0], parts[1]);
    }
}
