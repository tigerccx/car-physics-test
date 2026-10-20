
using System;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public static class CVarDiscovery
{
    static CVarDiscovery()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        EditorApplication.delayCall += Initialize;
    }

    private static void Initialize()
    {
        if (!EditorApplication.isPlayingOrWillChangePlaymode)
            CVarConfig.Load();
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
    private static void InitializePlayMode() => CVarConfig.Load();

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.ExitingEditMode)
            CVarConfig.Save();
        else if (state == PlayModeStateChange.EnteredEditMode)
            CVarConfig.Load();
    }

    public static void Discover()
    {
        foreach (var field in
            TypeCache.GetFieldsWithAttribute<CVarAttribute>())
        {
            if (!field.IsStatic)
                continue;

            if (!typeof(ICVar).IsAssignableFrom(field.FieldType))
                continue;

            try
            {
                // 触发静态字段初始化
                // 字段初始化时调用 CVarRegistry.Register()
                var cvar = field.GetValue(null) as ICVar;

                if (cvar == null)
                    Debug.LogWarning(
                        $"CVar field is null: {field.Name}");
            }
            catch (Exception e)
            {
                Debug.LogException(e);
            }
        }
    }
}
