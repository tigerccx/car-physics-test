using System;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

public static class CVarConfig
{
    public static string ConfigPath => Path.Combine(Application.dataPath, "Config", "cvar.config");
    public static event Action Loaded;

    [MenuItem("Tools/CVar/Save Config")]
    public static void Save()
    {
        CVarDiscovery.Discover();
        try
        {
            var lines = CVarRegistry.All
                .OrderBy(v => v.Name, StringComparer.OrdinalIgnoreCase)
                .Select(v => $"{v.Name}={Uri.EscapeDataString(v.GetString())}");
            string content = "# Auto-generated CVar config; values are URI-escaped.\n"
                + string.Join("\n", lines) + "\n";
            Directory.CreateDirectory(Path.GetDirectoryName(ConfigPath));
            File.WriteAllText(ConfigPath, content, new UTF8Encoding(false));
            Debug.Log($"CVar config saved: {ConfigPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to save CVar config: {e}");
        }
    }

    [MenuItem("Tools/CVar/Load Config")]
    public static void Load()
    {
        CVarDiscovery.Discover();
        if (!File.Exists(ConfigPath))
            return;

        try
        {
            int lineNumber = 0;
            foreach (string rawLine in File.ReadAllLines(ConfigPath, Encoding.UTF8))
            {
                lineNumber++;
                string line = rawLine.TrimStart();
                if (line.Length == 0 || line.StartsWith("#", StringComparison.Ordinal))
                    continue;

                int separator = line.IndexOf('=');
                if (separator <= 0)
                {
                    Debug.LogWarning($"Invalid CVar config line {lineNumber}: expected name=value.");
                    continue;
                }

                string name = line.Substring(0, separator).Trim();
                string value = Uri.UnescapeDataString(line.Substring(separator + 1));
                if (!CVarRegistry.TryGet(name, out _))
                    Debug.LogWarning($"Unknown CVar in config at line {lineNumber}: {name}");
                else if (!CVarRegistry.Set(name, value))
                    Debug.LogWarning($"Invalid CVar value at line {lineNumber}: {name}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to load CVar config: {e}");
        }

        Loaded?.Invoke();
    }
}
