using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

public interface ICVar
{
    string Name { get; }
    string Description { get; }
    Type ValueType { get; }
    string GetString();
    bool SetString(string value);
}

public sealed class CVar<T> : ICVar
{
    public string Name { get; }
    public string Description { get; }
    public T Value { get; private set; }

    public Type ValueType => typeof(T);

    public CVar(string name, T defaultValue, string description = "")
    {
        Name = name;
        Value = defaultValue;
        Description = description;
    }

    public void Set(T value)
    {
        Value = value;
    }

    public string GetString()
    {
        if (Value is float single)
            return single.ToString("R", CultureInfo.InvariantCulture);
        if (Value is double number)
            return number.ToString("R", CultureInfo.InvariantCulture);
        return Value is IFormattable formattable
            ? formattable.ToString(null, CultureInfo.InvariantCulture)
            : Value?.ToString() ?? string.Empty;
    }

    public bool SetString(string text)
    {
        if (!TryParse(text, out T parsedValue))
        {
            Debug.LogWarning($"Failed to parse CVar '{Name}' value '{text}' as {typeof(T).Name}");
            return false;
        }

        Value = parsedValue;
        return true;
    }

    private static bool TryParse(string text, out T value)
    {
        object parsedValue;

        Type type = typeof(T);

        if (type == typeof(bool))
        {
            if (TryParseBool(text, out bool boolValue))
            {
                parsedValue = boolValue;
                value = (T)parsedValue;
                return true;
            }
        }
        else if (type == typeof(int))
        {
            if (int.TryParse(
                    text,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out int intValue))
            {
                parsedValue = intValue;
                value = (T)parsedValue;
                return true;
            }
        }
        else if (type == typeof(float))
        {
            if (float.TryParse(
                    text,
                    NumberStyles.Float,
                    CultureInfo.InvariantCulture,
                    out float floatValue))
            {
                parsedValue = floatValue;
                value = (T)parsedValue;
                return true;
            }
        }
        else if (type == typeof(double))
        {
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double doubleValue))
            {
                value = (T)(object)doubleValue;
                return true;
            }
        }
        else if (type == typeof(string))
        {
            parsedValue = text;
            value = (T)parsedValue;
            return true;
        }

        value = default;
        return false;
    }

    private static bool TryParseBool(string text, out bool value)
    {
        switch (text.Trim().ToLowerInvariant())
        {
            case "1":
            case "true":
            case "on":
            case "yes":
            case "y":
                value = true;
                return true;

            case "0":
            case "false":
            case "off":
            case "no":
            case "n":
                value = false;
                return true;

            default:
                value = false;
                return false;
        }
    }
}

public static class CVarRegistry
{
    private static readonly Dictionary<string, ICVar> Vars =
        new(StringComparer.OrdinalIgnoreCase);

    public static CVar<T> Register<T>(
        string name,
        T defaultValue,
        string description = "")
    {
        if (Vars.TryGetValue(name, out ICVar existing))
            return (CVar<T>)existing;

        var cvar = new CVar<T>(name, defaultValue, description);
        Vars.Add(name, cvar);
        return cvar;
    }

    public static bool Set(string name, string value)
    {
        if (!Vars.TryGetValue(name, out ICVar cvar))
        {
            Debug.LogWarning($"Unknown CVar: {name}");
            return false;
        }

        return cvar.SetString(value);
    }

    public static bool TryGet(string name, out ICVar cvar)
    {
        return Vars.TryGetValue(name, out cvar);
    }

    public static IEnumerable<ICVar> All => Vars.Values;
}
