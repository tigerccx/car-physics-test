using System;

[AttributeUsage(AttributeTargets.Field, AllowMultiple = false)]
public sealed class CVarAttribute : Attribute
{
    public string Category { get; }
    public string Description { get; }
    public bool ReadOnly { get; }

    public CVarAttribute(
        string category = "General",
        string description = "",
        bool readOnly = false)
    {
        Category = category;
        Description = description;
        ReadOnly = readOnly;
    }
}