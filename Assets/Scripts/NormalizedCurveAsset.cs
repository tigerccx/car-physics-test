using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(
    fileName = "New Curve",
    menuName = "Game/Normalized Curve Asset")]
public class NormalizedCurveAsset : ScriptableObject
{
    public float m_xScale = 1.0f;
    public float m_yScale = 1.0f;
    public AnimationCurve m_curve = AnimationCurve.Linear(0f, 0f, 1f, 1f);

    public float Evaluate(float t)
    {
        return m_curve.Evaluate(t / m_xScale) * m_yScale;
    }
}