using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(
    fileName = "New Curve",
    menuName = "Game/Curve Asset")]
public class CurveAsset : ScriptableObject
{
    public AnimationCurve curve = AnimationCurve.Linear(0f, 0f, 1f, 1f);

    public float Evaluate(float t)
    {
        return curve.Evaluate(t);
    }
}