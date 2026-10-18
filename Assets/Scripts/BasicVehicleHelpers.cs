using UnityEngine;

public static class BasicVehicleHelper
{
    public static float SampleFrictionCurve(WheelFrictionCurve curve, float slip)
    {
        slip = Mathf.Abs(slip);

        float extremumSlip = Mathf.Max(curve.extremumSlip, 0.0001f);
        float asymptoteSlip = Mathf.Max(curve.asymptoteSlip, extremumSlip + 0.0001f);

        float value;

        if (slip <= extremumSlip)
        {
            float t = slip / extremumSlip;

            // Spline-like rise from 0 to extremumValue.
            t = SmoothStep01(t);

            value = Mathf.Lerp(0f, curve.extremumValue, t);
        }
        else if (slip <= asymptoteSlip)
        {
            float t = (slip - extremumSlip) / (asymptoteSlip - extremumSlip);

            // Spline-like fall from extremumValue to asymptoteValue.
            t = SmoothStep01(t);

            value = Mathf.Lerp(curve.extremumValue, curve.asymptoteValue, t);
        }
        else
        {
            value = curve.asymptoteValue;
        }

        return value * curve.stiffness;
    }

    private static float SmoothStep01(float t)
    {
        t = Mathf.Clamp01(t);
        return t * t * (3f - 2f * t);
    }
}