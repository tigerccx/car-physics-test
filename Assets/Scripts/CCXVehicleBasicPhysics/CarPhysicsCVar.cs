using UnityEngine;

public static class CCXVehicleCVars
{
    public static readonly CVar<bool> DrawDebug =
        CVarRegistry.Register(
            "vehicle.drawDebug",
            false,
            "Draw vehicle debug. ");

    public static readonly CVar<bool> DrawSuspensionDebug =
        CVarRegistry.Register(
            "vehicle.drawSuspensionDebug",
            false,
            "Draw suspension debugs.");
    public static readonly CVar<float> DrawSuspensionDebugScale =
        CVarRegistry.Register(
            "vehicle.drawSuspensionDebugScale",
            0.1f,
            "Draw suspension debugs in scale.");

    public static readonly CVar<bool> DrawWheelDebug =
        CVarRegistry.Register(
            "vehicle.drawWheelDebug",
            false,
            "Draw wheel debugs.");
    public static readonly CVar<float> DrawWheelDebugScale =
        CVarRegistry.Register(
            "vehicle.drawWheelDebugScale",
            1.0f,
            "Draw wheel debugs in scale.");

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void RegisterCVars()
    {
        //// Touching the static fields forces registration.
        //_ = DrawDebug; 
        //_ = DrawSuspensionDebug; 
        //_ = DrawSuspensionDebugScale; 
        //_ = DrawWheelDebug; 
        //_ = DrawWheelDebugScale; 
    }
}