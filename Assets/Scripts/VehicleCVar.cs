using UnityEngine;

public static class VehicleCVars
{
    //public static readonly CVar<bool> DrawDebug =
    //    CVarRegistry.Register(
    //        "vehicle.drawDebug",
    //        false,
    //        "Draw vehicle debug. ");

    public static readonly CVar<bool> DrawVehicleDebug =
        CVarRegistry.Register(
            "vehicle.drawVehicleDebug",
            false,
            "Draw vehicle debugs.");

    public static readonly CVar<bool> DrawWheelDebug =
        CVarRegistry.Register(
            "vehicle.drawWheelDebug",
            false,
            "Draw wheel debugs.");

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void RegisterCVars()
    {
        //_ = DrawDebug; 
        _ = DrawVehicleDebug;
        _ = DrawWheelDebug; 
    }
}