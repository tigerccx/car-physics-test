using UnityEngine;

public static class VehicleCVars
{
    //public static readonly CVar<bool> DrawDebug =
    //    CVarRegistry.Register(
    //        "vehicle.drawDebug",
    //        false,
    //        "Draw vehicle debug. ");

    [CVar("Vehicle.Control", "Draw vehicle control debug. ")]
    public static readonly CVar<bool> DrawVehicleControlDebug =
        CVarRegistry.Register(
            "vehicle.control.drawVehicleDebug",
            false,
            "Draw vehicle control debug.");

    [CVar("Vehicle.Control", "Draw vehicle control wheel debug. ")]
    public static readonly CVar<bool> DrawWheelControlDebug =
        CVarRegistry.Register(
            "vehicle.control.drawWheelDebug",
            false,
            "Draw vehicle control wheel debugs.");

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void RegisterCVars()
    {
        //_ = DrawDebug; 
        _ = DrawVehicleControlDebug;
        _ = DrawWheelControlDebug; 
    }
}