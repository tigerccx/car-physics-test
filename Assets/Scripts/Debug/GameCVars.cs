using UnityEngine;

public static class GameCVars
{
    public static readonly CVar<float> TimeScale =
        CVarRegistry.Register(
            "game.timeScale",
            1.0f,
            "Controls Unity Time.timeScale. 1 = normal, 0 = paused, 0.5 = slow motion.");

    public static void Register()
    {
        _ = TimeScale;
    }
}