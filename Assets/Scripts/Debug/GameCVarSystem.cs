using UnityEngine;

public sealed class GameCVarSystem : MonoBehaviour
{
    private float _defaultFixedDeltaTime;

    private void Awake()
    {
        _defaultFixedDeltaTime = Time.fixedDeltaTime;
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Bootstrap()
    {
        GameObject go = new GameObject("[GameCVarSystem]");
        go.AddComponent<GameCVarSystem>();
        DontDestroyOnLoad(go);
    }

    private void Update()
    {
        float scale = Mathf.Max(0.0f, GameCVars.TimeScale.Value);

        Time.timeScale = scale;
        Time.fixedDeltaTime = _defaultFixedDeltaTime * scale;
    }
}