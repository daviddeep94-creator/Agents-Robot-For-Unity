using UnityEngine;
using Unity.MLAgents;

/// <summary>
/// 训练管理器
/// 用于管理训练场景、参数配置和训练统计
/// </summary>
public class TrainingManager : MonoBehaviour
{
    [Header("时间设置")]
    [Tooltip("时间流逝速度，默认20（训练时加速）")]
    [Range(1f, 100f)]
    [SerializeField] private float timeScale = 20f;

    [Tooltip("是否在训练开始时自动设置时间速度")]
    [SerializeField] private bool autoSetTimeScale = true;

    [Header("训练统计")]
    [SerializeField, ReadOnly] private int currentEpisode;
    [SerializeField, ReadOnly] private float totalReward;
    [SerializeField, ReadOnly] private float averageReward;
    [SerializeField, ReadOnly] private float bestReward;
    [SerializeField, ReadOnly] private float episodeTime;

    // 训练历史
    private System.Collections.Generic.List<float> rewardHistory = new System.Collections.Generic.List<float>();
    private float episodeStartTime;

    private void Start()
    {
        if (autoSetTimeScale)
        {
            Time.timeScale = timeScale;
            Debug.Log($"[TrainingManager] 时间速度设置为 {timeScale}x");
        }

        // 延迟一帧初始化，确保所有 Agent 的 Start() 已执行
        Invoke(nameof(InitializeTraining), 0.1f);
    }

    /// <summary>
    /// 初始化训练
    /// </summary>
    private void InitializeTraining()
    {
        // 这里可以添加初始化逻辑，如加载配置等
        Debug.Log("[TrainingManager] 训练管理器已初始化");
    }

    private void Update()
    {
        UpdateEpisodeStats();
    }

    /// <summary>
    /// 更新Episode统计
    /// </summary>
    private void UpdateEpisodeStats()
    {
        episodeTime = Time.time - episodeStartTime;
    }

    /// <summary>
    /// 开始新Episode
    /// </summary>
    public void OnEpisodeStart()
    {
        episodeStartTime = Time.time;
        currentEpisode++;

        // 清空当前Episode奖励
        totalReward = 0f;
    }

    /// <summary>
    /// Episode结束
    /// </summary>
    public void OnEpisodeEnd(float episodeReward)
    {
        totalReward = episodeReward;
        rewardHistory.Add(episodeReward);

        // 计算平均奖励（最近100个episode）
        int windowSize = Mathf.Min(100, rewardHistory.Count);
        float sum = 0f;
        for (int i = rewardHistory.Count - windowSize; i < rewardHistory.Count; i++)
        {
            sum += rewardHistory[i];
        }
        averageReward = sum / windowSize;

        // 更新最佳奖励
        if (episodeReward > bestReward)
        {
            bestReward = episodeReward;
        }

        // 记录日志
        if (currentEpisode % 10 == 0)
        {
            Debug.Log($"Episode {currentEpisode}: Reward = {episodeReward:F2}, Avg = {averageReward:F2}, Best = {bestReward:F2}");
        }
    }

    /// <summary>
    /// 手动设置时间速度（运行时调用）
    /// </summary>
    public void SetTimeScale(float scale)
    {
        Time.timeScale = Mathf.Clamp(scale, 1f, 100f);
        timeScale = Time.timeScale;
        Debug.Log($"[TrainingManager] 时间速度手动设置为 {Time.timeScale:F1}x");
    }

    /// <summary>
    /// 暂停/恢复训练
    /// </summary>
    public void TogglePause()
    {
        Time.timeScale = Time.timeScale == 0f ? timeScale : 0f;
        Debug.Log($"[TrainingManager] 训练{(Time.timeScale == 0f ? "暂停" : "恢复")}");
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 450));
        GUILayout.BeginVertical("box");

        GUILayout.Label("训练管理器", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });

        // 统计信息
        GUILayout.Space(10);
        GUILayout.Label("训练统计", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        GUILayout.Label($"Episode: {currentEpisode}");
        GUILayout.Label($"当前奖励: {totalReward:F2}");
        GUILayout.Label($"平均奖励: {averageReward:F2}");
        GUILayout.Label($"最佳奖励: {bestReward:F2}");
        GUILayout.Label($"Episode时间: {episodeTime:F1}s");
        GUILayout.Label($"时间速度: {Time.timeScale:F1}x");

        // 控制按钮
        GUILayout.Space(10);
        GUILayout.Label("控制", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });

        if (GUILayout.Button("暂停/恢复"))
        {
            TogglePause();
        }

        if (GUILayout.Button("设置时间速度: 1x"))
        {
            SetTimeScale(1f);
        }

        if (GUILayout.Button("设置时间速度: 10x"))
        {
            SetTimeScale(10f);
        }

        if (GUILayout.Button("设置时间速度: 20x"))
        {
            SetTimeScale(20f);
        }

        if (GUILayout.Button("设置时间速度: 50x"))
        {
            SetTimeScale(50f);
        }

        GUILayout.EndVertical();
        GUILayout.EndArea();
    }
}

/// <summary>
/// ReadOnly属性，用于Inspector显示但不允许编辑
/// </summary>
public class ReadOnlyAttribute : PropertyAttribute { }

#if UNITY_EDITOR
[UnityEditor.CustomPropertyDrawer(typeof(ReadOnlyAttribute))]
public class ReadOnlyDrawer : UnityEditor.PropertyDrawer
{
    public override void OnGUI(Rect position, UnityEditor.SerializedProperty property, GUIContent label)
    {
        GUI.enabled = false;
        UnityEditor.EditorGUI.PropertyField(position, property, label, true);
        GUI.enabled = true;
    }
}
#endif

