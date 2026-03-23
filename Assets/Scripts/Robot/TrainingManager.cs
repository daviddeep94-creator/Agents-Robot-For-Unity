using UnityEngine;
using UnityEditor;
using Unity.MLAgents;

/// <summary>
/// 训练管理器
/// 用于管理训练场景、参数配置和训练流程
/// </summary>
public class TrainingManager : MonoBehaviour
{
    [Header("训练模式")]
    [Tooltip("当前训练阶段")]
    [SerializeField] private TrainingStage currentStage = TrainingStage.Balance;

    [Header("平衡训练配置")]
    [SerializeField] private RobotBalanceAgent balanceAgent;
    [SerializeField] private int balanceMaxEpisodes = 10000;
    [SerializeField] private float balanceTargetReward = 50f;

    [Header("移动训练配置")]
    [SerializeField] private RobotMovementAgent movementAgent;
    [SerializeField] private int movementMaxEpisodes = 20000;
    [SerializeField] private float movementTargetReward = 100f;

    [Header("训练参数")]
    [SerializeField] private float maxEpisodeTime = 30f;
    [SerializeField] private bool autoAdvanceStage = true;
    [SerializeField] private bool showTrainingUI = true;

    [Header("训练统计")]
    [SerializeField, ReadOnly] private int currentEpisode;
    [SerializeField, ReadOnly] private float totalReward;
    [SerializeField, ReadOnly] private float averageReward;
    [SerializeField, ReadOnly] private float bestReward;
    [SerializeField, ReadOnly] private float episodeTime;

    // 训练历史
    private System.Collections.Generic.List<float> rewardHistory = new System.Collections.Generic.List<float>();
    private float episodeStartTime;

    public enum TrainingStage
    {
        Balance,      // 阶段1：站立平衡
        Movement      // 阶段2：向目标移动
    }

    private void Start()
    {
        // 延迟一帧初始化，确保所有 Agent 的 Start() 已执行
        Invoke(nameof(InitializeTraining), 0.1f);
    }

    /// <summary>
    /// 初始化训练
    /// </summary>
    private void InitializeTraining()
    {
        // 查找Agent（如果没有手动指定）
        if (balanceAgent == null)
        {
            balanceAgent = FindObjectOfType<RobotBalanceAgent>();
        }
        if (movementAgent == null)
        {
            movementAgent = FindObjectOfType<RobotMovementAgent>();
        }

        // 设置最大步数
        if (balanceAgent != null)
        {
            balanceAgent.MaxStep = Mathf.RoundToInt(maxEpisodeTime / Time.fixedDeltaTime);
        }
        if (movementAgent != null)
        {
            movementAgent.MaxStep = Mathf.RoundToInt(maxEpisodeTime / Time.fixedDeltaTime);
        }

        // 激活当前阶段的Agent
        SetActiveAgent(currentStage);

        Debug.Log($"TrainingManager: 训练初始化完成，当前阶段: {currentStage}");
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

        // 检查是否应该进入下一阶段
        CheckStageProgression();
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
    /// 检查阶段进度
    /// </summary>
    private void CheckStageProgression()
    {
        if (!autoAdvanceStage) return;

        switch (currentStage)
        {
            case TrainingStage.Balance:
                // 平衡训练达标：达到目标奖励或训练完成足够多episode
                if (averageReward >= balanceTargetReward || currentEpisode >= balanceMaxEpisodes)
                {
                    AdvanceToStage(TrainingStage.Movement);
                }
                break;

            case TrainingStage.Movement:
                // 移动训练达标
                if (averageReward >= movementTargetReward || currentEpisode >= movementMaxEpisodes)
                {
                    Debug.Log("TrainingManager: 所有训练阶段完成！");
                }
                break;
        }
    }

    /// <summary>
    /// 进入下一训练阶段
    /// </summary>
    public void AdvanceToStage(TrainingStage newStage)
    {
        Debug.Log($"TrainingManager: 从阶段 {currentStage} 进入阶段 {newStage}");

        currentStage = newStage;
        SetActiveAgent(newStage);

        // 重置统计
        currentEpisode = 0;
        rewardHistory.Clear();
        bestReward = 0f;
        averageReward = 0f;
    }

    /// <summary>
    /// 设置活跃的Agent
    /// </summary>
    private void SetActiveAgent(TrainingStage stage)
    {
        if (balanceAgent != null)
        {
            balanceAgent.enabled = (stage == TrainingStage.Balance);
        }

        if (movementAgent != null)
        {
            movementAgent.enabled = (stage == TrainingStage.Movement);
        }
    }

    /// <summary>
    /// 保存训练数据
    /// </summary>
    public void SaveTrainingData()
    {
        string path = $"TrainingData_{System.DateTime.Now:yyyyMMdd_HHmmss}.json";
        string json = JsonUtility.ToJson(new TrainingData
        {
            stage = currentStage.ToString(),
            currentEpisode = currentEpisode,
            totalReward = totalReward,
            averageReward = averageReward,
            bestReward = bestReward,
            rewardHistory = rewardHistory.ToArray()
        }, true);

        System.IO.File.WriteAllText(path, json);
        Debug.Log($"TrainingManager: 训练数据已保存到 {path}");
    }

    [System.Serializable]
    private class TrainingData
    {
        public string stage;
        public int currentEpisode;
        public float totalReward;
        public float averageReward;
        public float bestReward;
        public float[] rewardHistory;
    }

    /// <summary>
    /// 手动切换训练阶段
    /// </summary>
    public void SwitchToBalanceStage()
    {
        AdvanceToStage(TrainingStage.Balance);
    }

    /// <summary>
    /// 手动切换训练阶段
    /// </summary>
    public void SwitchToMovementStage()
    {
        AdvanceToStage(TrainingStage.Movement);
    }

    /// <summary>
    /// 重置训练
    /// </summary>
    public void ResetTraining()
    {
        currentEpisode = 0;
        rewardHistory.Clear();
        bestReward = 0f;
        averageReward = 0f;
        totalReward = 0f;

        if (balanceAgent != null) balanceAgent.EndEpisode();
        if (movementAgent != null) movementAgent.EndEpisode();

        Debug.Log("TrainingManager: 训练已重置");
    }

    private void OnGUI()
    {
        if (!showTrainingUI) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 450));
        GUILayout.BeginVertical("box");

        GUILayout.Label("训练管理器", EditorStyles.boldLabel);

        // 当前阶段
        GUILayout.Label($"阶段: {currentStage}");

        // 统计信息
        GUILayout.Space(10);
        GUILayout.Label("训练统计", EditorStyles.boldLabel);
        GUILayout.Label($"Episode: {currentEpisode}");
        GUILayout.Label($"当前奖励: {totalReward:F2}");
        GUILayout.Label($"平均奖励: {averageReward:F2}");
        GUILayout.Label($"最佳奖励: {bestReward:F2}");
        GUILayout.Label($"Episode时间: {episodeTime:F1}s");
        GUILayout.Label($"时间速度: {Time.timeScale:F1}x");

        // 阶段控制
        GUILayout.Space(10);
        GUILayout.Label("阶段控制", EditorStyles.boldLabel);
        if (GUILayout.Button("平衡训练"))
        {
            SwitchToBalanceStage();
        }
        if (GUILayout.Button("移动训练"))
        {
            SwitchToMovementStage();
        }

        // 训练控制
        GUILayout.Space(10);
        GUILayout.Label("训练控制", EditorStyles.boldLabel);
        if (GUILayout.Button("重置训练"))
        {
            ResetTraining();
        }
        if (GUILayout.Button("保存数据"))
        {
            SaveTrainingData();
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
