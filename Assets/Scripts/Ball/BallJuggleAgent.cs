using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using System.Linq;

/// <summary>
/// 改进版颠球Agent（可收敛）
/// </summary>
public class BallJuggleAgent : Agent
{
    [Header("场景对象")]
    [SerializeField] private Rigidbody ball;
    [SerializeField] private Rigidbody player;

    [Header("运动配置")]
    [SerializeField] private float moveSpeed = 8f;
    [SerializeField] private float maxSpeed = 10f;  // 最大速度限制

    [Header("重置配置")]
    [SerializeField] private float startOffsetRange = 2;

    private Vector3 playPosition;
    bool collied = false;
    float previousDistanceToBall = 0f;
    int juggleCount = 0;
    float previousBallY = 5f;  // 记录上一帧球的高度

    // 统计信息
    private int totalJuggleCount = 0;
    private float trainingStartTime;
    private List<float> juggleTimestamps = new List<float>();

    [Header("训练控制")]
    [Range(1f, 100f)]
    [SerializeField] private float timeScale = 20f;
    private Rect guiRect = new Rect(10, 10, 200, 120);

    private void Start()
    {
        playPosition = player.position;
        trainingStartTime = Time.time;
    }

    public override void OnEpisodeBegin()
    {
        // 重置板子
        player.position = playPosition;
        player.velocity = Vector3.zero;
        player.angularVelocity = Vector3.zero;

        // 重置球
        float randomX = Random.Range(-startOffsetRange, startOffsetRange);
        float randomZ = Random.Range(-startOffsetRange, startOffsetRange);

        ball.transform.rotation = Quaternion.identity;
        ball.position = new Vector3(randomX, 5, randomZ);
        ball.velocity = Vector3.zero;
        ball.angularVelocity = Vector3.zero;

        collied = false;
        previousDistanceToBall = Vector3.Distance(player.position, ball.position);
        juggleCount = 0;
        previousBallY = 5f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 使用相对坐标：球相对于板子的位置和速度
        Vector3 relativePosition = ball.position - player.position;
        sensor.AddObservation(relativePosition.x);    // 1d 相对X位置
        sensor.AddObservation(relativePosition.y);    // 1d 相对Y位置
        sensor.AddObservation(relativePosition.z);    // 1d 相对Z位置

        sensor.AddObservation(ball.velocity.x);       // 1d 球的速度X
        sensor.AddObservation(ball.velocity.y);       // 1d 球的速度Y
        sensor.AddObservation(ball.velocity.z);       // 1d 球的速度Z

        // 添加板子的速度（用于判断板子运动趋势）
        sensor.AddObservation(player.velocity.x);    // 1d 板子速度X
        sensor.AddObservation(player.velocity.y);    // 1d 板子速度Y
        sensor.AddObservation(player.velocity.z);    // 1d 板子速度Z
        // 总计：9维观测空间
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];

        if (player.transform.position.y <= 1)
        {
            controlSignal.y = actions.ContinuousActions[2]*10;
        }

        player.AddForce(controlSignal * moveSpeed);

        // 限制最大速度，防止速度无限累积
        if (player.velocity.magnitude > maxSpeed)
        {
            player.velocity = player.velocity.normalized * maxSpeed;
        }

        // 计算球相对于板子中心的位置（用于判断是否用中心接球）
        Vector3 relativePosition = ball.position - player.position;
        float distanceFromCenter = new Vector2(relativePosition.x, relativePosition.z).magnitude;

        // 【关键】让球保持在中间：球越靠中间分越高
        // 球在板子中心附近给持续奖励，鼓励把球接在中心
        float centerReward = 1f - Mathf.Clamp01(distanceFromCenter / 1.5f); // 1.5m范围内
        AddReward(centerReward * 0.2f); // 提高系数，让这个奖励更明显

        // 额外：如果球非常接近中心（<0.5m），给额外奖励
        if (distanceFromCenter < 0.5f)
        {
            AddReward(0.05f);
        }

        // 速度控制奖励：鼓励板子保持平稳
        if (player.velocity.magnitude < maxSpeed * 0.5f)
        {
            AddReward(0.01f);
        }

        // 计算到球的距离
        float currentDistanceToBall = Vector3.Distance(player.position, ball.position);

        // 距离奖励：靠近球加分，远离球扣分（系数保持一致）
        float distanceDelta = previousDistanceToBall - currentDistanceToBall;
        AddReward(distanceDelta * 0.005f); // 降低距离奖励权重
        previousDistanceToBall = currentDistanceToBall;

        // 碰球奖励：碰到球加一点分
        bool hasCollided = collied;
        if (hasCollided)
        {
            // 中心接球给更高奖励
            float collisionReward = 0.5f + centerReward * 1.5f; // 0.5~2.0
            AddReward(collisionReward);
            collied = false;
        }

        // 成功颠球奖励：碰撞后球向上运动
        float currentY = ball.position.y;
        float deltaY = currentY - previousBallY;

        // 刚碰撞过，且球开始向上运动
        if (hasCollided && deltaY > 0.1f && previousBallY < 3f)
        {
            juggleCount++;
            totalJuggleCount++;
            juggleTimestamps.Add(Time.time);

            // 中心接球且颠球成功，给大奖励
            float juggleReward = 2f + centerReward * 3f; // 2~5分
            AddReward(juggleReward);
        }
        previousBallY = currentY;

        // 偏离中心惩罚：如果球偏离板子中心太多，给惩罚
        if (distanceFromCenter > 2f)
        {
            // 离中心越远，惩罚越大
            float penalty = (distanceFromCenter - 2f) * 0.1f;
            AddReward(-penalty);
        }

        // 失败惩罚
        if (ball.transform.position.y <= 1.5 || Mathf.Abs(player.transform.position.x) > 10 || Mathf.Abs(player.transform.position.z) > 10)
        {
            AddReward(-2);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject == ball.gameObject)
        {
            collied = true;
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        act[0] = Input.GetAxis("Horizontal");
        act[1] = Input.GetAxis("Vertical");
    }

    private void OnGUI()
    {
        Time.timeScale = timeScale;

        // 计算训练时长
        float trainingDuration = Time.time - trainingStartTime;
        int minutes = Mathf.FloorToInt(trainingDuration / 60f);
        int seconds = Mathf.FloorToInt(trainingDuration % 60f);

        // 计算最近10分钟内的颠球次数
        float currentTime = Time.time;
        int recentJuggleCount = juggleTimestamps.Count(t => currentTime - t <= 600f);

        GUI.Box(guiRect, "");
        GUILayout.BeginArea(guiRect);
        GUILayout.Label($"训练速度: {timeScale:F1}x");
        timeScale = GUILayout.HorizontalSlider(timeScale, 1f, 100f, GUILayout.Width(180));
        GUILayout.Space(10);
        GUILayout.Label($"训练时长: {minutes:00}:{seconds:00}");
        GUILayout.Label($"总颠球数: {totalJuggleCount}");
        GUILayout.Label($"最近10分钟: {recentJuggleCount}/10分钟");
        GUILayout.EndArea();
    }
}