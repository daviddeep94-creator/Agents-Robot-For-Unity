using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// 机器人站立平衡训练Agent
/// 使用PPO算法训练机器人在地面上保持平衡站立
/// </summary>
public class RobotBalanceAgent : Agent
{
    [Header("机器人配置")]
    [Tooltip("机器人根节点ArticulationBody")]
    [SerializeField] private ArticulationBody robotRoot;

    [Tooltip("控制力放大")]
    [SerializeField] private float force = 50f;

    [Header("训练目标")]
    [Tooltip("目标站立时间（秒），达到后给予奖励")]
    [SerializeField] private float targetStandTime = 10f;

    [Tooltip("机器人重心目标高度（米）")]
    [SerializeField] private float targetHeight = 1f;

    [Tooltip("最大允许的倾斜角度（度）")]
    [SerializeField] private float maxTiltAngle = 30f;

    [Header("奖励配置")]
    [Tooltip("每帧保持直立的基础奖励")]
    [SerializeField] private float uprightReward = 0.01f;

    [Tooltip("高度接近目标的奖励系数")]
    [SerializeField] private float heightRewardMultiplier = 10f;

    [Tooltip("倾斜惩罚系数")]
    [SerializeField] private float tiltPenaltyMultiplier = 0.1f;

    [Tooltip("关节移动惩罚系数（避免过度移动）")]
    [SerializeField] private float jointMovementPenalty = 0.001f;

    [Tooltip("最终完成奖励")]
    [SerializeField] private float completionReward = 1f;

    [Header("动作配置")]
    [Tooltip("关节动作幅度")]
    [SerializeField] private float jointForceScale = 100f;

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = true;

    [Header("训练管理")]
    [SerializeField] private TrainingManager trainingManager;

    // 状态变量
    private float episodeTimer;
    private float currentHeight;
    private float currentTiltAngle;
    private float previousHeight;
    private ArticulationBody[] allJoints;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private bool isGrounded;

    // 关节索引映射
    private int pelvisIndex, torsoIndex, neckIndex;
    private int leftHipIndex, rightHipIndex;
    private int leftKneeIndex, rightKneeIndex;
    private int leftAnkleIndex, rightAnkleIndex;
    private int leftShoulderIndex, rightShoulderIndex;
    private int leftElbowIndex, rightElbowIndex;

    // 前一帧关节角度
    private float[] previousJointAngles;

    // 保存的初始关节状态（用于重置）
    private Vector3[] initialJointPositions;
    private Quaternion[] initialJointRotations;
    private float[] initialJointXTargets;
    private float[] initialJointYTargets;
    private float[] initialJointZTargets;

    // 当前 drive target 基线值（用于相对计算）
    private float[] baselineJointXTargets;
    private float[] baselineJointYTargets;
    private float[] baselineJointZTargets;

    // 标记是否正在重置，避免重复触发
    private bool isResetting = false;

    // 跳过重置后的前几帧动作，让物理稳定
    private int framesSinceReset = 0;
    private const int RESET_STABLE_FRAMES = 3;

    private void Awake()
    {
        // 确保组件被启用（即使组件被禁用，Awake 也会被调用）
        if (!this.enabled)
        {
            this.enabled = true;
            Debug.LogWarning("[RobotBalanceAgent] 组件被禁用，已自动启用");
        }
    }

    private void Start()
    {
        Debug.Log("[RobotBalanceAgent] Start() 被调用");

        // 自动查找 TrainingManager（如果未手动指定）
        if (trainingManager == null)
        {
            trainingManager = FindObjectOfType<TrainingManager>();
            if (trainingManager != null)
            {
                Debug.Log("[RobotBalanceAgent] 自动找到 TrainingManager");
            }
        }

        InitializeRobot();
    }

    /// <summary>
    /// 初始化机器人引用和关节索引
    /// </summary>
    private void InitializeRobot()
    {
        if (robotRoot == null)
        {
            robotRoot = GetComponent<ArticulationBody>();
        }

        if (robotRoot == null)
        {
            Debug.LogError("RobotBalanceAgent: 未找到机器人根节点ArticulationBody！");
            return;
        }

        // 获取所有关节
        allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        Debug.Log($"[RobotBalanceAgent] 找到 {allJoints.Length} 个 ArticulationBody:");

        // 打印所有关节名称用于调试
        for (int i = 0; i < allJoints.Length; i++)
        {
            if (allJoints[i] != null)
            {
                Debug.Log($"  [{i}] {allJoints[i].name}");
            }
        }

        // 查找并索引各个关节
        pelvisIndex = FindJointIndex("Pelvis_Joint");
        torsoIndex = FindJointIndex("Torso_Joint");
        neckIndex = FindJointIndex("Neck_Joint");
        leftHipIndex = FindJointIndex("Left_HipJoint");
        rightHipIndex = FindJointIndex("Right_HipJoint");
        leftKneeIndex = FindJointIndex("Left_KneeJoint");
        rightKneeIndex = FindJointIndex("Right_KneeJoint");
        leftAnkleIndex = FindJointIndex("Left_AnkleJoint");
        rightAnkleIndex = FindJointIndex("Right_AnkleJoint");
        leftShoulderIndex = FindJointIndex("Left_ShoulderJoint");
        rightShoulderIndex = FindJointIndex("Right_ShoulderJoint");
        leftElbowIndex = FindJointIndex("Left_ElbowJoint");
        rightElbowIndex = FindJointIndex("Right_ElbowJoint");

        Debug.Log($"[RobotBalanceAgent] 关节索引: Torso={torsoIndex}, Neck={neckIndex}, L_Hip={leftHipIndex}, R_Hip={rightHipIndex}");

        // 初始化前一帧角度数组
        previousJointAngles = new float[12];

        // 保存所有关节的初始状态
        SaveInitialJointStates();
    }

    /// <summary>
    /// 保存所有关节的初始状态
    /// </summary>
    private void SaveInitialJointStates()
    {
        RobotResetUtility.RobotInitialStates states = RobotResetUtility.SaveInitialStates(robotRoot);

        initialJointPositions = states.positions;
        initialJointRotations = states.rotations;
        initialJointXTargets = states.xTargets;
        initialJointYTargets = states.yTargets;
        initialJointZTargets = states.zTargets;
        baselineJointXTargets = (float[])states.xTargets.Clone();
        baselineJointYTargets = (float[])states.yTargets.Clone();
        baselineJointZTargets = (float[])states.zTargets.Clone();

        Debug.Log($"[RobotBalanceAgent] 已保存 {states.jointCount} 个关节的初始状态");
    }

    /// <summary>
    /// 查找关节索引
    /// </summary>
    private int FindJointIndex(string jointName)
    {
        for (int i = 0; i < allJoints.Length; i++)
        {
            if (allJoints[i] != null && allJoints[i].name == jointName)
            {
                return i;
            }
        }
        Debug.LogWarning($"RobotBalanceAgent: 未找到关节 {jointName}");
        return -1;
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin(); // 必须调用父类的方法

        Debug.Log($"[RobotBalanceAgent] ========== OnEpisodeBegin 被调用 ==========");
        Debug.Log($"[RobotBalanceAgent] 当前机器人位置: {robotRoot.transform.position}");

        // 防止重置过程中重复调用
        if (isResetting)
        {
            Debug.LogWarning("[RobotBalanceAgent] OnEpisodeBegin: 正在重置中，跳过本次调用");
            return;
        }

        episodeTimer = 0f;
        previousHeight = 0f;
        framesSinceReset = 0; // 重置帧计数器

        // 记录初始位置和旋转（从保存的状态获取）
        int rootIndex = -1;
        for (int i = 0; i < allJoints.Length; i++)
        {
            if (allJoints[i] == robotRoot)
            {
                rootIndex = i;
                break;
            }
        }

        if (rootIndex >= 0)
        {
            startPosition = initialJointPositions[rootIndex];
            startRotation = initialJointRotations[rootIndex];
            Debug.Log($"[RobotBalanceAgent] 保存的初始位置: {startPosition}");
        }

        // 重置机器人姿态到初始状态
        Debug.Log("[RobotBalanceAgent] 准备调用 ResetRobotPose");
        ResetRobotPose();
        Debug.Log("[RobotBalanceAgent] ResetRobotPose 调用完成");

        // 初始化当前状态，避免第一帧误判
        currentHeight = robotRoot.transform.position.y;

        // 使用躯干或臀部的旋转来判断倾角
        Transform bodyTransform = null;
        if (torsoIndex >= 0 && allJoints[torsoIndex] != null)
        {
            bodyTransform = allJoints[torsoIndex].transform;
        }
        else if (pelvisIndex >= 0 && allJoints[pelvisIndex] != null)
        {
            bodyTransform = allJoints[pelvisIndex].transform;
        }

        if (bodyTransform != null)
        {
            currentTiltAngle = Vector3.Angle(bodyTransform.up, Vector3.up);
        }
        else
        {
            currentTiltAngle = Vector3.Angle(robotRoot.transform.up, Vector3.up);
        }

        Debug.Log($"[RobotBalanceAgent] OnEpisodeBegin 结束:{bodyTransform.name} 高度={currentHeight:F2}m, 倾角={currentTiltAngle:F1}°, 位置={robotRoot.transform.position}");

        // 通知 TrainingManager
        if (trainingManager != null)
        {
            trainingManager.OnEpisodeStart();
        }

        if (showDebugInfo)
        {
            Debug.Log("Episode Start: 重置机器人姿态");
        }
    }

    /// <summary>
    /// 重置机器人姿态到初始状态
    /// </summary>
    private void ResetRobotPose()
    {
        isResetting = true;

        RobotResetUtility.ResetRobotPose(
            robotRoot,
            initialJointPositions,
            initialJointRotations,
            initialJointXTargets,
            initialJointYTargets,
            initialJointZTargets,
            ref baselineJointXTargets,
            ref baselineJointYTargets,
            ref baselineJointZTargets,
            "[RobotBalanceAgent] ResetRobotPose:"
        );

        isResetting = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotRoot == null || allJoints == null || allJoints.Length == 0)
        {
            Debug.LogError($"[RobotBalanceAgent] CollectObservations: 机器人或关节未初始化 - robotRoot={robotRoot != null}, allJoints={allJoints?.Length}");
            return;
        }

        Debug.Log($"[RobotBalanceAgent] CollectObservations 被调用 - Step={StepCount}");

        // 1. 机器人位置（归一化）
        sensor.AddObservation(robotRoot.transform.position.y / targetHeight);

        // 2. 机器人旋转（欧拉角，归一化到-1到1）
        Vector3 euler = robotRoot.transform.rotation.eulerAngles;
        sensor.AddObservation(NormalizeAngle(euler.x));
        sensor.AddObservation(NormalizeAngle(euler.y));
        sensor.AddObservation(NormalizeAngle(euler.z));

        // 3. 机器人速度和角速度（归一化）
        sensor.AddObservation(Mathf.Clamp(robotRoot.velocity.y / 2f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.x / 5f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.y / 5f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.z / 5f, -1f, 1f));

        // 4. 各关节角度（归一化）
        float[] jointAngles = GetJointAngles();
        foreach (float angle in jointAngles)
        {
            sensor.AddObservation(NormalizeAngle(angle));
        }

        // 5. 各关节速度（归一化）
        float[] jointVelocities = GetJointVelocities();
        foreach (float velocity in jointVelocities)
        {
            sensor.AddObservation(Mathf.Clamp(velocity / 5f, -1f, 1f));
        }

        // 6. 重力方向（相对机器人）
        Vector3 localGravity = robotRoot.transform.InverseTransformDirection(Physics.gravity);
        sensor.AddObservation(localGravity.normalized);

        // 总观测维度: 1 + 3 + 1 + 3 + 12 + 12 + 3 = 35
    }

    /// <summary>
    /// 归一化角度到-1到1范围
    /// </summary>
    private float NormalizeAngle(float angle)
    {
        angle = (angle + 180f) % 360f - 180f;
        return Mathf.Clamp(angle / 180f, -1f, 1f);
    }

    /// <summary>
    /// 获取所有关节的当前角度
    /// </summary>
    private float[] GetJointAngles()
    {
        float[] angles = new float[12];
        int index = 0;

        // 臀部、躯干、颈部
        if (torsoIndex >= 0 && torsoIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[torsoIndex]);
        if (neckIndex >= 0 && neckIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[neckIndex]);

        // 左腿
        if (leftHipIndex >= 0 && leftHipIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[leftHipIndex]);
        if (leftKneeIndex >= 0 && leftKneeIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[leftKneeIndex]);
        if (leftAnkleIndex >= 0 && leftAnkleIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[leftAnkleIndex]);

        // 右腿
        if (rightHipIndex >= 0 && rightHipIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[rightHipIndex]);
        if (rightKneeIndex >= 0 && rightKneeIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[rightKneeIndex]);
        if (rightAnkleIndex >= 0 && rightAnkleIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[rightAnkleIndex]);

        // 左臂
        if (leftShoulderIndex >= 0 && leftShoulderIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[leftShoulderIndex]);
        if (leftElbowIndex >= 0 && leftElbowIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[leftElbowIndex]);

        // 右臂
        if (rightShoulderIndex >= 0 && rightShoulderIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[rightShoulderIndex]);
        if (rightElbowIndex >= 0 && rightElbowIndex < allJoints.Length) angles[index++] = GetJointAngle(allJoints[rightElbowIndex]);

        return angles;
    }

    /// <summary>
    /// 获取单个关节的角度
    /// </summary>
    private float GetJointAngle(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        return joint.xDrive.target; // 使用xDrive作为主要旋转轴
    }

    /// <summary>
    /// 获取所有关节的角速度
    /// </summary>
    private float[] GetJointVelocities()
    {
        float[] velocities = new float[12];
        int index = 0;

        foreach (var joint in allJoints)
        {
            if (joint != null && joint.jointType != ArticulationJointType.FixedJoint)
            {
                if (index < 12)
                {
                    velocities[index++] = joint.angularVelocity.magnitude;
                }
            }
        }

        return velocities;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (robotRoot == null)
        {
            Debug.LogError("[RobotBalanceAgent] OnActionReceived: robotRoot 为 null！");
            return;
        }

        Debug.Log($"[RobotBalanceAgent] OnActionReceived 被调用: Step={StepCount}");


        // 应用关节力矩（12个连续动作对应12个关节）26d
        ApplyJointForces(actions.ContinuousActions);

        // 计算奖励
        CalculateAndApplyReward();

        // 检查终止条件
        CheckTermination();

        episodeTimer += Time.fixedDeltaTime;
    }

    /// <summary>
    /// 应用关节力矩
    /// </summary>
    private void ApplyJointForces(ActionSegment<float> actions)
    {
        int index = 0;

        // 身体（1轴）
        if (torsoIndex >= 0)
        {
            ApplyForceToJoint(allJoints[torsoIndex], actions[index++], 0, 0);
        }

        // 脖子（3轴）
        if (neckIndex >= 0)
        {
            ApplyForceToJoint(allJoints[neckIndex], actions[index++], actions[index++], actions[index++]);
        }

        // 左腿
        if (leftHipIndex >= 0)  // 髋关节（3轴）
        {
            ApplyForceToJoint(allJoints[leftHipIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (leftKneeIndex >= 0)  // 膝关节（1轴）
        {
            ApplyForceToJoint(allJoints[leftKneeIndex], actions[index++], 0, 0);
        }
        if (leftAnkleIndex >= 0)  // 踝关节（3轴）
        {
            ApplyForceToJoint(allJoints[leftAnkleIndex], actions[index++], actions[index++], actions[index++]);
        }

        // 右腿
        if (rightHipIndex >= 0)  // 髋关节（3轴）
        {
            ApplyForceToJoint(allJoints[rightHipIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (rightKneeIndex >= 0)  // 膝关节（1轴）
        {
            ApplyForceToJoint(allJoints[rightKneeIndex], actions[index++], 0, 0);
        }
        if (rightAnkleIndex >= 0)  // 踝关节（3轴）
        {
            ApplyForceToJoint(allJoints[rightAnkleIndex], actions[index++], actions[index++], actions[index++]);
        }

        // 左臂
        if (leftShoulderIndex >= 0)  // 肩关节（3轴）
        {
            ApplyForceToJoint(allJoints[leftShoulderIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (leftElbowIndex >= 0)  // 肘关节（1轴）
        {
            ApplyForceToJoint(allJoints[leftElbowIndex], actions[index++], 0, 0);
        }

        // 右臂
        if (rightShoulderIndex >= 0)  // 肩关节（3轴）
        {
            ApplyForceToJoint(allJoints[rightShoulderIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (rightElbowIndex >= 0)  // 肘关节（1轴）
        {
            ApplyForceToJoint(allJoints[rightElbowIndex], actions[index++], 0, 0);
        }
    }

    /// <summary>
    /// 应用力矩到单个关节
    /// </summary>
    private void ApplyForceToJoint(ArticulationBody joint, float xForce, float yForce, float zForce)
    {
        if (joint == null) return;

        int index = System.Array.IndexOf(allJoints, joint);
        if (index < 0) return;

        xForce *= force;
        yForce *= force;
        zForce *= force;

        // 获取当前 drive 目标
        var xDrive = joint.xDrive;
        var yDrive = joint.yDrive;
        var zDrive = joint.zDrive;

        // 计算相对于基线的增量
        float deltaX = xDrive.target - baselineJointXTargets[index];
        float deltaY = yDrive.target - baselineJointYTargets[index];
        float deltaZ = zDrive.target - baselineJointZTargets[index];

        // 应用新的力矩（基于基线）
        xDrive.target = baselineJointXTargets[index] + deltaX + xForce * jointForceScale * Time.fixedDeltaTime;
        joint.xDrive = xDrive;

        if (joint.jointType == ArticulationJointType.SphericalJoint)
        {
            yDrive.target = baselineJointYTargets[index] + deltaY + yForce * jointForceScale * Time.fixedDeltaTime;
            joint.yDrive = yDrive;

            zDrive.target = baselineJointZTargets[index] + deltaZ + zForce * jointForceScale * Time.fixedDeltaTime;
            joint.zDrive = zDrive;
        }
    }

    /// <summary>
    /// 计算并应用奖励
    /// </summary>
    private void CalculateAndApplyReward()
    {
        if (robotRoot == null) return;

        // 获取当前状态
        currentHeight = robotRoot.transform.position.y;

        // 使用躯干或臀部的旋转来判断倾角（使用世界坐标系的up方向）
        Transform bodyTransform = null;
        if (torsoIndex >= 0 && allJoints[torsoIndex] != null)
        {
            bodyTransform = allJoints[torsoIndex].transform;
        }
        else if (pelvisIndex >= 0 && allJoints[pelvisIndex] != null)
        {
            bodyTransform = allJoints[pelvisIndex].transform;
        }

        if (bodyTransform != null)
        {
            // 使用Transform的forward方向，因为ArticulationBody的up可能不准确
            // 机器人直立时forward应该指向世界坐标系的前方
            Vector3 worldUp = bodyTransform.rotation * Vector3.up;
            currentTiltAngle = Vector3.Angle(worldUp, Vector3.up);

            // 每100帧打印调试信息
            if (StepCount % 100 == 0)
            {
                Debug.Log($"[RobotBalanceAgent] {bodyTransform.name} - localRotation: {bodyTransform.localRotation.eulerAngles:F1}, worldUp angle: {currentTiltAngle:F1}°");
            }
        }
        else
        {
            // 如果找不到身体部位，使用根节点（虽然不准确）
            currentTiltAngle = Vector3.Angle(robotRoot.transform.up, Vector3.up);
        }

        // 1. 直立奖励（保持在一定高度且不倾斜）
        float heightError = Mathf.Abs(currentHeight - targetHeight);
        float heightReward = Mathf.Exp(-heightError) * heightRewardMultiplier;

        // 2. 倾斜惩罚
        float tiltPenalty = (currentTiltAngle / maxTiltAngle) * tiltPenaltyMultiplier;

        // 3. 高度变化奖励（接近目标高度）
        float heightChangeReward = 0f;
        if (previousHeight > 0f)
        {
            float heightDelta = currentHeight - previousHeight;
            // 向目标高度移动给予奖励
            if (heightDelta > 0 && currentHeight < targetHeight)
            {
                heightChangeReward = heightDelta * heightRewardMultiplier;
            }
        }

        // 4. 关节移动惩罚（避免过度摆动）
        float movementPenalty = 0f;
        float[] currentJointAngles = GetJointAngles();
        for (int i = 0; i < Mathf.Min(previousJointAngles.Length, currentJointAngles.Length); i++)
        {
            movementPenalty += Mathf.Abs(currentJointAngles[i] - previousJointAngles[i]);
        }
        movementPenalty *= jointMovementPenalty;

        // 更新前一帧角度
        previousJointAngles = currentJointAngles;

        // 5. 基础存活奖励
        float survivalReward = uprightReward;

        // 总奖励
        float totalReward = survivalReward + heightReward + heightChangeReward
                          - tiltPenalty - movementPenalty;

        AddReward(totalReward);

        if (showDebugInfo)
        {
            Debug.Log($"Reward: {totalReward:F4} | Height: {heightReward:F4} | Tilt: {-tiltPenalty:F4} | Movement: {-movementPenalty:F4}");
        }
    }

    /// <summary>
    /// 检查终止条件
    /// </summary>
    private void CheckTermination()
    {
        if (robotRoot == null) return;

        // 每100帧打印一次状态用于调试
        if (StepCount % 100 == 0)
        {
            Debug.Log($"[RobotBalanceAgent] 状态检查 - Step:{StepCount}, 高度:{currentHeight:F2}m, 倾角:{currentTiltAngle:F1}°, 时间:{episodeTimer:F1}s");
        }

        // 失败条件1：倾斜过度
        if (currentTiltAngle > maxTiltAngle)
        {
            Debug.Log($"[RobotBalanceAgent] 失败: 倾斜角度 {currentTiltAngle:F1}° > {maxTiltAngle}°");
            AddReward(-1f); // 失败惩罚
            //EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 倾斜角度 {currentTiltAngle:F1}° 超过 {maxTiltAngle}°");
            }
            return;
        }

        // 失败条件2：高度过低（摔倒）
        if (currentHeight < targetHeight * 0.5f)
        {
            Debug.Log($"[RobotBalanceAgent] 失败: 高度 {currentHeight:F2}m < {targetHeight * 0.5f:F2}m (阈值)");
            AddReward(-1f);
            //EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 高度 {currentHeight:F2}m 低于阈值");
            }
            return;
        }

        // 失败条件3：偏离初始位置过远
        float horizontalDistance = Vector2.Distance(
            new Vector2(robotRoot.transform.position.x, robotRoot.transform.position.z),
            new Vector2(startPosition.x, startPosition.z)
        );
        if (horizontalDistance > 0.5f)
        {
            AddReward(-0.5f);
            //EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 偏离位置 {horizontalDistance:F2}m");
            }
            return;
        }

        // 成功条件：达到目标站立时间
        if (episodeTimer >= targetStandTime)
        {
            AddReward(completionReward);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Success: 成功站立 {episodeTimer:F1}秒");
            }
        }
    }

    /// <summary>
    /// 结束 Episode 并通知 TrainingManager
    /// </summary>
    private void EndEpisodeAndNotify()
    {
        // 获取当前 episode 的总奖励
        float episodeReward = GetCumulativeReward();

        // 通知 TrainingManager
        if (trainingManager != null)
        {
            trainingManager.OnEpisodeEnd(episodeReward);
        }

        Debug.Log($"[RobotBalanceAgent] Episode End: 总奖励 = {episodeReward:F2}");
    }

    /// <summary>
    /// Heuristic模式（用于人类控制测试）
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;

        // 所有动作默认为0
        for (int i = 0; i < continuousActions.Length; i++)
        {
            continuousActions[i] = 0f;
        }

        // 身体（1轴）- index 0
        continuousActions[0] = Input.GetAxis("Vertical"); // 躯干前后

        // 脖子（3轴）- index 1-3
        continuousActions[1] = Input.GetKey(KeyCode.W) ? 1f : (Input.GetKey(KeyCode.S) ? -1f : 0f); // 前后
        continuousActions[2] = Input.GetKey(KeyCode.A) ? 1f : (Input.GetKey(KeyCode.D) ? -1f : 0f); // 左右
        continuousActions[3] = Input.GetKey(KeyCode.Q) ? 1f : (Input.GetKey(KeyCode.E) ? -1f : 0f); // 上下

        // 左髋（3轴）- index 4-6
        continuousActions[4] = Input.GetKey(KeyCode.R) ? 1f : 0f;
        continuousActions[5] = Input.GetKey(KeyCode.F) ? 1f : 0f;
        continuousActions[6] = Input.GetKey(KeyCode.V) ? 1f : 0f;

        // 左膝（1轴）- index 7
        continuousActions[7] = Input.GetKey(KeyCode.T) ? 1f : 0f;

        // 左踝（3轴）- index 8-10
        continuousActions[8] = Input.GetKey(KeyCode.Y) ? 1f : 0f;
        continuousActions[9] = Input.GetKey(KeyCode.G) ? 1f : 0f;
        continuousActions[10] = Input.GetKey(KeyCode.B) ? 1f : 0f;

        // 右髋（3轴）- index 11-13
        continuousActions[11] = Input.GetKey(KeyCode.U) ? 1f : 0f;
        continuousActions[12] = Input.GetKey(KeyCode.J) ? 1f : 0f;
        continuousActions[13] = Input.GetKey(KeyCode.N) ? 1f : 0f;

        // 右膝（1轴）- index 14
        continuousActions[14] = Input.GetKey(KeyCode.I) ? 1f : 0f;

        // 右踝（3轴）- index 15-17
        continuousActions[15] = Input.GetKey(KeyCode.O) ? 1f : 0f;
        continuousActions[16] = Input.GetKey(KeyCode.K) ? 1f : 0f;
        continuousActions[17] = Input.GetKey(KeyCode.M) ? 1f : 0f;

        // 左肩（3轴）- index 18-20
        continuousActions[18] = Input.GetKey(KeyCode.Z) ? 1f : 0f;
        continuousActions[19] = Input.GetKey(KeyCode.X) ? 1f : 0f;
        continuousActions[20] = Input.GetKey(KeyCode.C) ? 1f : 0f;

        // 左肘（1轴）- index 21
        continuousActions[21] = Input.GetKey(KeyCode.Alpha1) ? 1f : 0f;

        // 右肩（3轴）- index 22-24
        continuousActions[22] = Input.GetKey(KeyCode.Alpha2) ? 1f : 0f;
        continuousActions[23] = Input.GetKey(KeyCode.Alpha3) ? 1f : 0f;
        continuousActions[24] = Input.GetKey(KeyCode.Alpha4) ? 1f : 0f;
    }

    private void OnDrawGizmos()
    {
        if (!showDebugInfo || robotRoot == null) return;

        // 绘制目标高度线
        Gizmos.color = Color.green;
        Gizmos.DrawLine(
            new Vector3(robotRoot.transform.position.x, targetHeight, robotRoot.transform.position.z),
            new Vector3(robotRoot.transform.position.x, targetHeight + 0.1f, robotRoot.transform.position.z)
        );

        // 绘制机器人朝向
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(robotRoot.transform.position, robotRoot.transform.up * 0.5f);

        // 绘制理想朝向
        Gizmos.color = Color.red;
        Gizmos.DrawRay(robotRoot.transform.position, Vector3.up * 0.5f);
    }
}
