using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;

public enum TargetType
{
    None=0,
    BalancePoint=1,
}
/// <summary>
/// 机器人站立平衡训练Agent
/// 使用PPO算法训练机器人在地面上保持平衡站立
/// </summary>
public class RobotBalanceAgent : Agent
{
    [Header("机器人配置")]
    [Tooltip("机器人根节点ArticulationBody")]
    [SerializeField] private ArticulationBody robotRoot;

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

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = true;

    [Header("目标")]
    [SerializeField] private Transform target;

    [Header("目标类型")]
    [SerializeField] private TargetType targetType = TargetType.BalancePoint;
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

    // 延迟重置机制
    private bool isFailed = false;
    private float failTimer = 0f;
    [SerializeField] private float failDelayTime = 2f; // 失败后等待时间（秒）

    private void Awake()
    {
        // 确保组件被启用（即使组件被禁用，Awake 也会被调用）
        if (!this.enabled)
        {
            this.enabled = true;
            Debug.LogWarning("[RobotBalanceAgent] 组件被禁用，已自动启用");
        }
        if(robotRoot == null)
        {
            robotRoot = GetComponentInChildren<ArticulationBody>();
            if (robotRoot != null)
            {
                Debug.Log("[RobotBalanceAgent] 自动找到 robotRoot");
            }
        }
    }

    private void Start()
    {
        Debug.Log("[RobotBalanceAgent] Start() 被调用");

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

        // 重置失败状态
        isFailed = false;
        failTimer = 0f;

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

    bool onGroundLeft, onGroundRight;
    public void OnGround(bool left)
    {
        onGroundLeft = left;
        onGroundRight = !left;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotRoot == null || allJoints == null || allJoints.Length == 0)
        {
            Debug.LogError($"[RobotBalanceAgent] CollectObservations: 机器人或关节未初始化 - robotRoot={robotRoot != null}, allJoints={allJoints?.Length}");
            return;
        }

        Debug.Log($"[RobotBalanceAgent] CollectObservations 被调用 - Step={StepCount}");

        int Ground = 0;
        if (onGroundLeft)
        {
            Ground += 1;
            onGroundLeft = false;
        }
        if (onGroundRight)
        {
            Ground += 2;
            onGroundRight = false;
        }

        int dimensionality = 0;

        // 1. 机器人根节点世界位置
        sensor.AddObservation(robotRoot.transform.position);
        dimensionality += 3;

        // 2. 机器人根节点世界旋转
        sensor.AddObservation(robotRoot.transform.rotation.eulerAngles);
        dimensionality += 3;
        // 3. 机器人根节点速度
        sensor.AddObservation(robotRoot.velocity);
        dimensionality += 3;
        // 4. 机器人根节点角速度
        sensor.AddObservation(robotRoot.angularVelocity);
        dimensionality += 3;

        // 5. 各关节局部旋转 角速度
        foreach (var joint in allJoints)
        {
            if (joint == robotRoot) continue;
            if (joint != null && joint.jointType != ArticulationJointType.FixedJoint)
            {
                sensor.AddObservation(joint.transform.localRotation.eulerAngles);
                sensor.AddObservation(joint.angularVelocity);
                dimensionality += 6;
            }
        }
        sensor.AddObservation(Ground);
        dimensionality += 1;

        sensor.AddObservation((int)targetType);
        sensor.AddObservation(target.position);
        dimensionality += 4;
        Debug.Log($"总维度:{dimensionality}");
        // 总观测维度: 1(位置) + 3(世界旋转) + 4(世界速度) + 12*3(关节局部旋转) + 12*3(关节局部角速度) = 77
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

        // 检查 Behavior Type：只有在训练模式或推理模式下才应用动作
        //if (BehaviorType == BehaviorType.HeuristicOnly || BehaviorType == BehaviorType.InferenceOnly)
        //{
        //    // Heuristic 或 Inference 模式下不应用 AI 动作
        //    return;
        //}

        Debug.Log($"[RobotBalanceAgent] OnActionReceived 被调用: Step={StepCount}");

        if (targetType == TargetType.None)
            return;

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

        //一共26轴
    }

    /// <summary>
    /// 应用力矩到单个关节（AI输出[-1,1]范围，映射到[0,1]后再映射到关节角度）
    /// </summary>
    private void ApplyForceToJoint(ArticulationBody joint, float xForce, float yForce, float zForce)
    {
        if (joint == null) return;

        int index = System.Array.IndexOf(allJoints, joint);
        if (index < 0) return;

        // 获取当前 drive 目标
        var xDrive = joint.xDrive;
        var yDrive = joint.yDrive;
        var zDrive = joint.zDrive;

        xDrive.target = xForce * 180;
        joint.xDrive = xDrive;

        if (joint.jointType == ArticulationJointType.SphericalJoint)
        {
            yDrive.target = yForce * 180;
            joint.yDrive = yDrive;

            zDrive.target = zForce * 180;
            joint.zDrive = zDrive;
        }
        // 将AI输出[-1,1]转换为[0,1]范围，再映射到关节角度范围内
        //float xNormalized = (xForce + 1f) / 2f;
        //xDrive.target = Mathf.LerpAngle(xDrive.lowerLimit, xDrive.upperLimit, xNormalized);
        //joint.xDrive = xDrive;

        //if (joint.jointType == ArticulationJointType.SphericalJoint)
        //{
        //    float yNormalized = (yForce + 1f) / 2f;
        //    yDrive.target = Mathf.LerpAngle(yDrive.lowerLimit, yDrive.upperLimit, yNormalized);
        //    joint.yDrive = yDrive;

        //    float zNormalized = (zForce + 1f) / 2f;
        //    zDrive.target = Mathf.LerpAngle(zDrive.lowerLimit, zDrive.upperLimit, zNormalized);
        //    joint.zDrive = zDrive;
        //}
    }

    /// <summary>
    /// 计算并应用奖励
    /// </summary>
    private void CalculateAndApplyReward()
    {
        if (robotRoot == null) return;

        switch (targetType)
        {
            case TargetType.None:
                break;
            case TargetType.BalancePoint:
                Ballance();
                break;
            default:
                break;
        }
    }

    public void Ballance()
    {
        // 获取臀部位置
        Vector3 pelvisPosition = Vector3.zero;
        if (pelvisIndex >= 0 && allJoints[pelvisIndex] != null)
        {
            pelvisPosition = allJoints[pelvisIndex].transform.position;
        }
        else if (robotRoot != null)
        {
            pelvisPosition = robotRoot.transform.position;
        }

        currentHeight = pelvisPosition.y;

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
            Vector3 worldUp = bodyTransform.rotation * Vector3.up;
            currentTiltAngle = Vector3.Angle(worldUp, Vector3.up);

            if (StepCount % 100 == 0)
            {
                Debug.Log($"[RobotBalanceAgent] {bodyTransform.name} - localRotation: {bodyTransform.localRotation.eulerAngles:F1}, worldUp angle: {currentTiltAngle:F1}°");
            }
        }
        else
        {
            currentTiltAngle = Vector3.Angle(robotRoot.transform.up, Vector3.up);
        }

        // 1. 距离目标点的奖励（臀部越靠近目标点分数越高）
        float distanceReward = 0f;
        if (target != null)
        {
            // 计算臀部到目标点的水平距离（只考虑x和z方向）
            Vector3 pelvisXZ = new Vector3(pelvisPosition.x, 0, pelvisPosition.z);
            Vector3 targetXZ = new Vector3(target.position.x, 0, target.position.z);
            float horizontalDistance = Vector3.Distance(pelvisXZ, targetXZ);

            // 距离越小，奖励越高（使用指数衰减）
            distanceReward = Mathf.Exp(-horizontalDistance) * heightRewardMultiplier;

            // 高度方向的奖励（接近目标高度）
            float heightDistance = Mathf.Abs(pelvisPosition.y - target.position.y);
            float heightReward = Mathf.Exp(-heightDistance) * heightRewardMultiplier;

            // 综合距离奖励
            distanceReward = (distanceReward + heightReward) * 0.5f;
        }

        // 2. 倾斜惩罚
        float tiltPenalty = (currentTiltAngle / maxTiltAngle) * tiltPenaltyMultiplier;

        // 3. 距离变化奖励（向目标移动）
        float approachReward = 0f;
        if (target != null)
        {
            Vector3 pelvisXZ = new Vector3(pelvisPosition.x, 0, pelvisPosition.z);
            Vector3 targetXZ = new Vector3(target.position.x, 0, target.position.z);
            float currentDistance = Vector3.Distance(pelvisXZ, targetXZ);

            // 比较前一帧的位置（使用保存的startPosition作为参考）
            Vector3 prevXZ = new Vector3(startPosition.x, 0, startPosition.z);
            float prevDistance = Vector3.Distance(prevXZ, targetXZ);

            // 向目标移动给予奖励
            if (currentDistance < prevDistance)
            {
                approachReward = (prevDistance - currentDistance) * heightRewardMultiplier;
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
        float totalReward = survivalReward + distanceReward + approachReward
                          - tiltPenalty - movementPenalty;

        AddReward(totalReward);

        if (showDebugInfo)
        {
            Debug.Log($"Reward: {totalReward:F4} | Distance: {distanceReward:F4} | Approach: {approachReward:F4} | Tilt: {-tiltPenalty:F4} | Movement: {-movementPenalty:F4}");
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

        // 渐进式惩罚机制：当接近失败条件时，逐渐增加惩罚
        // 给机器人时间纠正自己

        // 渐进式惩罚1：倾斜过度警告
        if (!isFailed && currentTiltAngle > maxTiltAngle)
        {
            // 超过最大倾角，开始计时惩罚
            if (failTimer == 0f)
            {
                Debug.Log($"[RobotBalanceAgent] 警告: 倾斜角度 {currentTiltAngle:F1}° > {maxTiltAngle}°，开始计时 {failDelayTime} 秒");
            }

            failTimer += Time.fixedDeltaTime;
            // 每帧给予渐进惩罚
            float progressivePenalty = -0.01f * (failTimer / failDelayTime);
            AddReward(progressivePenalty);

            if (showDebugInfo && StepCount % 50 == 0)
            {
                Debug.Log($"[RobotBalanceAgent] 倾斜惩罚中: {failTimer:F1}s / {failDelayTime}s, 累计惩罚: {progressivePenalty:F4}");
            }

            // 如果持续超时则真正失败
            if (failTimer >= failDelayTime)
            {
                Debug.Log($"[RobotBalanceAgent] 失败: 倾斜角度持续 {failDelayTime} 秒无法纠正");
                AddReward(-1f);
                isFailed = true;
                EndEpisode();
                return;
            }
        }
        else
        {
            // 如果倾角恢复正常，重置计时器
            if (failTimer > 0f)
            {
                failTimer = 0f;
            }
        }

        // 渐进式惩罚2：高度过低警告
        if (!isFailed && currentHeight < targetHeight * 0.5f)
        {
            if (failTimer == 0f)
            {
                Debug.Log($"[RobotBalanceAgent] 警告: 高度 {currentHeight:F2}m < {targetHeight * 0.5f:F2}m，开始计时 {failDelayTime} 秒");
            }

            failTimer += Time.fixedDeltaTime;
            float progressivePenalty = -0.01f * (failTimer / failDelayTime);
            AddReward(progressivePenalty);

            if (failTimer >= failDelayTime)
            {
                Debug.Log($"[RobotBalanceAgent] 失败: 高度过低持续 {failDelayTime} 秒无法恢复");
                AddReward(-1f);
                isFailed = true;
                EndEpisode();
                return;
            }
        }
        else
        {
            if (failTimer > 0f)
            {
                failTimer = 0f;
            }
        }

        // 渐进式惩罚3：偏离初始位置过远
        float horizontalDistance = Vector2.Distance(
            new Vector2(robotRoot.transform.position.x, robotRoot.transform.position.z),
            new Vector2(startPosition.x, startPosition.z)
        );
        if (!isFailed && horizontalDistance > 0.5f)
        {
            if (failTimer == 0f)
            {
                Debug.Log($"[RobotBalanceAgent] 警告: 偏离位置 {horizontalDistance:F2}m > 0.5m，开始计时 {failDelayTime} 秒");
            }

            failTimer += Time.fixedDeltaTime;
            float progressivePenalty = -0.005f * (failTimer / failDelayTime);
            AddReward(progressivePenalty);

            if (failTimer >= failDelayTime)
            {
                Debug.Log($"[RobotBalanceAgent] 失败: 偏离位置持续 {failDelayTime} 秒");
                AddReward(-0.5f);
                isFailed = true;
                EndEpisode();
                return;
            }
        }
        else
        {
            if (failTimer > 0f)
            {
                failTimer = 0f;
            }
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
