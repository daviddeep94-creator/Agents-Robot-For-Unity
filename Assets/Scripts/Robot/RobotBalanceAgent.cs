using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;

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

    [Tooltip("机器人重心高度误差")]
    [SerializeField] private float targetHeightDiff = 0.25f;

    [Tooltip("最大允许的倾斜角度（度）")]
    [SerializeField] private float maxTiltAngle = 30f;

    [Tooltip("锁定身体上半部分")]
    [SerializeField] private bool lockBodyUp = true;
    // ===== 防抖相关 =====
    private float[] lastActions;
    private float actionSmoothPenalty = 0.05f;     // 动作变化惩罚
    private float angularVelocityPenalty = 0.02f;  // 整体角速度惩罚
    private float jointVelocityPenalty = 0.001f;   // 关节速度惩罚

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = true;

    [Header("目标")]
    [SerializeField] private Transform target;

    [Header("目标类型")]
    [SerializeField] private TargetType targetType = TargetType.BalancePoint;
    // 状态变量
    private float episodeTimer;
    private float currentDis;
    private float currentTiltAngle;

    private ArticulationBody[] allJoints;

    // 关节索引映射
    private int pelvisIndex, torsoIndex, neckIndex;
    private int leftHipIndex, rightHipIndex;
    private int leftKneeIndex, rightKneeIndex;
    private int leftAnkleIndex, rightAnkleIndex;
    private int leftShoulderIndex, rightShoulderIndex;
    private int leftElbowIndex, rightElbowIndex;

    // 保存的初始关节状态（用于重置）
    private Vector3[] initialJointPositions;
    private Quaternion[] initialJointRotations;
    private float[] initialJointXTargets;
    private float[] initialJointYTargets;
    private float[] initialJointZTargets;

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
        if (!target)
        {
            target = new GameObject("Target").transform;
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
        target.transform.position = allJoints[pelvisIndex].transform.position;
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
        //Debug.Log($"[RobotBalanceAgent] ========== OnEpisodeBegin 被调用 ==========");
        episodeTimer = 0f;
        // 初始化 action buffer
        int actionSize = GetComponent<BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions;
        lastActions = new float[actionSize];
        // 重置机器人姿态到初始状态
        ResetRobotPose();
    }

    /// <summary>
    /// 重置机器人姿态到初始状态
    /// </summary>
    private void ResetRobotPose()
    {
        RobotResetUtility.ResetRobotPose(
            robotRoot,
            initialJointPositions,
            initialJointRotations,
            initialJointXTargets,
            initialJointYTargets,
            initialJointZTargets,
            "[RobotBalanceAgent] ResetRobotPose:"
        );
    }

    bool leftFootOnGround, rightFootOnGround;
    bool pelvisOnGround, torsoOnGround;
    public void BodyHit(string name)
    {
        leftFootOnGround = name == "Left_Foot_Visual";
        rightFootOnGround = name == "Right_Foot_Visual";
        pelvisOnGround = name == "Pelvis_Visual";
        torsoOnGround = name == "Torso_Visual";
    }
    public void ClearHit()
    {
        leftFootOnGround = false;
        rightFootOnGround = false;
        pelvisOnGround = false;
        torsoOnGround = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotRoot == null || allJoints == null || allJoints.Length == 0)
        {
            Debug.LogError($"[RobotBalanceAgent] CollectObservations: 机器人或关节未初始化 - robotRoot={robotRoot != null}, allJoints={allJoints?.Length}");
            return;
        }

        int Ground = 0;
        if (leftFootOnGround)
        {
            Ground += 1;
        }
        if (rightFootOnGround)
        {
            Ground += 2;
        }
        if (pelvisOnGround)
        {
            Ground += 4;
        }
        if(torsoOnGround)
        {
            Ground += 8;
        }

        int dimensionality = 0;
        ArticulationBody body = allJoints[pelvisIndex];
        // 计算机器人整体重心和整体速度
        Vector3 centerOfMass = Vector3.zero;
        Vector3 centerOfMassVelocity = Vector3.zero;
        float totalMass = 0f;

        foreach (var joint in allJoints)
        {
            if (joint != null)
            {
                float mass = joint.mass;
                centerOfMass += (joint.worldCenterOfMass-body.transform.position) * mass;
                centerOfMassVelocity += joint.velocity * mass;
                totalMass += mass;
            }
        }

        if (totalMass > 0)
        {
            centerOfMass /= totalMass;
            centerOfMassVelocity /= totalMass;
        }
        
        if(leftFootOnGround && rightFootOnGround)
        {
            centerOfMass = centerOfMass - Vector3.Lerp(allJoints[leftAnkleIndex].transform.position, allJoints[rightAnkleIndex].transform.position, 0.5f);
        }
        else if (leftFootOnGround)
        {
            centerOfMass = centerOfMass - allJoints[leftAnkleIndex].transform.position;
        }
        else if (rightFootOnGround)
        {
            centerOfMass = centerOfMass - allJoints[rightAnkleIndex].transform.position;
        }
        //计算局部重心
        centerOfMass = body.transform.InverseTransformPoint(centerOfMass);
        // 1. 机器人整体重心
        sensor.AddObservation(centerOfMass);
        dimensionality += 3;
        //计算局部速度
        centerOfMassVelocity = body.transform.InverseTransformVector(centerOfMassVelocity);
        // 2. 机器人整体速度
        sensor.AddObservation(centerOfMassVelocity);
        dimensionality += 3;

        // 3. 机器人根节点世界旋转
        sensor.AddObservation(body.transform.rotation.eulerAngles);
        dimensionality += 3;
        // 4. 机器人根节点上向量
        sensor.AddObservation(body.transform.up);
        dimensionality += 3;
        // 5. 踝关节上向量
        sensor.AddObservation(allJoints[leftAnkleIndex].transform.up);
        sensor.AddObservation(allJoints[rightAnkleIndex].transform.up);
        dimensionality += 6;

        sensor.AddObservation(body.transform.InverseTransformPoint(allJoints[leftAnkleIndex].transform.position));
        sensor.AddObservation(body.transform.InverseTransformPoint(allJoints[rightAnkleIndex].transform.position));
        dimensionality += 6;
        // 6. 各关节局部旋转12*3=36
        foreach (var joint in allJoints)
        {
            if (joint == body) continue;
            if (joint != null && joint.jointType != ArticulationJointType.FixedJoint)
            {
                sensor.AddObservation(joint.transform.localRotation.eulerAngles);
                dimensionality += 3;
            }
        }

        sensor.AddObservation(Ground);
        dimensionality += 1;

        sensor.AddObservation((int)targetType);
        sensor.AddObservation(target.position-body.transform.position);
        dimensionality += 4;

        Debug.Log($"总维度:{dimensionality}");
        // 总观测维度: 12(根位置) + 6(踝关节上向量)+ 6(踝关节相对位置) + 12*3(关节局部旋转) + 1(Ground) + 1(targetType) + 3(target)= 65
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (robotRoot == null)
        {
            Debug.LogError("[RobotBalanceAgent] OnActionReceived: robotRoot 为 null！");
            return;
        }

        if (targetType == TargetType.None)
            return;

        // 应用关节力矩（12个连续动作对应12个关节）26d
        ApplyJointForces(actions.ContinuousActions);
        //防抖
        ApplyStabilityPenalty(actions.ContinuousActions);
        // 计算奖励
        CalculateAndApplyReward();

        // 检查终止条件
        CheckTermination();

        ClearHit();

        episodeTimer += Time.fixedDeltaTime;
    }

    /// <summary>
    /// 应用关节力矩
    /// </summary>
    private void ApplyJointForces(ActionSegment<float> actions)
    {
        int index = 0;

        // 左腿
        if (leftHipIndex >= 0)  // 髋关节（3轴）
        {
            ApplyForceToJoint(allJoints[leftHipIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (leftKneeIndex >= 0)  // 膝关节（1轴）
        {
            ApplyForceToJoint(allJoints[leftKneeIndex], actions[index++], 0, 0);
        }
        if (leftAnkleIndex >= 0)  // 踝关节（2轴）
        {
            ApplyForceToJoint(allJoints[leftAnkleIndex], actions[index++], 0, actions[index++]);
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
        if (rightAnkleIndex >= 0)  // 踝关节（2轴）
        {
            ApplyForceToJoint(allJoints[rightAnkleIndex], actions[index++], 0, actions[index++]);
        }

        if(!lockBodyUp)
        {
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
            // 身体（2轴）
            if (torsoIndex >= 0)
            {
                ApplyForceToJoint(allJoints[torsoIndex], actions[index++], 0, actions[index++]);
            }

            // 脖子（2轴）
            if (neckIndex >= 0)
            {
                ApplyForceToJoint(allJoints[neckIndex], actions[index++], actions[index++], 0);
            }
        }
        //一共24轴
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
    }
    private void ApplyStabilityPenalty(ActionSegment<float> actions)
    {
        // ===== 1. 动作变化惩罚（最关键）=====
        float actionDelta = 0f;
        for (int i = 0; i < actions.Length; i++)
        {
            float diff = actions[i] - lastActions[i];
            actionDelta += diff * diff;

            // 保存当前动作
            lastActions[i] = actions[i];
        }
        AddReward(-actionDelta * actionSmoothPenalty);

        // ===== 2. 整体角速度惩罚 =====
        Vector3 angularVel = robotRoot.angularVelocity;
        AddReward(-angularVel.sqrMagnitude * angularVelocityPenalty);

        // ===== 3. 关节速度惩罚 =====
        float jointVelSum = 0f;
        foreach (var joint in allJoints)
        {
            if (joint != null)
            {
                jointVelSum += joint.angularVelocity.sqrMagnitude;
            }
        }
        AddReward(-jointVelSum * jointVelocityPenalty);
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
        // ===== 1. 获取核心状态 =====
        Transform pelvisPos = allJoints[pelvisIndex].transform;

        // Transform body = (torsoIndex >= 0) ? allJoints[torsoIndex].transform : robotRoot.transform;

        currentTiltAngle = Vector3.Angle(pelvisPos.up, Vector3.up); // 0 ~ 180

        // ===== 2. 基础站立奖励（核心）=====
        float upright = Mathf.Clamp01(1f - currentTiltAngle / maxTiltAngle); // 越直越接近1
        AddReward(upright * 0.02f);

        // ===== 3. 目标距离奖励（防止蹲着）=====
        currentDis = (pelvisPos.position - target.transform.position).magnitude;
        float heightReward = Mathf.Exp(-currentDis * 5f);
        AddReward(heightReward * 0.01f);

        // ===== 4. Ground 接触（关键新增）=====
        // 只允许脚接触地面
        if (pelvisOnGround || torsoOnGround)
        {
            AddReward(-0.05f); // 强惩罚（倒地）
        }

        // 鼓励双脚着地（稳定）
        float footReward = 0;
        if (leftFootOnGround) footReward += 0.005f;
        if (rightFootOnGround) footReward += 0.005f;
        footReward += ((Vector3.Dot(allJoints[leftAnkleIndex].transform.up, Vector3.up)) - 0.8f) * 0.04f;
        footReward += ((Vector3.Dot(allJoints[rightAnkleIndex].transform.up, Vector3.up)) - 0.8f) * 0.04f;
        AddReward(footReward);
    }

    /// <summary>
    /// 检查终止条件
    /// </summary>
    private void CheckTermination()
    {
        if (robotRoot == null) return;

        // 倒地（核心终止）
        if (pelvisOnGround || torsoOnGround)
        {
            AddReward(-1f);
            EndEpisode();
            //Debug.Log($"中止，倒地");
            return;
        }

        // 倾斜过大
        if (currentTiltAngle > maxTiltAngle)
        {
            AddReward(-0.5f);
            EndEpisode();
            //Debug.Log($"中止，倾斜过大{currentTiltAngle}");
            return;
        }

        // 高度误差过大
        if (currentDis > targetHeightDiff)
        {
            AddReward(-0.5f);
            EndEpisode();
            //Debug.Log($"中止，高度{currentHeight}超过阈值{targetHeightDiff}");
            return;
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

        int index = 0;

        // 身体（2轴）- index 0-1
        if (!lockBodyUp && torsoIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.Alpha1) ? 1f : (Input.GetKey(KeyCode.Alpha2) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.Alpha3) ? 1f : (Input.GetKey(KeyCode.Alpha4) ? -1f : 0f);
        }

        // 脖子（2轴）- index 2-3
        if (!lockBodyUp && neckIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.W) ? 1f : (Input.GetKey(KeyCode.S) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.A) ? 1f : (Input.GetKey(KeyCode.D) ? -1f : 0f);
        }

        // 左髋（3轴）- index 4-6
        if (leftHipIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.R) ? 1f : (Input.GetKey(KeyCode.F) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.T) ? 1f : (Input.GetKey(KeyCode.G) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.Y) ? 1f : (Input.GetKey(KeyCode.H) ? -1f : 0f);
        }

        // 左膝（1轴）- index 7
        if (leftKneeIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.U) ? 1f : (Input.GetKey(KeyCode.J) ? -1f : 0f);
        }

        // 左踝（1轴）- index 8
        if (leftAnkleIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.I) ? 1f : (Input.GetKey(KeyCode.K) ? -1f : 0f);
        }

        // 右髋（3轴）- index 9-11
        if (rightHipIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.O) ? 1f : (Input.GetKey(KeyCode.L) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.P) ? 1f : (Input.GetKey(KeyCode.Semicolon) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.LeftBracket) ? 1f : (Input.GetKey(KeyCode.RightBracket) ? -1f : 0f);
        }

        // 右膝（1轴）- index 12
        if (rightKneeIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.Z) ? 1f : (Input.GetKey(KeyCode.X) ? -1f : 0f);
        }

        // 右踝（1轴）- index 13
        if (rightAnkleIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.C) ? 1f : (Input.GetKey(KeyCode.V) ? -1f : 0f);
        }

        if (!lockBodyUp)
        {
            // 左肩（3轴）- index 14-16
            if (leftShoulderIndex >= 0)
            {
                continuousActions[index++] = Input.GetKey(KeyCode.B) ? 1f : (Input.GetKey(KeyCode.N) ? -1f : 0f);
                continuousActions[index++] = Input.GetKey(KeyCode.M) ? 1f : (Input.GetKey(KeyCode.Comma) ? -1f : 0f);
                continuousActions[index++] = Input.GetKey(KeyCode.Period) ? 1f : (Input.GetKey(KeyCode.Slash) ? -1f : 0f);
            }

            // 左肘（1轴）- index 17
            if (leftElbowIndex >= 0)
            {
                continuousActions[index++] = Input.GetKey(KeyCode.Q) ? 1f : (Input.GetKey(KeyCode.W) ? -1f : 0f);
            }

            // 右肩（3轴）- index 18-20
            if (rightShoulderIndex >= 0)
            {
                continuousActions[index++] = Input.GetKey(KeyCode.E) ? 1f : (Input.GetKey(KeyCode.R) ? -1f : 0f);
                continuousActions[index++] = Input.GetKey(KeyCode.T) ? 1f : (Input.GetKey(KeyCode.Y) ? -1f : 0f);
                continuousActions[index++] = Input.GetKey(KeyCode.U) ? 1f : (Input.GetKey(KeyCode.I) ? -1f : 0f);
            }

            // 右肘（1轴）- index 21
            if (rightElbowIndex >= 0)
            {
                continuousActions[index++] = Input.GetKey(KeyCode.O) ? 1f : (Input.GetKey(KeyCode.P) ? -1f : 0f);
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (!showDebugInfo || robotRoot == null) return;

        // 绘制目标高度线
        Gizmos.color = Color.green;
        Gizmos.DrawLine(
            new Vector3(robotRoot.transform.position.x, targetHeightDiff, robotRoot.transform.position.z),
            new Vector3(robotRoot.transform.position.x, targetHeightDiff + 0.1f, robotRoot.transform.position.z)
        );

        // 绘制机器人朝向
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(robotRoot.transform.position, robotRoot.transform.up * 0.5f);

        // 绘制理想朝向
        Gizmos.color = Color.red;
        Gizmos.DrawRay(robotRoot.transform.position, Vector3.up * 0.5f);
    }
}
