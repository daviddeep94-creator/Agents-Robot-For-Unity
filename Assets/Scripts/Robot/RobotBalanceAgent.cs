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
    Move,
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

    [Tooltip("机器人目标距离误差")]
    [SerializeField] private float targetDisDiff = 1;
    [Tooltip("机器人速度影响")]
    [SerializeField] private float velocityImpact = 0.1f;
    [Tooltip("最大允许的倾斜角度（度）")]
    [SerializeField] private float maxTiltAngle = 30f;

    [Tooltip("锁定身体上半部分")]
    [SerializeField] private bool lockBodyUp = true;
    // ===== 防抖相关 =====
    private float[] lastActions;
    private float actionSmoothPenalty = 0.2f;     // 动作变化惩罚
    private float angularVelocityPenalty = 0.1f;  // 整体角速度惩罚
    private float jointVelocityPenalty = 0.005f;   // 关节速度惩罚

    // ===== 重置相关 =====
    [Header("重置相关")]
    [Tooltip("重置时给一个随机倾斜角度")]
    [SerializeField] private float rangeTilt = 5;

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = true;

    [Header("关闭重置")]
    [SerializeField] private bool closeReset = false;

    [Header("目标")]
    [SerializeField] private Transform target;
    Vector3 targetOriginPos;
    [Header("目标类型")]
    [SerializeField] private TargetType targetType = TargetType.BalancePoint;
    // 状态变量
    private float episodeTimer;
    private Vector3 targetDri;
    private float targetDis;
    private float currentTiltAngle;

    private ArticulationBody[] allJoints;

    // 关节索引映射
    private int pelvisIndex, torsoIndex;
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
        //if (!this.enabled)
        //{
        //    this.enabled = true;
        //    Debug.LogWarning("[RobotBalanceAgent] 组件被禁用，已自动启用");
        //}
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
        targetOriginPos = target.position;
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

        Debug.Log($"[RobotBalanceAgent] 关节索引: Torso={torsoIndex}, L_Hip={leftHipIndex}, R_Hip={rightHipIndex}");
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
        onTargetTime = 0;
        // 初始化 action buffer
        int actionSize = GetComponent<BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions;
        lastActions = new float[actionSize];

        // 重置机器人姿态到初始状态
        ResetRobotPose();

        if (targetType == TargetType.BalancePoint)
        {
            allJoints[pelvisIndex].TeleportRoot(allJoints[pelvisIndex].transform.position, Quaternion.Euler(Random.Range(-1f, 1f) * rangeTilt, 0, Random.Range(-1f, 1f) * rangeTilt));
        }

        if (targetType == TargetType.Move)
        {
            target.position = new Vector3(Random.Range(-10, 10), target.position.y, Random.Range(-10, 10));
        }
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
    Vector3 leftFeetPoint, rightFeetPoint;
    bool pelvisOnGround, torsoOnGround;
    public void BodyHit(string name, Collision collision)
    {
        Debug.Log($"[RobotBalanceAgent] BodyHit: {name}");
        switch (name)
        {
            case "Left_AnkleJoint":
                leftFootOnGround = true;
                leftFeetPoint = Vector3.zero;
                int num = 0;
                foreach (var item in collision.contacts)
                {
                    leftFeetPoint += item.point;
                    num++;
                }
                leftFeetPoint /= num;
                break;
            case "Right_AnkleJoint":
                rightFootOnGround = true;
                rightFeetPoint = Vector3.zero;
                num = 0;
                foreach (var item in collision.contacts)
                {
                    rightFeetPoint += item.point;
                    num++;
                }
                rightFeetPoint /= num;
                break;
            case "Pelvis_Joint":
                pelvisOnGround = true;
                break;
            case "Torso_Joint":
                torsoOnGround = true;
                break;
            default:
                break;
        }
    }
    public void ClearHit()
    {
        leftFootOnGround = false;
        rightFootOnGround = false;
        pelvisOnGround = false;
        torsoOnGround = false;
    }
    Vector3 worldCenterOfMass;
    Vector3 worldCenterOfMassVelocity;
    float bodyUpAngle;
    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotRoot == null || allJoints == null || allJoints.Length == 0)
        {
            Debug.LogError($"[RobotBalanceAgent] CollectObservations: 机器人或关节未初始化 - robotRoot={robotRoot != null}, allJoints={allJoints?.Length}");
            return;
        }

        int dimensionality = 0;
        ArticulationBody body = allJoints[pelvisIndex];
        // 计算机器人整体重心和整体速度
        worldCenterOfMass = Vector3.zero;
        worldCenterOfMassVelocity = Vector3.zero;
        float totalMass = 0f;

        Vector3 supportPoint = body.transform.position;

        foreach (var joint in allJoints)
        {
            if (joint != null)
            {
                float mass = joint.mass;
                worldCenterOfMass += joint.worldCenterOfMass * mass;
                worldCenterOfMassVelocity += joint.velocity * mass;
                totalMass += mass;
            }
        }

        if (totalMass > 0)
        {
            worldCenterOfMass /= totalMass;
            worldCenterOfMassVelocity /= totalMass;
        }
        Debug.DrawLine(supportPoint, worldCenterOfMass, Color.red);


        // 1. 机器人整体重心
        sensor.AddObservation(body.transform.InverseTransformPoint(worldCenterOfMass));
        dimensionality += 3;

        // 2. 机器人整体速度
        sensor.AddObservation(body.transform.InverseTransformVector(worldCenterOfMassVelocity));
        dimensionality += 3;

        // 3. 机器人根节点世界旋转
        sensor.AddObservation(body.transform.rotation);
        dimensionality += 4;

        // 4. 机器人根节点角速度
        sensor.AddObservation(body.transform.InverseTransformVector(body.angularVelocity));
        dimensionality += 3;

        Vector3 leftFeet = leftFeetPoint == Vector3.zero ? allJoints[leftAnkleIndex].transform.position : leftFeetPoint;
        Vector3 rightFeet = rightFeetPoint == Vector3.zero ? allJoints[rightAnkleIndex].transform.position : rightFeetPoint;
        sensor.AddObservation(body.transform.InverseTransformPoint(leftFeet));
        sensor.AddObservation(body.transform.InverseTransformPoint(rightFeet));
        dimensionality += 6;
        // 6. 各关节局部旋转
        foreach (var joint in allJoints)
        {
            if (joint == body) continue;
            if (joint != null && joint.jointType != ArticulationJointType.FixedJoint)
            {
                if (joint.jointType == ArticulationJointType.RevoluteJoint)
                {
                    sensor.AddObservation(NormalizeAngle(joint.transform.localRotation.eulerAngles.x));
                    dimensionality += 1;
                }
                else if (joint.jointType == ArticulationJointType.SphericalJoint)
                {
                    if (joint.xDrive.lowerLimit != 0 || joint.xDrive.upperLimit != 0)
                    {
                        sensor.AddObservation(NormalizeAngle(joint.transform.localRotation.eulerAngles.x));
                        dimensionality += 1;
                    }
                    if (joint.yDrive.lowerLimit != 0 || joint.yDrive.upperLimit != 0)
                    {
                        sensor.AddObservation(NormalizeAngle(joint.transform.localRotation.eulerAngles.y));
                        dimensionality += 1;
                    }
                    if (joint.zDrive.lowerLimit != 0 || joint.zDrive.upperLimit != 0)
                    {
                        sensor.AddObservation(NormalizeAngle(joint.transform.localRotation.eulerAngles.z));
                        dimensionality += 1;
                    }
                }

                sensor.AddObservation(joint.transform.parent.InverseTransformVector(joint.angularVelocity));
                dimensionality += 3;
            }
        }

        sensor.AddObservation(leftFootOnGround);
        dimensionality += 1;
        sensor.AddObservation(rightFootOnGround);
        dimensionality += 1;

        sensor.AddObservation((int)targetType);
        sensor.AddObservation(body.transform.InverseTransformPoint(target.transform.position) * 0.1f);
        dimensionality += 4;
#if UNITY_EDITOR
        Debug.Log($"总维度:{dimensionality}");
#endif
    }

    public static float NormalizeAngle(float angle)
    {
        return NormalizeAngle180(angle) / 180;// 先映射到 [-1,1)
    }
    /// <summary>
    /// 将角度归一化到 [-180, 180) 区间
    /// </summary>
    public static float NormalizeAngle180(float angle)
    {
        angle = NormalizeAngle360(angle); // 先映射到 [0,360)
        if (angle >= 180f) angle -= 360f;
        return angle;
    }
    /// <summary>
    /// 将角度归一化到 [0, 360) 区间
    /// </summary>
    public static float NormalizeAngle360(float angle)
    {
        angle %= 360f;
        if (angle < 0) angle += 360f;
        return angle;
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
            // 身体（2轴）
            if (torsoIndex >= 0)
            {
                ApplyForceToJoint(allJoints[torsoIndex], actions[index++], 0, actions[index++]);
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
        //一共22轴
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
        Debug.Log("防抖扣分" + -jointVelSum * jointVelocityPenalty);
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
            case TargetType.Move:
                Ballance();
                Move();
                break;
            default:
                break;
        }
    }
    float onTargetTime;

    public void Ballance()
    {
        // ===== 1. 获取核心状态 =====
        ArticulationBody pelvisPos = allJoints[pelvisIndex];

        Vector3 leftFeet = leftFeetPoint == Vector3.zero ? allJoints[leftAnkleIndex].transform.position : leftFeetPoint;
        Vector3 rightFeet = rightFeetPoint == Vector3.zero ? allJoints[rightAnkleIndex].transform.position : rightFeetPoint;
        //平衡核心计算
        float reward;
        Vector2 support1 = new Vector2(leftFeet.x, leftFeet.z);
        Vector2 support2 = new Vector2(rightFeet.x, rightFeet.z);
        Vector2 center = new Vector2(worldCenterOfMass.x, worldCenterOfMass.z);
        Vector2 velocity = new Vector2(pelvisPos.velocity.x, pelvisPos.velocity.z) * Time.fixedDeltaTime * velocityImpact;
        float centerToSupportDistance = PointToSegmentDistance(center + velocity, support1, support2);
        //重心离中心点越远越扣分
        reward = -centerToSupportDistance;
        Debug.Log("重心距离得分" + reward);

        //身体倾斜角度
        float bodyUpright = Vector3.Angle(pelvisPos.transform.up, Vector3.up);
        reward -= bodyUpright * 0.1f;
        if (bodyUpright < 10)
        {
            reward += (1 - (bodyUpright / 10)) * 0.1f;
        }
        // =====  高度奖励（防止蹲着）=====
        targetDri = target.position - pelvisPos.transform.position;
        targetDis = targetDri.magnitude;
        float heightDiff = Mathf.Abs(targetDri.y) / 0.3f;
        reward += -heightDiff * 0.01f;

        // =====  Ground 接触（关键新增）=====
        // 只允许脚接触地面
        if (pelvisOnGround || torsoOnGround)
        {
            reward += -0.1f; // 强惩罚（倒地）
        }

        // 禁止双脚离地（稳定）
        if (!leftFootOnGround && !rightFootOnGround)
        {
            reward -= 0.4f;
            Debug.Log("双脚离地" + reward);
        }

        Debug.Log("平衡得分" + reward);
        AddReward(reward);
        //达到训练完成的目标
        if (targetDis < 0.05f)
        {
            onTargetTime += Time.fixedDeltaTime;
        }
        else
        {
            onTargetTime = 0;
        }

        if (targetType == TargetType.BalancePoint)
        {
            if (onTargetTime > 20)
            {
                targetType = TargetType.Move;
                onTargetTime = 0;
            }
        }
    }

    public void Move()
    {
        // ===== 1. 获取核心状态 =====
        Transform pelvisPos = allJoints[pelvisIndex].transform;
        if (targetDri.magnitude > 0.5)
        {
            //面向目标加分，否则扣分
            float Reward = Vector3.Dot(targetDri.normalized, pelvisPos.forward);
            AddReward(Reward * 0.02f);
        }
    }

    /// <summary>
    /// 检查终止条件
    /// </summary>
    private void CheckTermination()
    {
        if(closeReset) return;
        if (robotRoot == null) return;

        // 倒地（核心终止）
        if (pelvisOnGround || torsoOnGround)
        {
            AddReward(-1f);
            EndEpisode();
            //Debug.Log($"中止，倒地");
            return;
        }
        if (targetType == TargetType.BalancePoint)
        {
            // 倾斜过大
            if (currentTiltAngle > maxTiltAngle)
            {
                AddReward(-0.5f);
                EndEpisode();
                //Debug.Log($"中止，倾斜过大{currentTiltAngle}");
                return;
            }

            // 高度误差过大
            if (Mathf.Abs(targetDri.y) > targetHeightDiff)
            {
                AddReward(-0.5f);
                EndEpisode();
                //Debug.Log($"中止，高度{currentHeight}超过阈值{targetHeightDiff}");
                return;
            }

            // 距离误差过大
            if (Mathf.Max(Mathf.Abs(targetDri.x), Mathf.Abs(targetDri.z)) > targetDisDiff)
            {
                AddReward(-0.5f);
                EndEpisode();
                //Debug.Log($"中止，高度{currentHeight}超过阈值{targetHeightDiff}");
                return;
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

        int index = 0;

        // 身体（2轴）- index 0-1
        if (!lockBodyUp && torsoIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.Alpha1) ? 1f : (Input.GetKey(KeyCode.Alpha2) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.Alpha3) ? 1f : (Input.GetKey(KeyCode.Alpha4) ? -1f : 0f);
        }

        // 左髋（3轴）- index 2-4
        if (leftHipIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.R) ? 1f : (Input.GetKey(KeyCode.F) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.T) ? 1f : (Input.GetKey(KeyCode.G) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.Y) ? 1f : (Input.GetKey(KeyCode.H) ? -1f : 0f);
        }

        // 左膝（1轴）- index 5
        if (leftKneeIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.U) ? 1f : (Input.GetKey(KeyCode.J) ? -1f : 0f);
        }

        // 左踝（1轴）- index 6
        if (leftAnkleIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.I) ? 1f : (Input.GetKey(KeyCode.K) ? -1f : 0f);
        }

        // 右髋（3轴）- index 7-9
        if (rightHipIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.O) ? 1f : (Input.GetKey(KeyCode.L) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.P) ? 1f : (Input.GetKey(KeyCode.Semicolon) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.LeftBracket) ? 1f : (Input.GetKey(KeyCode.RightBracket) ? -1f : 0f);
        }

        // 右膝（1轴）- index 10
        if (rightKneeIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.Z) ? 1f : (Input.GetKey(KeyCode.X) ? -1f : 0f);
        }

        // 右踝（1轴）- index 11
        if (rightAnkleIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.C) ? 1f : (Input.GetKey(KeyCode.V) ? -1f : 0f);
        }

        // 左肩（3轴）- index 12-14
        if (leftShoulderIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.B) ? 1f : (Input.GetKey(KeyCode.N) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.M) ? 1f : (Input.GetKey(KeyCode.Comma) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.Period) ? 1f : (Input.GetKey(KeyCode.Slash) ? -1f : 0f);
        }

        // 左肘（1轴）- index 15
        if (leftElbowIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.Q) ? 1f : (Input.GetKey(KeyCode.W) ? -1f : 0f);
        }

        // 右肩（3轴）- index 16-18
        if (rightShoulderIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.E) ? 1f : (Input.GetKey(KeyCode.R) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.T) ? 1f : (Input.GetKey(KeyCode.Y) ? -1f : 0f);
            continuousActions[index++] = Input.GetKey(KeyCode.U) ? 1f : (Input.GetKey(KeyCode.I) ? -1f : 0f);
        }

        // 右肘（1轴）- index 19
        if (rightElbowIndex >= 0)
        {
            continuousActions[index++] = Input.GetKey(KeyCode.O) ? 1f : (Input.GetKey(KeyCode.P) ? -1f : 0f);
        }

    }
    /// <summary>
    /// 计算点到线段的最短距离
    /// </summary>
    private float PointToSegmentDistance(Vector2 point, Vector2 line1, Vector2 line2)
    {
        return Vector2.Distance(point, PointToSegmentPoint(point, line1, line2));
    }
    private Vector2 PointToSegmentPoint(Vector2 point, Vector2 line1, Vector2 line2)
    {
        Vector2 AB = line2 - line1;
        Vector2 AP = point - line1;

        float abSqr = Vector2.Dot(AB, AB);

        // 防止线段退化为一个点
        if (abSqr == 0f)
        {
            return  line1;
        }

        float t = Vector2.Dot(AP, AB) / abSqr;

        if (t < 0f)
        {
            // 最近点是 line1
            return  line1;
        }
        else if (t > 1f)
        {
            // 最近点是 line2
            return  line2;
        }
        else
        {
            // 投影点
            return line1 + t * AB;
        }
    }
    /// <summary>
    /// 计算点到线段的最短距离
    /// 高性能版，（避免 sqrt）
    /// </summary>
    private float PointToSegmentDistanceSqr(Vector2 point, Vector2 line1, Vector2 line2)
    {
        Vector2 AB = line2 - line1;
        Vector2 AP = point - line1;

        float abSqr = Vector2.Dot(AB, AB);

        if (abSqr == 0f)
        {
            return (point - line1).sqrMagnitude;
        }

        float t = Vector2.Dot(AP, AB) / abSqr;
        t = Mathf.Clamp01(t);

        Vector2 projection = line1 + t * AB;
        return (point - projection).sqrMagnitude;
    }
    private void OnDrawGizmos()
    {
        if (!showDebugInfo || robotRoot == null) return;

#if UNITY_EDITOR
        Vector3 point1 = leftFeetPoint == Vector3.zero ? allJoints[leftAnkleIndex].transform.position : leftFeetPoint;
        Vector3 point2 = rightFeetPoint == Vector3.zero ? allJoints[rightAnkleIndex].transform.position : rightFeetPoint;
        point1.y = 0;
        point2.y = 0;
        Debug.DrawLine(point1, point2, Color.green);

        Vector3 velocityOfWorld = allJoints[pelvisIndex].velocity * Time.fixedDeltaTime * velocityImpact;
        velocityOfWorld.y = 0;
        Vector3 centerOfWorld = worldCenterOfMass;
        centerOfWorld.y = 0;
        Debug.DrawLine(centerOfWorld, centerOfWorld + velocityOfWorld, Color.blue);

        Vector2 closePointOfSuport2D = PointToSegmentPoint(new Vector2(centerOfWorld.x, centerOfWorld.z) + new Vector2(velocityOfWorld.x, velocityOfWorld.z), new Vector2(point1.x, point1.z), new Vector2(point2.x, point2.z));
        Vector3 closePointOfSuport = new Vector3(closePointOfSuport2D.x, 0, closePointOfSuport2D.y);

        Debug.DrawLine(centerOfWorld + velocityOfWorld, closePointOfSuport, Color.red);
#endif
    }
}
