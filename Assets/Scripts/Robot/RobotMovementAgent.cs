using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// 机器人移动训练Agent
/// 在站立平衡的基础上，训练机器人向目标点移动
/// </summary>
public class RobotMovementAgent : Agent
{
    [Header("机器人配置")]
    [Tooltip("机器人根节点ArticulationBody")]
    [SerializeField] private ArticulationBody robotRoot;

    [Header("目标配置")]
    [Tooltip("目标Transform")]
    [SerializeField] private Transform targetTransform;

    [Tooltip("目标生成范围（米）")]
    [SerializeField] private float targetSpawnRange = 5f;

    [Tooltip("最小目标距离（米）")]
    [SerializeField] private float minTargetDistance = 1f;

    [Tooltip("最大目标距离（米）")]
    [SerializeField] private float maxTargetDistance = 3f;

    [Header("奖励配置")]
    [Tooltip("接近目标的基础奖励")]
    [SerializeField] private float approachReward = 0.01f;

    [Tooltip("到达目标的奖励")]
    [SerializeField] private float reachReward = 2f;

    [Tooltip("保持直立的奖励")]
    [SerializeField] private float uprightReward = 0.005f;

    [Tooltip("偏离直立的惩罚")]
    [SerializeField] private float tiltPenaltyMultiplier = 0.1f;

    [Tooltip("关节移动惩罚")]
    [SerializeField] private float jointMovementPenalty = 0.0005f;

    [Tooltip("速度惩罚（避免过快）")]
    [SerializeField] private float velocityPenaltyMultiplier = 0.01f;

    [Header("动作配置")]
    [Tooltip("关节力矩缩放")]
    [SerializeField] private float jointForceScale = 150f;

    [Header("平衡限制")]
    [Tooltip("最大允许倾斜角度（度）")]
    [SerializeField] private float maxTiltAngle = 25f;

    [Tooltip("最小允许高度（米）")]
    [SerializeField] private float minHeight = 0.6f;

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = false;

    // 状态变量
    private float distanceToTarget;
    private float previousDistanceToTarget;
    private float currentTiltAngle;
    private float currentHeight;
    private ArticulationBody[] allJoints;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float[] previousJointAngles;

    // 关节索引
    private int pelvisIndex, torsoIndex, neckIndex;
    private int leftHipIndex, rightHipIndex;
    private int leftKneeIndex, rightKneeIndex;
    private int leftAnkleIndex, rightAnkleIndex;
    private int leftShoulderIndex, rightShoulderIndex;
    private int leftElbowIndex, rightElbowIndex;

    private void Start()
    {
        InitializeRobot();
    }

    /// <summary>
    /// 初始化机器人
    /// </summary>
    private void InitializeRobot()
    {
        if (robotRoot == null)
        {
            robotRoot = GetComponent<ArticulationBody>();
        }

        if (robotRoot == null)
        {
            Debug.LogError("RobotMovementAgent: 未找到机器人根节点ArticulationBody！");
            return;
        }

        allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();

        // 查找关节索引
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

        previousJointAngles = new float[12];

        // 创建目标（如果不存在）
        if (targetTransform == null)
        {
            CreateTarget();
        }
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
        return -1;
    }

    /// <summary>
    /// 创建目标物体
    /// </summary>
    private void CreateTarget()
    {
        GameObject targetObj = new GameObject("Target");
        targetTransform = targetObj.transform;
        targetTransform.position = Vector3.zero;

        // 添加视觉指示器
        GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        indicator.name = "TargetVisual";
        indicator.transform.SetParent(targetTransform, false);
        indicator.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        indicator.transform.localScale = new Vector3(0.3f, 0.1f, 0.3f);

        Renderer renderer = indicator.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.yellow;
        }

        Collider collider = indicator.GetComponent<Collider>();
        if (collider != null)
        {
            collider.enabled = false;
        }
    }

    public override void OnEpisodeBegin()
    {
        previousDistanceToTarget = float.MaxValue;

        // 记录初始位置
        startPosition = robotRoot.transform.position;
        startRotation = robotRoot.transform.rotation;

        // 重置机器人姿态
        ResetRobotPose();

        // 随机生成新目标位置
        SpawnNewTarget();

        if (showDebugInfo)
        {
            Debug.Log("Episode Start: 机器人重置，新目标已生成");
        }
    }

    /// <summary>
    /// 重置机器人姿态
    /// </summary>
    private void ResetRobotPose()
    {
        // 重置到初始位置（保持高度）
        Vector3 resetPosition = startPosition;
        resetPosition.y = 0.875f; // 保持直立高度
        robotRoot.transform.position = resetPosition;
        robotRoot.transform.rotation = Quaternion.identity;

        // 重置所有关节
        foreach (var joint in allJoints)
        {
            if (joint != null && joint.jointType != ArticulationJointType.FixedJoint)
            {
                var xDrive = joint.xDrive;
                xDrive.target = 0f;
                joint.xDrive = xDrive;

                var yDrive = joint.yDrive;
                yDrive.target = 0f;
                joint.yDrive = yDrive;

                var zDrive = joint.zDrive;
                zDrive.target = 0f;
                joint.zDrive = zDrive;
            }
        }

        // 添加轻微随机扰动
        robotRoot.transform.rotation = Quaternion.Euler(
            Random.Range(-3f, 3f),
            Random.Range(-3f, 3f),
            Random.Range(-3f, 3f)
        );
    }

    /// <summary>
    /// 生成新目标位置
    /// </summary>
    private void SpawnNewTarget()
    {
        // 在机器人前方一定范围内随机生成目标
        float distance = Random.Range(minTargetDistance, maxTargetDistance);
        float angle = Random.Range(-Mathf.PI / 3, Mathf.PI / 3); // ±60度范围

        Vector3 robotForward = robotRoot.transform.forward;
        Vector3 targetOffset = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0) * robotForward * distance;

        Vector3 targetPosition = robotRoot.transform.position + targetOffset;
        targetPosition.y = robotRoot.transform.position.y; // 保持同一高度

        targetTransform.position = targetPosition;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotRoot == null || targetTransform == null) return;

        // 1. 相对目标位置（归一化）
        Vector3 toTarget = targetTransform.position - robotRoot.transform.position;
        sensor.AddObservation(toTarget.normalized.x);
        sensor.AddObservation(toTarget.normalized.y);
        sensor.AddObservation(toTarget.normalized.z);

        // 2. 目标距离（归一化）
        float normalizedDistance = Mathf.Clamp(distanceToTarget / maxTargetDistance, 0f, 1f);
        sensor.AddObservation(normalizedDistance);

        // 3. 机器人旋转
        Vector3 euler = robotRoot.transform.rotation.eulerAngles;
        sensor.AddObservation(NormalizeAngle(euler.x));
        sensor.AddObservation(NormalizeAngle(euler.y));
        sensor.AddObservation(NormalizeAngle(euler.z));

        // 4. 机器人速度（归一化）
        Vector3 velocity = robotRoot.velocity;
        sensor.AddObservation(Mathf.Clamp(velocity.x / 2f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(velocity.y / 2f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(velocity.z / 2f, -1f, 1f));

        // 5. 机器人角速度（归一化）
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.x / 5f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.y / 5f, -1f, 1f));
        sensor.AddObservation(Mathf.Clamp(robotRoot.angularVelocity.z / 5f, -1f, 1f));

        // 6. 关节角度（归一化）
        float[] jointAngles = GetJointAngles();
        foreach (float angle in jointAngles)
        {
            sensor.AddObservation(NormalizeAngle(angle));
        }

        // 7. 关节速度（归一化）
        float[] jointVelocities = GetJointVelocities();
        foreach (float jointVel in jointVelocities)
        {
            sensor.AddObservation(Mathf.Clamp(jointVel / 5f, -1f, 1f));
        }

        // 8. 重力方向（局部坐标）
        Vector3 localGravity = robotRoot.transform.InverseTransformDirection(Physics.gravity);
        sensor.AddObservation(localGravity.normalized);

        // 总观测维度: 3 + 1 + 3 + 3 + 3 + 12 + 12 + 3 = 40
    }

    /// <summary>
    /// 归一化角度
    /// </summary>
    private float NormalizeAngle(float angle)
    {
        angle = (angle + 180f) % 360f - 180f;
        return Mathf.Clamp(angle / 180f, -1f, 1f);
    }

    /// <summary>
    /// 获取关节角度
    /// </summary>
    private float[] GetJointAngles()
    {
        float[] angles = new float[12];
        int index = 0;

        if (torsoIndex >= 0) angles[index++] = allJoints[torsoIndex].xDrive.target;
        if (neckIndex >= 0) angles[index++] = allJoints[neckIndex].xDrive.target;
        if (leftHipIndex >= 0) angles[index++] = allJoints[leftHipIndex].xDrive.target;
        if (leftKneeIndex >= 0) angles[index++] = allJoints[leftKneeIndex].xDrive.target;
        if (leftAnkleIndex >= 0) angles[index++] = allJoints[leftAnkleIndex].xDrive.target;
        if (rightHipIndex >= 0) angles[index++] = allJoints[rightHipIndex].xDrive.target;
        if (rightKneeIndex >= 0) angles[index++] = allJoints[rightKneeIndex].xDrive.target;
        if (rightAnkleIndex >= 0) angles[index++] = allJoints[rightAnkleIndex].xDrive.target;
        if (leftShoulderIndex >= 0) angles[index++] = allJoints[leftShoulderIndex].xDrive.target;
        if (leftElbowIndex >= 0) angles[index++] = allJoints[leftElbowIndex].xDrive.target;
        if (rightShoulderIndex >= 0) angles[index++] = allJoints[rightShoulderIndex].xDrive.target;
        if (rightElbowIndex >= 0) angles[index++] = allJoints[rightElbowIndex].xDrive.target;

        return angles;
    }

    /// <summary>
    /// 获取关节速度
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
        if (robotRoot == null) return;

        // 应用关节力矩
        ApplyJointForces(actions.ContinuousActions);

        // 计算奖励
        CalculateAndApplyReward();

        // 检查终止条件
        CheckTermination();
    }

    /// <summary>
    /// 应用关节力矩
    /// </summary>
    private void ApplyJointForces(ActionSegment<float> actions)
    {
        int index = 0;

        // 躯干和颈部
        if (torsoIndex >= 0)
        {
            ApplyForceToJoint(allJoints[torsoIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (neckIndex >= 0)
        {
            ApplyForceToJoint(allJoints[neckIndex], actions[index++], 0, 0);
        }

        // 左腿（3轴控制）
        if (leftHipIndex >= 0)
        {
            ApplyForceToJoint(allJoints[leftHipIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (leftKneeIndex >= 0)
        {
            ApplyForceToJoint(allJoints[leftKneeIndex], actions[index++], 0, 0);
        }
        if (leftAnkleIndex >= 0)
        {
            ApplyForceToJoint(allJoints[leftAnkleIndex], actions[index++], actions[index++], actions[index++]);
        }

        // 右腿（3轴控制）
        if (rightHipIndex >= 0)
        {
            ApplyForceToJoint(allJoints[rightHipIndex], actions[index++], actions[index++], actions[index++]);
        }
        if (rightKneeIndex >= 0)
        {
            ApplyForceToJoint(allJoints[rightKneeIndex], actions[index++], 0, 0);
        }

        // 右踝关节
        if (rightAnkleIndex >= 0)
        {
            ApplyForceToJoint(allJoints[rightAnkleIndex], actions[index++], actions[index++], actions[index++]);
        }

        // 手臂（简化控制）
        if (leftShoulderIndex >= 0)
        {
            ApplyForceToJoint(allJoints[leftShoulderIndex], actions[index++], actions[index++], 0);
        }
        if (rightShoulderIndex >= 0)
        {
            ApplyForceToJoint(allJoints[rightShoulderIndex], actions[index++], actions[index++], 0);
        }
    }

    /// <summary>
    /// 应用力矩到关节
    /// </summary>
    private void ApplyForceToJoint(ArticulationBody joint, float xForce, float yForce, float zForce)
    {
        if (joint == null) return;

        var xDrive = joint.xDrive;
        xDrive.target += xForce * jointForceScale * Time.fixedDeltaTime;
        joint.xDrive = xDrive;

        if (joint.jointType == ArticulationJointType.SphericalJoint)
        {
            var yDrive = joint.yDrive;
            yDrive.target += yForce * jointForceScale * Time.fixedDeltaTime;
            joint.yDrive = yDrive;

            var zDrive = joint.zDrive;
            zDrive.target += zForce * jointForceScale * Time.fixedDeltaTime;
            joint.zDrive = zDrive;
        }
    }

    /// <summary>
    /// 计算奖励
    /// </summary>
    private void CalculateAndApplyReward()
    {
        if (robotRoot == null || targetTransform == null) return;

        // 计算距离
        distanceToTarget = Vector3.Distance(robotRoot.transform.position, targetTransform.position);

        // 计算倾斜角度
        currentTiltAngle = Vector3.Angle(robotRoot.transform.up, Vector3.up);

        // 计算高度
        currentHeight = robotRoot.transform.position.y;

        // 1. 接近奖励
        float distanceReward = 0f;
        if (previousDistanceToTarget < float.MaxValue)
        {
            float distanceDelta = previousDistanceToTarget - distanceToTarget;
            distanceReward = distanceDelta * approachReward;
        }
        previousDistanceToTarget = distanceToTarget;

        // 2. 直立奖励
        float uprightFactor = 1f - (currentTiltAngle / maxTiltAngle);
        float uprightRewardVal = uprightFactor * uprightReward;

        // 3. 倾斜惩罚
        float tiltPenalty = (currentTiltAngle / maxTiltAngle) * tiltPenaltyMultiplier;

        // 4. 速度惩罚（鼓励稳定移动）
        float velocity = robotRoot.velocity.magnitude;
        float velocityPenalty = velocity * velocityPenaltyMultiplier;

        // 5. 关节移动惩罚
        float movementPenalty = 0f;
        float[] currentJointAngles = GetJointAngles();
        for (int i = 0; i < Mathf.Min(previousJointAngles.Length, currentJointAngles.Length); i++)
        {
            movementPenalty += Mathf.Abs(currentJointAngles[i] - previousJointAngles[i]);
        }
        movementPenalty *= jointMovementPenalty;
        previousJointAngles = currentJointAngles;

        // 总奖励
        float totalReward = distanceReward + uprightRewardVal - tiltPenalty
                          - velocityPenalty - movementPenalty;

        AddReward(totalReward);

        if (showDebugInfo)
        {
            Debug.Log($"Reward: {totalReward:F4} | Dist: {distanceReward:F4} | Upright: {uprightRewardVal:F4} | Tilt: {-tiltPenalty:F4}");
        }
    }

    /// <summary>
    /// 检查终止条件
    /// </summary>
    private void CheckTermination()
    {
        if (robotRoot == null || targetTransform == null) return;

        // 成功：到达目标
        if (distanceToTarget < 0.2f)
        {
            AddReward(reachReward);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log("Episode Success: 到达目标！");
            }
            return;
        }

        // 失败：倾斜过度
        if (currentTiltAngle > maxTiltAngle)
        {
            AddReward(-1f);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 倾斜 {currentTiltAngle:F1}°");
            }
            return;
        }

        // 失败：高度过低
        if (currentHeight < minHeight)
        {
            AddReward(-1f);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 高度过低 {currentHeight:F2}m");
            }
            return;
        }

        // 失败：偏离初始位置过远
        float horizontalDistance = Vector2.Distance(
            new Vector2(robotRoot.transform.position.x, robotRoot.transform.position.z),
            new Vector2(startPosition.x, startPosition.z)
        );
        if (horizontalDistance > 10f)
        {
            AddReward(-0.5f);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log($"Episode Failed: 偏离过远 {horizontalDistance:F2}m");
            }
            return;
        }

        // 失败：超时
        if (StepCount >= MaxStep)
        {
            AddReward(-0.5f);
            EndEpisode();
            if (showDebugInfo)
            {
                Debug.Log("Episode Failed: 超时");
            }
        }
    }

    /// <summary>
    /// Heuristic模式
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;

        float forward = Input.GetKey(KeyCode.W) ? 1f : (Input.GetKey(KeyCode.S) ? -1f : 0f);
        float backward = Input.GetKey(KeyCode.S) ? -1f : 0f;
        float left = Input.GetKey(KeyCode.A) ? 1f : 0f;
        float right = Input.GetKey(KeyCode.D) ? 1f : 0f;

        // 躯干
        continuousActions[0] = forward;
        continuousActions[1] = left - right;
        continuousActions[2] = 0f;

        // 颈部
        continuousActions[3] = 0f;

        // 左腿
        continuousActions[4] = forward;
        continuousActions[5] = 0f;
        continuousActions[6] = left - right;

        // 左膝
        continuousActions[7] = forward;

        // 左踝
        continuousActions[8] = -forward * 0.5f;
        continuousActions[9] = 0f;
        continuousActions[10] = (left - right) * 0.5f;

        // 右腿
        continuousActions[11] = forward;
        continuousActions[12] = 0f;
        continuousActions[13] = left - right;

        // 右膝
        continuousActions[14] = forward;

        // 右踝
        continuousActions[15] = -forward * 0.5f;
        continuousActions[16] = 0f;
        continuousActions[17] = (left - right) * 0.5f;

        // 手臂
        continuousActions[18] = forward;
        continuousActions[19] = 0f;
        continuousActions[20] = forward;
        continuousActions[21] = 0f;

        for (int i = 22; i < continuousActions.Length; i++)
        {
            continuousActions[i] = 0f;
        }
    }

    private void OnDrawGizmos()
    {
        if (!showDebugInfo || robotRoot == null) return;

        // 绘制到目标的连线
        if (targetTransform != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(robotRoot.transform.position, targetTransform.position);

            // 绘制目标位置
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(targetTransform.position, 0.2f);
        }

        // 绘制机器人朝向
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(robotRoot.transform.position, robotRoot.transform.forward);
    }
}
