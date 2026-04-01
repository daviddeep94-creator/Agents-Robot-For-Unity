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

    [Header("目标站立时间")]
    [SerializeField] private float standTime = 0;


    [Tooltip("机器人目标高度")]
    [SerializeField] private float targetHeight = 0.8f;

    [Header("机器人body角度奖励，默认只设置臀部奖励")]
    [SerializeField] private float bodyUpReward = 0;

    [Header("给一个随机力")]
    [SerializeField] private float randomForce = 10;
    [Header("施加力的频率")]
    [SerializeField] private float randomForcefrequency = 0.1f;
    [Tooltip("关节平滑")]
    [SerializeField] private float jointLerp = 0.2f;

    // ===== 重置相关 =====
    [Header("重置时给一个随机倾斜角度,从0开始训练")]
    [SerializeField] private float rangeTilt = 0;

    [Header("调试")]
    [SerializeField] private bool showDebugInfo = true;

    [Header("关闭重置")]
    [SerializeField] private bool closeReset = false;


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

        RandomAngle();
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
        int actionSize = GetComponent<BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions;

        // 重置机器人姿态到初始状态
        ResetRobotPose();
        standTime = 0;
        RandomAngle();
    }

    public void RandomAngle()
    {
        if (rangeTilt > 0)
        {
            allJoints[pelvisIndex].gameObject.SetActive(false);
            allJoints[pelvisIndex].transform.rotation = Quaternion.Euler(Random.Range(-rangeTilt, rangeTilt), 0, Random.Range(-rangeTilt, rangeTilt));
            allJoints[pelvisIndex].gameObject.SetActive(true);
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

    bool leftFeetOnGround, rightFeetOnGround;
    bool leftHandOnGround, rightHandOnGround;
    bool pelvisOnGround, torsoOnGround;
    public void BodyHit(string name)
    {
        Debug.Log($"[RobotBalanceAgent] BodyHit: {name}");
        switch (name)
        {
            case "Left_AnkleJoint":
                leftFeetOnGround = true;
                break;
            case "Right_AnkleJoint":
                rightFeetOnGround = true;
                break;
            case "Left_ElbowJoint":
                leftHandOnGround = true;
                break;
            case "Right_ElbowJoint":
                rightHandOnGround = true;
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
        leftFeetOnGround = false;
        rightFeetOnGround = false;
        pelvisOnGround = false;
        torsoOnGround = false;
        rightHandOnGround = false;
        leftHandOnGround = false;
    }
    Vector3 worldCenterOfMass;
    Vector3 worldCenterOfMassVelocity;
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

        // 3. 机器人根节点上
        sensor.AddObservation(body.transform.up);
        dimensionality += 3;

        // 4. 机器人根节点角速度
        sensor.AddObservation(body.transform.InverseTransformVector(body.angularVelocity));
        dimensionality += 3;

        Vector3 leftFeet = allJoints[leftAnkleIndex].transform.position;
        Vector3 rightFeet = allJoints[rightAnkleIndex].transform.position;
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

        sensor.AddObservation(leftFeetOnGround);
        dimensionality += 1;
        sensor.AddObservation(rightFeetOnGround);
        dimensionality += 1;
        sensor.AddObservation(leftHandOnGround);
        dimensionality += 1;
        sensor.AddObservation(rightHandOnGround);
        dimensionality += 1;
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

        // 应用关节力矩（12个连续动作对应12个关节）26d
        ApplyJointForces(actions.ContinuousActions);

        // 计算奖励
        CalculateAndApplyReward();

        // 检查终止条件
        CheckTermination();

        ClearHit();
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

        xDrive.target = Mathf.Lerp(xDrive.target, xForce * 90, jointLerp);
        joint.xDrive = xDrive;

        if (joint.jointType == ArticulationJointType.SphericalJoint)
        {
            yDrive.target = Mathf.Lerp(yDrive.target, yForce * 90, jointLerp);
            joint.yDrive = yDrive;

            zDrive.target = Mathf.Lerp(zDrive.target, zForce * 90, jointLerp);
            joint.zDrive = zDrive;
        }
    }

    /// <summary>
    /// 计算并应用奖励
    /// </summary>
    private void CalculateAndApplyReward()
    {
        Balance();
    }

    public void Balance()
    {
        standTime += Time.fixedDeltaTime;
        ArticulationBody pelvis = allJoints[pelvisIndex];
        ArticulationBody body = allJoints[torsoIndex];

        float reward = 0f;

        // ===== 存活奖励 =====
        reward += 0.01f;

        // ===== 竖直奖励（核心）=====
        float upright = Vector3.Dot(pelvis.transform.up, Vector3.up);
        reward += upright * 0.1f;

        if (bodyUpReward > 0)
        {
            float bodyuUpright = Vector3.Angle(body.transform.up, Vector3.up);
            reward -= bodyuUpright * bodyUpReward;
        }


        if (upright < 0.5f) // 大约50°倾斜
        {
            AddReward(-1f);
            EndEpisode();
            return;
        }

        reward -= Mathf.Abs(pelvis.transform.position.y - targetHeight) * 0.05f;

        // ===== 双脚离地 =====
        if (!leftFeetOnGround && !rightFeetOnGround)
        {
            reward -= 0.05f;
        }

        AddReward(reward);

        AddReward(-pelvis.angularVelocity.sqrMagnitude * 0.001f);

        if (standTime >= targetStandTime)
        {
            //AddReward(+1f);
            EndEpisode();
            return;
        }

        RandomForce();
    }

    public void RandomForce()
    {
        if (randomForce == 0) return;

        if (Random.value < randomForcefrequency)
            allJoints[torsoIndex].AddForce(new Vector3(Random.Range(-randomForce, randomForce), 0, Random.Range(-randomForce, randomForce)));
    }

    public void Move()
    {
    }

    /// <summary>
    /// 检查终止条件
    /// </summary>
    private void CheckTermination()
    {
        if(closeReset) return;
        if (robotRoot == null) return;

        if (pelvisOnGround || torsoOnGround)
        {
            AddReward(-5f);
            EndEpisode();
            return;
        }

        // ===== 双手触地 =====
        if (leftHandOnGround || rightHandOnGround)
        {
            AddReward(-1f);
            EndEpisode();
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
        if (torsoIndex >= 0)
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
        Vector3 point1 = allJoints[leftAnkleIndex].transform.position;
        Vector3 point2 = allJoints[rightAnkleIndex].transform.position;
        point1.y = 0;
        point2.y = 0;
        Debug.DrawLine(point1, point2, Color.green);

        Vector3 velocityOfWorld = allJoints[pelvisIndex].velocity;
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
