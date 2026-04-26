using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;



public class RobotWalkerCustom : Agent
{
    public Camera mainCamera;
    [Header("模仿训练参考")]
    public AnimationStateController reference;

    [Header("机器人配置")]
    [SerializeField] private ArticulationBody robotRoot;

    [Header("OrientationCube")]
    [SerializeField] private Transform orientationCube;

    [Header("目标方块")]
    [SerializeField] private Rigidbody targetCube;

    [Header("目标移动速度 (m/s)")]
    [SerializeField] private float m_maxWalkingSpeed = 10f;

    //目标行走速度
    private float m_TargetWalkingSpeed = 10;

    public float MTargetWalkingSpeed // 属性
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }
    //智能体是否每个回合采样一个新的目标速度？
    //如果为true，walkSpeed将在OnEpisodeBegin()中在0到m_maxWalkingSpeed之间随机设置
    //如果为false，目标速度将是walkingSpeed
    [Header("智能体是否每个回合采样一个新的目标速度？")]
    public bool randomizeWalkSpeedEachEpisode;

    [Header("重置随机角")]
    [SerializeField] private bool randomTilt = true;

    [SerializeField] private float randomForce = 10f;
    [SerializeField] private float randomForceFrequency = 0.1f;
    [Header("目标角度Lerp")]
    [SerializeField] private float targetLerp = 0.5f;
    [Header("关闭重置")]
    [SerializeField] private bool closeReset = false;

    [Header("能量消耗得分比例")]
    [SerializeField] private float energyRatio = 0.5f;

    [Header("目标移动范围")]
    [SerializeField] private float targetSpawnRange = 15f;

    private ArticulationBody[] allJoints;
    public HumanBodyBones[] allJointsBones;
    private float maxStiffness;
    private int pelvisIndex, torsoIndex, headIndex;
    private int leftHipIndex, rightHipIndex;
    private int leftKneeIndex, rightKneeIndex;
    private int leftFeetIndex, rightFeetIndex;
    private int leftShoulderIndex, rightShoulderIndex;
    private int leftElbowIndex, rightElbowIndex;
    Transform leftHandEmpty, rightHandEmpty;

    private Vector3[] initialJointPositions;
    private Quaternion[] initialJointRotations;
    private float[] initialJointXTargets;
    private float[] initialJointYTargets;
    private float[] initialJointZTargets;
    private float[] initialJointStiffness;


    private float successTimer = 0f;
    private float episodeTimer = 0f;

    // 起点位置和朝向（机器人上次碰到目标方块时的位置和朝向）
    private Vector3 spawnPosition;
    private float spawnYRotation;

    [Header("每局最长时间")]
    public float maxEpisodeTime = 70f;

    float mainCameraDis;

    public Animator animator;
    private void Awake()
    {
        animator = GetComponentInChildren<Animator>();
        if (targetCube == null)
        {
            GameObject target = GameObject.CreatePrimitive(PrimitiveType.Cube);
            target.name = "TargetCube";
            target.transform.localScale = Vector3.one * 0.5f;
            // 设置目标方块为第6层级
            target.gameObject.layer = 6;
            targetCube = target.gameObject.AddComponent<Rigidbody>();
            targetCube.gameObject.SetActive(false);
        }
        RandomizeTargetPosition();
    }

    private void Start()
    {
        if(orientationCube == null)
        {
            orientationCube = new GameObject("OrientationCube").transform;
        }
        InitializeRobot();

        // 激活前方块随机位置，确保机器人不会被顶飞
        RandomizeTargetPosition();
        targetCube.gameObject.SetActive(true);

        if (robotRoot == null) robotRoot = GetComponentInChildren<ArticulationBody>();

        if (mainCamera)
        {
            mainCameraDis = Vector3.Distance(allJoints[pelvisIndex].transform.position, mainCamera.transform.position);
        }
    }

    private void InitializeRobot()
    {
        allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();

        // 获取所有关节对应的 HumanBodyBones
        allJointsBones = new HumanBodyBones[allJoints.Length];
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJointsBones[i] = GetHumanBodyBoneFromJointName(allJoints[i].name);
        }

        foreach (var item in allJoints)
        {
            if (item.xDrive.stiffness > maxStiffness)
                maxStiffness = item.xDrive.stiffness;
            if (item.yDrive.stiffness > maxStiffness)
                maxStiffness = item.yDrive.stiffness;
            if (item.zDrive.stiffness > maxStiffness)
                maxStiffness = item.zDrive.stiffness;
            if(!item.gameObject.TryGetComponent(out BodyHit1 bodyHit))
            {
                bodyHit = item.gameObject.AddComponent<BodyHit1>();
                bodyHit.agent = this;
            }
        }
        pelvisIndex = FindJointIndex("Hips");
        torsoIndex = FindJointIndex("Spine");
        headIndex = FindJointIndex("Head");
        leftHipIndex = FindJointIndex("LeftUpLeg");
        rightHipIndex = FindJointIndex("RightUpLeg");
        leftKneeIndex = FindJointIndex("LeftLeg");
        rightKneeIndex = FindJointIndex("RightLeg");
        leftFeetIndex = FindJointIndex("LeftFoot");
        rightFeetIndex = FindJointIndex("RightFoot");
        leftShoulderIndex = FindJointIndex("LeftArm");
        rightShoulderIndex = FindJointIndex("RightArm");
        leftElbowIndex = FindJointIndex("LeftForeArm");
        rightElbowIndex = FindJointIndex("RightForeArm");

        leftHandEmpty = allJoints[leftElbowIndex].transform.GetChild(0);
        rightHandEmpty = allJoints[rightElbowIndex].transform.GetChild(0);

        SaveInitialJointStates();
    }

    private int FindJointIndex(string name)
    {
        for (int i = 0; i < allJoints.Length; i++)
            if (allJoints[i].name.Contains(name)) return i;
        return -1;
    }

    /// <summary>
    /// 根据关节名称获取对应的 HumanBodyBones
    /// </summary>
    private HumanBodyBones GetHumanBodyBoneFromJointName(string jointName)
    {
        string name = jointName.ToLower();

        if (name.Contains("pelvis") || name.Contains("hips")) return HumanBodyBones.Hips;
        if (name.Contains("torso") || name.Contains("spine")) return HumanBodyBones.Spine;
        if (name.Contains("head")) return HumanBodyBones.Head;
        if (name.Contains("neck")) return HumanBodyBones.Neck;

        if (name.Contains("left") && (name.Contains("upleg") || name.Contains("hip") || name.Contains("thigh"))) return HumanBodyBones.LeftUpperLeg;
        if (name.Contains("right") && (name.Contains("upleg") || name.Contains("hip") || name.Contains("thigh"))) return HumanBodyBones.RightUpperLeg;
        if (name.Contains("left") && (name.Contains("leg") || name.Contains("knee") || name.Contains("shin"))) return HumanBodyBones.LeftLowerLeg;
        if (name.Contains("right") && (name.Contains("leg") || name.Contains("knee") || name.Contains("shin"))) return HumanBodyBones.RightLowerLeg;
        if (name.Contains("left") && name.Contains("foot")) return HumanBodyBones.LeftFoot;
        if (name.Contains("right") && name.Contains("foot")) return HumanBodyBones.RightFoot;

        if (name.Contains("left") && (name.Contains("arm") || name.Contains("shoulder") || name.Contains("upperarm"))) return HumanBodyBones.LeftUpperArm;
        if (name.Contains("right") && (name.Contains("arm") || name.Contains("shoulder") || name.Contains("upperarm"))) return HumanBodyBones.RightUpperArm;
        if (name.Contains("left") && (name.Contains("forearm") || name.Contains("elbow") || name.Contains("lowerarm"))) return HumanBodyBones.LeftLowerArm;
        if (name.Contains("right") && (name.Contains("forearm") || name.Contains("elbow") || name.Contains("lowerarm"))) return HumanBodyBones.RightLowerArm;
        if (name.Contains("left") && name.Contains("hand")) return HumanBodyBones.LeftHand;
        if (name.Contains("right") && name.Contains("hand")) return HumanBodyBones.RightHand;

        return HumanBodyBones.LastBone;
    }

    private void SaveInitialJointStates()
    {
        initialJointPositions = new Vector3[allJoints.Length];
        initialJointRotations = new Quaternion[allJoints.Length];
        initialJointXTargets = new float[allJoints.Length];
        initialJointYTargets = new float[allJoints.Length];
        initialJointZTargets = new float[allJoints.Length];
        initialJointStiffness = new float[allJoints.Length];

        for (int i = 0; i < allJoints.Length; i++)
        {
            initialJointPositions[i] = allJoints[i].transform.localPosition;
            initialJointRotations[i] = allJoints[i].transform.localRotation;
            initialJointXTargets[i] = allJoints[i].xDrive.target;
            initialJointYTargets[i] = allJoints[i].yDrive.target;
            initialJointZTargets[i] = allJoints[i].zDrive.target;
            // 保存初始 stiffness（使用 xDrive 的 stiffness）
            initialJointStiffness[i] = allJoints[i].xDrive.stiffness;
        }
    }

    public override void OnEpisodeBegin()
    {
        ResetRobotPose();
        ClearHit();
        episodeTimer = 0f;

        // OrientationCube 跟随 pelvis 位置，但朝向目标方块，y轴向上
        if (orientationCube != null)
        {
            UpdateOrientationCubeRotation();
        }
    }

    public void ResetRobotPose()
    {
        Debug.Log("重置机器人姿势");
        allJoints[pelvisIndex].gameObject.SetActive(false);
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].transform.localPosition = initialJointPositions[i];
            allJoints[i].transform.localRotation = initialJointRotations[i];

            var xDrive = allJoints[i].xDrive;
            xDrive.target = initialJointXTargets[i];
            xDrive.stiffness = initialJointStiffness[i];
            allJoints[i].xDrive = xDrive;

            var yDrive = allJoints[i].yDrive;
            yDrive.target = initialJointYTargets[i];
            yDrive.stiffness = initialJointStiffness[i];
            allJoints[i].yDrive = yDrive;

            var zDrive = allJoints[i].zDrive;
            zDrive.target = initialJointZTargets[i];
            zDrive.stiffness = initialJointStiffness[i];
            allJoints[i].zDrive = zDrive;
        }

        // 使用记录的起点位置（如果是第一次或closeReset则使用初始位置）
        if (spawnPosition != Vector3.zero)
        {
            Vector3 offset = spawnPosition - initialJointPositions[pelvisIndex];
            offset.y = 0f; // 只调整水平位置，保持y轴不变
            allJoints[pelvisIndex].transform.position += offset;
        }

        // 使用记录的Y轴朝向（如果是第一次或closeReset则随机）
        allJoints[pelvisIndex].transform.rotation = Quaternion.Euler(0f, spawnYRotation, 0f);

        allJoints[pelvisIndex].gameObject.SetActive(true);

        if (randomizeWalkSpeedEachEpisode)
        {
            if (reference)
            {
                reference.SetRandomBlend();
                MTargetWalkingSpeed = reference.AnimationSpeed;
            }
            else
            {
                MTargetWalkingSpeed = randomizeWalkSpeedEachEpisode ? Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
            }
        }
        //设置我们的目标行走速度
    }

    /// <summary>
    /// 更新 OrientationCube 的旋转，使其始终指向目标方块，但 y 轴向上
    /// </summary>
    private void UpdateOrientationCubeRotation()
    {
        if (orientationCube == null || targetCube == null) return;

        orientationCube.position = allJoints[pelvisIndex].transform.position;

        Vector3 directionToTarget = targetCube.position - orientationCube.position;
        directionToTarget.y = 0; // 忽略垂直方向

        if (directionToTarget.sqrMagnitude > 0.001f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(directionToTarget, Vector3.up);
            orientationCube.rotation = targetRotation;
        }
    }

    /// <summary>
    /// 随机移动目标方块到新位置（正负15米范围内）
    /// </summary>
    private void RandomizeTargetPosition()
    {
        if (targetCube == null) return;

        float randomX = Random.Range(-targetSpawnRange, targetSpawnRange);
        float randomZ = Random.Range(-targetSpawnRange, targetSpawnRange);
        Vector3 newPosition = new Vector3(randomX, 1, randomZ) + transform.parent.position;

        // 直接设置 Transform 位置，绕过物理引擎
        targetCube.transform.position = newPosition;
        targetCube.transform.rotation = Quaternion.identity;

        // 停止 Rigidbody 运动，防止物理引擎干扰
        targetCube.velocity = Vector3.zero;
        targetCube.angularVelocity = Vector3.zero;
    }

    /// <summary>
    /// 记录当前机器人位置和朝向为新的起点
    /// </summary>
    private void SetCurrentAsSpawnPoint()
    {
        spawnPosition = allJoints[pelvisIndex].transform.position;
        spawnYRotation = allJoints[pelvisIndex].transform.eulerAngles.y;
    }

    Dictionary<GameObject,bool> bodyOnGround = new Dictionary<GameObject, bool>();
    float hitTime;
    public void BodyHit(GameObject name, Collision collision)
    {
        if (collision.gameObject.layer == 3)
        {
            if (bodyOnGround.ContainsKey(name))
                bodyOnGround[name] = true;
            else
                bodyOnGround.Add(name, true);
        }
        if(collision.gameObject.layer == 6)
        {
            if (Time.time == hitTime)
                return;
            // 机器人碰到目标方块
            AddReward(1);
            Debug.Log("碰到目标方块！奖励 +" + 1);

            // 将当前位置设置为新的起点
            SetCurrentAsSpawnPoint();

            // 随机移动目标方块
            RandomizeTargetPosition();

            // 可选：重置 episode 时间
            episodeTimer = 0f;
            hitTime = Time.time;
        }
    }

    public void ClearHit()
    {
        foreach (var item in bodyOnGround.Keys.ToArray())
        {
            bodyOnGround[item] = false;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 更新 OrientationCube 指向目标方块
        UpdateOrientationCubeRotation();

        // 目标方块相对位置
        Vector3 targetPosition = orientationCube.transform.InverseTransformPoint(targetCube.position);
        sensor.AddObservation(targetPosition);

        //布娃娃的平均速度
        var avgVel = GetAvgVelocity();
        //我们要匹配的目标速度
        var velGoal = orientationCube.forward * MTargetWalkingSpeed;
        //当前布娃娃速度，归一化
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));
        //相对于立方体的平均身体速度
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(avgVel));


        var pelvis = allJoints[pelvisIndex];
        // pelvis朝向参考
        sensor.AddObservation(Quaternion.Inverse(allJoints[pelvisIndex].transform.rotation) * orientationCube.rotation);
        sensor.AddObservation(Quaternion.Inverse(allJoints[torsoIndex].transform.rotation) * orientationCube.rotation);
        // 手
        Vector3 leftHand = orientationCube.InverseTransformPoint(leftHandEmpty.position);
        Vector3 rightHand = orientationCube.InverseTransformPoint(rightHandEmpty.position);
        sensor.AddObservation(leftHand);
        sensor.AddObservation(rightHand);

        // 关节旋转（归一化）
        foreach (var joint in allJoints)
        {
            if (joint == pelvis) continue;

            sensor.AddObservation(orientationCube.transform.InverseTransformDirection(joint.velocity));
            sensor.AddObservation(orientationCube.transform.InverseTransformDirection(joint.angularVelocity));
            //在我们的方向立方体空间中获取相对于臀部的位置
            sensor.AddObservation(orientationCube.transform.InverseTransformDirection(joint.worldCenterOfMass - allJoints[pelvisIndex].transform.position));

            if (joint.jointType != ArticulationJointType.FixedJoint)
            {
                sensor.AddObservation(transform.localRotation);
                sensor.AddObservation(joint.xDrive.stiffness / maxStiffness);
            }

            if(bodyOnGround.ContainsKey(joint.gameObject))
                sensor.AddObservation(bodyOnGround[joint.gameObject]);
            else
                sensor.AddObservation(false);
        }
        //每帧关节的能量消耗
        if (energyRatio != 0)
            sensor.AddObservation(energy);
    }

    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //所有刚体
        int numOfRb = 0;
        foreach (var item in allJoints)
        {
            numOfRb++;
            velSum += item.velocity;
            //检查NaN值
            if (float.IsNaN(item.velocity.y))
            {
                EndEpisode();
                AddReward(-1);
                return Vector3.one * 100;
            }
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeTimer += Time.fixedDeltaTime;

        ApplyJointForces(actions.ContinuousActions);

        CalculateReward();

        ImitationTraining();

        CheckDone();

        ClearHit();
    }
    List<float> lastActions = new List<float>();
    public float GetLastAction(int index)
    {
        if (index < lastActions.Count)
            return lastActions[index];
        return 0f;
    }
    public float SetLastAction(int index, float value)
    {
        if (index < lastActions.Count)
            lastActions[index] = value;
        else
            lastActions.Add(value);
        return value;
    }

    public float EnergyConsumption(int index, float action)
    {
        if (energyRatio == 0) return 0;
        float lastAction = GetLastAction(index);
        float energy = Mathf.Abs(action - lastAction);
        SetLastAction(index, action);
        return energy;
    }

    float energy = 0f;
    private void ApplyJointForces(ActionSegment<float> actions)
    {
        if(closeReset) return;
        int index = -1;
        energy = 0f;
        for (int i = 0; i < allJoints.Length; i++)
        {
            float jointEnergy = 0f;
            var joint = allJoints[i];
            if (joint.jointType == ArticulationJointType.FixedJoint) continue;

            // Stiffness 控制（每个关节一个）
            index++;
            float tStiffness = Mathf.Clamp01((actions[index] + 1f) / 2f);
            float targetStiffness = Mathf.Lerp(0f, initialJointStiffness[i], tStiffness);

            if (joint.xDrive.lowerLimit != joint.xDrive.upperLimit && joint.twistLock != ArticulationDofLock.LockedMotion)
            {
                index++;
                float tX = Mathf.Clamp01((actions[index] + 1f) / 2f);
                jointEnergy += EnergyConsumption(index, tX);

                ArticulationDrive xDrive = joint.xDrive;
                float target = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, tX);
                xDrive.target = Mathf.Lerp(xDrive.target, target, targetLerp);
                xDrive.stiffness = targetStiffness;
                joint.xDrive = xDrive;
            }

            if (joint.jointType == ArticulationJointType.SphericalJoint)
            {
                if (joint.yDrive.lowerLimit != joint.yDrive.upperLimit && joint.swingYLock != ArticulationDofLock.LockedMotion)
                {
                    index++;
                    float tY = Mathf.Clamp01((actions[index] + 1f) / 2f);
                    jointEnergy += EnergyConsumption(index, tY);

                    ArticulationDrive yDrive = joint.yDrive;
                    float target = Mathf.Lerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, tY);
                    yDrive.target = Mathf.Lerp(yDrive.target, target, targetLerp);
                    yDrive.stiffness = targetStiffness; // 三个轴使用同一个 stiffness
                    joint.yDrive = yDrive;
                }   

                if (joint.zDrive.lowerLimit != joint.zDrive.upperLimit && joint.swingZLock != ArticulationDofLock.LockedMotion)
                {
                    index++;
                    float tZ = Mathf.Clamp01((actions[index] + 1f) / 2f);
                    jointEnergy += EnergyConsumption(index, tZ);

                    ArticulationDrive zDrive = joint.zDrive;
                    float target = Mathf.Lerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, tZ);
                    zDrive.target = Mathf.Lerp(zDrive.target, target, targetLerp);
                    zDrive.stiffness = targetStiffness; // 三个轴使用同一个 stiffness
                    joint.zDrive = zDrive;
                }
            }
            if (energyRatio != 0)
                energy += jointEnergy * (targetStiffness / maxStiffness);
        }
        AddReward(-(energy * energyRatio));
        Debug.Log("能量消耗 " + energy);
        Debug.Log("输出维度 " + (index + 1));
    }
    Vector3 lastVelocity;
    private void CalculateReward()
    {
        // 更新 OrientationCube 位置
        UpdateOrientationCubeRotation();

        var cubeForward = orientationCube.transform.forward;

        // 根据以下元素的组合为此步骤设置奖励
        // a. 匹配目标速度
        //如果完美匹配，该奖励将接近1，如果偏离则接近0
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

        Vector3 forward = allJoints[headIndex].transform.forward;
        forward.y = 0;
        float facing = (Vector3.Dot(forward, orientationCube.forward) + 1) * .5f;

        //最高得分是1，速度匹配奖励乘以朝向奖励，确保只有当朝向正确时才有高分
        AddReward(matchSpeedReward * facing);
       
        float leftFeetZ = orientationCube.transform.InverseTransformPoint(allJoints[leftFeetIndex].transform.position).z;
        float rightFeetZ = orientationCube.transform.InverseTransformPoint(allJoints[rightFeetIndex].transform.position).z;
        bool leftFoward = leftFeetZ > rightFeetZ;
        //如果发生一次交叉就给奖励，如果ai钻漏洞，降低分数
        if (leftFoward != lastLeftFoward)
            AddReward(0.2f);
        lastLeftFoward = leftFoward;
    }
    bool lastLeftFoward;

    public void ImitationTraining()
    {
        if (!reference) return;

        reference.GetDiff(orientationCube, animator, allJointsBones, out float disDiff, out float angleDiff);

        disDiff = Mathf.Clamp(disDiff, 0, 0.5f) / 0.5f;
        angleDiff = Mathf.Clamp(angleDiff, 0, 90) / 90f;
        float diffReward = (disDiff + angleDiff) / 2f;

        float disReward = Mathf.Pow(1 - Mathf.Pow(diffReward, 2), 2);
        Debug.Log("模仿匹配度" + disReward);
        //限制最高奖励为0.3
        disReward *= 0.3f;
        
        AddReward(disReward);
    }

    //平均速度与目标行走速度差异的归一化值
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        float velocityDifference = Vector3.Distance(actualVelocity, velocityGoal);
        //实际速度与目标速度之间的距离
        var velDeltaMagnitude = Mathf.Clamp(velocityDifference, 0, MTargetWalkingSpeed);

        //返回一条从1衰减到0的下降S形曲线上的值
        //如果完美匹配，该奖励将接近1，如果偏离则接近0
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        // 示例：全设为0
        for (int i = 0; i < continuousActions.Length; i++)
        {
            continuousActions[i] = Input.GetAxis("Horizontal"); 
        }
    }
    private void CheckDone()
    {
        if (closeReset) return;

        // 检查episode时间是否超限
        //if (episodeTimer >= maxEpisodeTime)
        //{
        //    AddReward(-2f);
        //    EndEpisode();
        //    return;
        //}
        // 创建字典快照，避免遍历时被修改
        var bodies = bodyOnGround.ToArray();
        foreach (var body in bodies)
        {
            if (body.Value)
            {
                if (body.Key != allJoints[leftFeetIndex].gameObject && body.Key != allJoints[rightFeetIndex].gameObject /*&& body.Key != allJoints[rightKneeIndex] && body.Key != allJoints[leftKneeIndex]*/)
                {
                    Debug.Log("身体部位 " + body.Key.name + " 接触地面");
                    AddReward(-1f);
                    EndEpisode();
                }
            }
        }
    }

    private float NormalizeAngle180(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        if (angle < -180f) angle += 360f;
        return angle;
    }

    private void FixedUpdate()
    {
        if (mainCamera)
        {
            mainCamera.transform.position = allJoints[pelvisIndex].transform.position - (mainCamera.transform.forward * mainCameraDis);
        }
    }
}