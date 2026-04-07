using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public enum TargetType
{
    None = 0,
    BalancePoint = 1,
    Move,
}

public class RobotBalanceAgent : Agent
{
    [Header("机器人配置")]
    [SerializeField] private ArticulationBody robotRoot;

    [Header("OrientationCube")]
    [SerializeField] private Transform orientationCube;

    [Header("重置随机角")]
    [SerializeField] private bool randomTilt = true;

    [SerializeField] private float randomForce = 10f;
    [SerializeField] private float randomForceFrequency = 0.1f;

    [Header("关闭重置")]
    [SerializeField] private bool closeReset = false;

    private ArticulationBody[] allJoints;
    private float maxStiffness;
    private int pelvisIndex, torsoIndex;
    private int leftHipIndex, rightHipIndex;
    private int leftKneeIndex, rightKneeIndex;
    private int leftAnkleIndex, rightAnkleIndex;
    private int leftShoulderIndex, rightShoulderIndex;
    private int leftElbowIndex, rightElbowIndex;

    [SerializeField] private Transform leftHandEmpty;
    [SerializeField] private Transform rightHandEmpty;

    private Vector3[] initialJointPositions;
    private Quaternion[] initialJointRotations;
    private float[] initialJointXTargets;
    private float[] initialJointYTargets;
    private float[] initialJointZTargets;

    private bool leftFeetOnGround, rightFeetOnGround;
    private bool leftHandOnGround, rightHandOnGround;
    private bool pelvisOnGround, torsoOnGround;

    private float successTimer = 0f;
    private float episodeTimer = 0f;

    [Header("每局最长时间")]
    public float maxEpisodeTime = 20f;

    private void Start()
    {
        if (robotRoot == null) robotRoot = GetComponentInChildren<ArticulationBody>();
        InitializeRobot();
        RandomTilt();
    }

    private void InitializeRobot()
    {
        allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        foreach (var item in allJoints)
        {
            if (item.xDrive.stiffness > maxStiffness)
                maxStiffness = item.xDrive.stiffness;
            if (item.yDrive.stiffness > maxStiffness)
                maxStiffness = item.yDrive.stiffness;
            if (item.zDrive.stiffness > maxStiffness)
                maxStiffness = item.zDrive.stiffness;
        }
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

        leftHandEmpty = robotRoot.transform.Find("Torso_Joint/Left_ShoulderJoint/Left_ElbowJoint/Left_Hand_Empty");
        rightHandEmpty = robotRoot.transform.Find("Torso_Joint/Right_ShoulderJoint/Right_ElbowJoint/Right_Hand_Empty");

        SaveInitialJointStates();
    }

    private int FindJointIndex(string name)
    {
        for (int i = 0; i < allJoints.Length; i++)
            if (allJoints[i].name == name) return i;
        return -1;
    }

    private void SaveInitialJointStates()
    {
        initialJointPositions = new Vector3[allJoints.Length];
        initialJointRotations = new Quaternion[allJoints.Length];
        initialJointXTargets = new float[allJoints.Length];
        initialJointYTargets = new float[allJoints.Length];
        initialJointZTargets = new float[allJoints.Length];

        for (int i = 0; i < allJoints.Length; i++)
        {
            initialJointPositions[i] = allJoints[i].transform.localPosition;
            initialJointRotations[i] = allJoints[i].transform.localRotation;
            initialJointXTargets[i] = allJoints[i].xDrive.target;
            initialJointYTargets[i] = allJoints[i].yDrive.target;
            initialJointZTargets[i] = allJoints[i].zDrive.target;
        }
    }

    public override void OnEpisodeBegin()
    {
        ResetRobotPose();
        ClearHit();
        episodeTimer = 0f;
        orientationCube.rotation = Quaternion.identity;
        RandomTilt();

        // OrientationCube 跟随 pelvis 位置
        if (orientationCube != null)
            orientationCube.position = allJoints[pelvisIndex].transform.position;
    }

    public void ResetRobotPose()
    {
        allJoints[pelvisIndex].gameObject.SetActive(false);
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].transform.localPosition = initialJointPositions[i];
            allJoints[i].transform.localRotation = initialJointRotations[i];

            var xDrive = allJoints[i].xDrive;
            xDrive.target = initialJointXTargets[i];
            allJoints[i].xDrive = xDrive;

            var yDrive = allJoints[i].yDrive;
            yDrive.target = initialJointYTargets[i];
            allJoints[i].yDrive = yDrive;

            var zDrive = allJoints[i].zDrive;
            zDrive.target = initialJointZTargets[i];
            allJoints[i].zDrive = zDrive;
        }

        // 随机Y轴旋转
        allJoints[pelvisIndex].transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);


        allJoints[pelvisIndex].gameObject.SetActive(true);
    }

    private void RandomTilt()
    {
        if (!randomTilt) return;
        float xAngle = Random.Range(-10f, 10f);
        float zAngle = Random.Range(-10f, 10f);
        allJoints[pelvisIndex].transform.rotation *= Quaternion.Euler(xAngle, 0, zAngle);
    }

    public void BodyHit(string name)
    {
        switch (name)
        {
            case "Left_AnkleJoint": leftFeetOnGround = true; break;
            case "Right_AnkleJoint": rightFeetOnGround = true; break;
            case "Left_ElbowJoint": leftHandOnGround = true; break;
            case "Right_ElbowJoint": rightHandOnGround = true; break;
            case "Pelvis_Joint": pelvisOnGround = true; break;
            case "Torso_Joint": torsoOnGround = true; break;
        }
    }

    public void ClearHit()
    {
        leftFeetOnGround = rightFeetOnGround = false;
        leftHandOnGround = rightHandOnGround = false;
        pelvisOnGround = torsoOnGround = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        orientationCube.position = allJoints[pelvisIndex].transform.position;
        //布娃娃的平均速度
        var avgVel = GetAvgVelocity();
        //当前布娃娃速度，归一化
        sensor.AddObservation(avgVel.magnitude);
        //相对于立方体的平均身体速度
        sensor.AddObservation(orientationCube.transform.InverseTransformDirection(avgVel));

        var pelvis = allJoints[pelvisIndex];
        // pelvis朝向参考
        sensor.AddObservation(Quaternion.FromToRotation(allJoints[pelvisIndex].transform.forward, orientationCube.forward));

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
            sensor.AddObservation(orientationCube.transform.InverseTransformDirection(joint.worldCenterOfMass - allJoints[pelvisIndex].worldCenterOfMass));

            if (joint.jointType != ArticulationJointType.FixedJoint)
            {
                sensor.AddObservation(NormalizeAngle180(joint.transform.localRotation.eulerAngles.x) / 180f);
                sensor.AddObservation(NormalizeAngle180(joint.transform.localRotation.eulerAngles.y) / 180f);
                sensor.AddObservation(NormalizeAngle180(joint.transform.localRotation.eulerAngles.z) / 180f);
                sensor.AddObservation(joint.xDrive.stiffness / maxStiffness);
            }
        }

        sensor.AddObservation(leftFeetOnGround);
        sensor.AddObservation(rightFeetOnGround);
        sensor.AddObservation(leftHandOnGround);
        sensor.AddObservation(rightHandOnGround);
        sensor.AddObservation(pelvisOnGround);
        sensor.AddObservation(torsoOnGround);
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
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeTimer += Time.fixedDeltaTime;

        ApplyJointForces(actions.ContinuousActions);

        CalculateReward();

        CheckDone();

        ClearHit();
    }

    private void ApplyJointForces(ActionSegment<float> actions)
    {
        int index = 0;
        for (int i = 0; i < allJoints.Length; i++)
        {
            var joint = allJoints[i];
            if (joint.jointType == ArticulationJointType.FixedJoint) continue;

            float tX = Mathf.Clamp01((actions[index++] + 1f) / 2f);
            ArticulationDrive xDrive = joint.xDrive;
            xDrive.target = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, tX);
            joint.xDrive = xDrive;
            if (joint.jointType == ArticulationJointType.SphericalJoint)
            {
                float tY = Mathf.Clamp01((actions[index++] + 1f) / 2f);
                float tZ = Mathf.Clamp01((actions[index++] + 1f) / 2f);

                ArticulationDrive yDrive = joint.yDrive;
                yDrive.target = Mathf.Lerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, tY);
                joint.yDrive = yDrive;

                ArticulationDrive zDrive = joint.zDrive;
                zDrive.target = Mathf.Lerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, tZ);
                joint.zDrive = zDrive;
            }
        }
        Debug.Log("输出维度 " + index);
    }
    Vector3 lastVelocity;
    private void CalculateReward()
    {
        float pForward = (Vector3.Dot(allJoints[pelvisIndex].transform.forward, orientationCube.forward) + 1) * 0.5f;
        float pUp = (Vector3.Dot(allJoints[pelvisIndex].transform.up, orientationCube.up) + 1) * 0.5f;
        float bForward = (Vector3.Dot(allJoints[torsoIndex].transform.forward, orientationCube.forward) + 1) * 0.5f;

        float facing = pForward * pUp * bForward;
        Vector3 velocity = GetAvgVelocity();
        float stable = 1 - Mathf.Pow(velocity.magnitude, 2);
        Debug.Log("稳定性 " + stable);


        float reward = stable * facing;

        AddReward(reward);
        //抖动惩罚：如果当前速度与上一次速度方向相反，说明发生了抖动，给予额外惩罚
        if (Vector3.Dot(velocity, lastVelocity) < 0)
        {
            float shakePenalty = Vector3.Distance(velocity, lastVelocity); // 抖动惩罚强度可调整
            shakePenalty *= shakePenalty;
            shakePenalty *= 0.5f;
            Debug.Log("抖动惩罚 " + shakePenalty);
            AddReward(-shakePenalty);
        }
        lastVelocity = velocity;
        Debug.Log("得分 " + reward);
        if (stable > 0.9f && facing > 0.9f)
        {
            successTimer += Time.fixedDeltaTime;
        }
        else
        {
            successTimer = 0f;
        }
        //抖动惩罚
        //float shake = allJoints[pelvisIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[torsoIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[leftAnkleIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[rightAnkleIndex].angularVelocity.sqrMagnitude;
        //shake *= 0.005f;
        //AddReward(shake);
        //Debug.Log("抖动惩罚 " + shake);

        if (successTimer > 2f)
        {
            AddReward(10);
            orientationCube.rotation = Quaternion.Euler(0, Random.Range(0, 360f), 0);
            successTimer = 0;
            episodeTimer = 0;
            return;
        }
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

        if (pelvisOnGround || torsoOnGround || leftHandOnGround || rightHandOnGround)
        {
            AddReward(-2f);
            EndEpisode();
            return;
        }

        // 检查episode时间是否超限
        if (episodeTimer >= maxEpisodeTime)
        {
            AddReward(-2f);
            EndEpisode();
            return;
        }
    }

    private float NormalizeAngle180(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        if (angle < -180f) angle += 360f;
        return angle;
    }
}