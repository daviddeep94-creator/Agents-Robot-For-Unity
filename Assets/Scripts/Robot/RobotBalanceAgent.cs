using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

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
    [Header("目标角度Lerp")]
    [SerializeField] private float targetLerp = 0.5f;
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


    private float successTimer = 0f;
    private float episodeTimer = 0f;

    [Header("每局最长时间")]
    public float maxEpisodeTime = 20f;

    private void Start()
    {
        if(orientationCube == null)
        {
            orientationCube = new GameObject("OrientationCube").transform;
        }
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
        if (!closeReset)
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

    Dictionary<string,bool> bodyOnGround = new Dictionary<string, bool>();
    public void BodyHit(string name)
    {
        if(bodyOnGround.ContainsKey(name))
            bodyOnGround[name] = true;
        else
            bodyOnGround.Add(name, true);

        if (name != "Left_AnkleJoint" && name != "Right_AnkleJoint" && name != "Right_KneeJoint" && name != "Left_KneeJoint")
        {
            AddReward(-2f);
            EndEpisode();
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
        orientationCube.position = allJoints[pelvisIndex].transform.position;
        //布娃娃的平均速度
        var avgVel = GetAvgVelocity();

        //当前布娃娃速度，归一化
        sensor.AddObservation(avgVel.magnitude);
        //每帧关节的能量消耗
        sensor.AddObservation(energy);
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

            if(bodyOnGround.ContainsKey(joint.name))
                sensor.AddObservation(bodyOnGround[joint.name]);
            else
                sensor.AddObservation(false);
        }
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
                AddReward(-10);
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
            var joint = allJoints[i];
            if (joint.jointType == ArticulationJointType.FixedJoint) continue;

            index++;
            float tX = Mathf.Clamp01((actions[index] + 1f) / 2f);
            energy+= EnergyConsumption(index, tX);

            ArticulationDrive xDrive = joint.xDrive;
            float target = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, tX);
            xDrive.target = Mathf.Lerp(xDrive.target, target, targetLerp);
            joint.xDrive = xDrive;
            if (joint.jointType == ArticulationJointType.SphericalJoint)
            {
                index++;
                float tY = Mathf.Clamp01((actions[index] + 1f) / 2f);
                energy += EnergyConsumption(index, tY);
                index++;
                float tZ = Mathf.Clamp01((actions[index] + 1f) / 2f);
                energy += EnergyConsumption(index, tZ);

                ArticulationDrive yDrive = joint.yDrive;
                target = Mathf.Lerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, tY);
                yDrive.target = Mathf.Lerp(yDrive.target, target, targetLerp);
                joint.yDrive = yDrive;

                ArticulationDrive zDrive = joint.zDrive;
                target = Mathf.Lerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, tZ);
                zDrive.target = Mathf.Lerp(zDrive.target, target, targetLerp);
                joint.zDrive = zDrive;
            }
        }
        AddReward(-(energy * 0.1f));
        Debug.Log("能量消耗 " + energy);
        Debug.Log("输出维度 " + (index + 1));
    }
    Vector3 lastVelocity;
    private void CalculateReward()
    {
        //Debug.Log($"pelvisIndex相对于orientationCube{(allJoints[pelvisIndex].transform.rotation *orientationCube.rotation).eulerAngles}");
        //float pUp = 1 - Vector3.Angle(allJoints[pelvisIndex].transform.up, orientationCube.up) / 180f;
        //float pforward = 1 - Vector3.Angle(allJoints[pelvisIndex].transform.forward, orientationCube.forward) / 180f;

        //float tUp = 1 - Vector3.Angle(allJoints[torsoIndex].transform.up, orientationCube.up) / 180f;
        //float tforward = 1 - Vector3.Angle(allJoints[torsoIndex].transform.forward, orientationCube.forward) / 180f;

        //float facing = (pUp + pforward) * (tUp + tforward);
        //facing *= 0.25f;
        orientationCube.position = allJoints[pelvisIndex].transform.position;
        float pAngle = 1 - Quaternion.Angle(allJoints[pelvisIndex].transform.rotation, orientationCube.rotation) / 180f;
        float tAngle = 1 - Quaternion.Angle(allJoints[torsoIndex].transform.rotation, orientationCube.rotation) / 180f;
        Debug.Log($"pAngle: {pAngle} tAngle: {tAngle}" );
        float facing = pAngle * pAngle + tAngle * tAngle;
        Vector3 velocity = GetAvgVelocity();

        float stable = 1 - Mathf.Pow(velocity.magnitude, 2);
        Debug.Log("稳定性 " + stable);


        float reward = stable + facing;

        AddReward(reward);
        //最高得分是3，完全站立不动且朝向正确
        Debug.Log("得分 " + reward);
        if (stable > 0.9f && facing > 0.9f)
        {
            successTimer += Time.fixedDeltaTime;
        }
        else
        {
            successTimer = 0f;
        }

        if (successTimer > 2f)
        {
            AddReward(10);
            if (!closeReset)
                orientationCube.rotation = Quaternion.Euler(0, Random.Range(0, 360f), 0);
            successTimer = 0;
            episodeTimer = 0;
            return;
        }

        float leftFeetZ = orientationCube.InverseTransformPoint(allJoints[leftAnkleIndex].worldCenterOfMass).z;
        float rightFeetZ = orientationCube.InverseTransformPoint(allJoints[rightAnkleIndex].worldCenterOfMass).z;

        AddReward(-Mathf.Abs(leftFeetZ - rightFeetZ));

        //if(!leftFeetOnGround && !rightFeetOnGround)
        //{
        //    AddReward(-0.5f);
        //}
        //抖动惩罚：如果当前速度与上一次速度方向相反，说明发生了抖动，给予额外惩罚
        //if (Vector3.Dot(velocity, lastVelocity) < 0)
        //{
        //    float shakePenalty = Vector3.Distance(velocity, lastVelocity); // 抖动惩罚强度可调整
        //    shakePenalty *= shakePenalty;
        //    shakePenalty *= 0.5f;
        //    Debug.Log("抖动惩罚 " + shakePenalty);
        //    AddReward(-shakePenalty);
        //}
        //lastVelocity = velocity;
        //抖动惩罚
        //float shake = allJoints[pelvisIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[torsoIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[leftAnkleIndex].angularVelocity.sqrMagnitude;
        //shake += allJoints[rightAnkleIndex].angularVelocity.sqrMagnitude;
        //shake *= 0.005f;
        //AddReward(shake);
        //Debug.Log("抖动惩罚 " + shake);
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