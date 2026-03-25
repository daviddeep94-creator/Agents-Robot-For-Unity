using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// 机器人复位工具类
/// 提供统一的 ArticulationBody 机器人复位功能
/// </summary>
public static class RobotResetUtility
{
    /// <summary>
    /// 重置机器人姿态到保存的初始状态
    /// </summary>
    /// <param name="robotRoot">机器人根节点 ArticulationBody</param>
    /// <param name="initialJointPositions">保存的初始位置数组</param>
    /// <param name="initialJointRotations">保存的初始旋转数组</param>
    /// <param name="initialJointXTargets">保存的初始 X Drive 目标数组</param>
    /// <param name="initialJointYTargets">保存的初始 Y Drive 目标数组</param>
    /// <param name="initialJointZTargets">保存的初始 Z Drive 目标数组</param>
    /// <param name="logPrefix">日志前缀，用于区分调用来源</param>
    public static void ResetRobotPose(
        ArticulationBody robotRoot,
        Vector3[] initialJointPositions,
        Quaternion[] initialJointRotations,
        float[] initialJointXTargets,
        float[] initialJointYTargets,
        float[] initialJointZTargets,
        string logPrefix = "[RobotResetUtility]")
    {
        Debug.Log($"{logPrefix} 开始重置机器人");

        ArticulationBody[] allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();

        // 1. 禁用所有 ArticulationBody
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].enabled = false;
        }

        // 2. 恢复所有关节的位置和旋转
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].transform.localRotation = initialJointRotations[i];
            allJoints[i].transform.localPosition = initialJointPositions[i];
        }

        // 3. 恢复所有关节的驱动目标
        for (int i = 0; i < allJoints.Length; i++)
        {
            if (allJoints[i] != null &&
                allJoints[i].jointType != ArticulationJointType.FixedJoint &&
                i < initialJointXTargets.Length)
            {
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
        }

        // 4. 重新启用所有 ArticulationBody
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].enabled = true;
        }

        Debug.Log($"{logPrefix} 重置完成");
    }


    /// <summary>
    /// 保存机器人的初始状态
    /// </summary>
    public static RobotInitialStates SaveInitialStates(ArticulationBody robotRoot)
    {
        ArticulationBody[] allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        int jointCount = allJoints.Length;

        Vector3[] positions = new Vector3[jointCount];
        Quaternion[] rotations = new Quaternion[jointCount];
        float[] xTargets = new float[jointCount];
        float[] yTargets = new float[jointCount];
        float[] zTargets = new float[jointCount];

        for (int i = 0; i < jointCount; i++)
        {
            if (allJoints[i] != null)
            {
                positions[i] = allJoints[i].transform.localPosition;
                rotations[i] = allJoints[i].transform.localRotation;
                xTargets[i] = allJoints[i].xDrive.target;
                yTargets[i] = allJoints[i].yDrive.target;
                zTargets[i] = allJoints[i].zDrive.target;
            }
        }

        return new RobotInitialStates
        {
            positions = positions,
            rotations = rotations,
            xTargets = xTargets,
            yTargets = yTargets,
            zTargets = zTargets,
            jointCount = jointCount
        };
    }

    /// <summary>
    /// 机器人初始状态数据结构
    /// </summary>
    [System.Serializable]
    public struct RobotInitialStates
    {
        public Vector3[] positions;
        public Quaternion[] rotations;
        public float[] xTargets;
        public float[] yTargets;
        public float[] zTargets;
        public int jointCount;
    }
}
