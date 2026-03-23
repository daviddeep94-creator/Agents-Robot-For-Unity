using System;
using UnityEngine;

/// <summary>
/// 机器人复位测试工具
/// </summary>
public class RobotResetTester : MonoBehaviour
{
    [Header("机器人配置")]
    [Tooltip("机器人的根节点 ArticulationBody")]
    public Transform robotRoot;

    [Header("UI 配置")]
    [Tooltip("按钮位置")]
    public Vector2 buttonPosition = new Vector2(10, 100);
    [Tooltip("按钮大小")]
    public Vector2 buttonSize = new Vector2(120, 40);

    [Header("调试信息")]
    [SerializeField] private bool showDebugInfo = true;

    // 保存的初始状态
    private ArticulationBody[] allJoints;
    private Quaternion[] initialJointRotations;
    private Vector3[] initialJointPosition;

    private void Start()
    {
        Debug.Log("[RobotResetTester] Start() 被调用");

        // 自动查找机器人根节点
        if (robotRoot)
        {
            allJoints = robotRoot.GetComponentsInChildren<ArticulationBody>();
        }
        else
        {
            Debug.Log("没有找到robotRoot");
            return;
        }
        initialJointRotations = new Quaternion[allJoints.Length];
        initialJointPosition = new Vector3[allJoints.Length];
        for (int i = 0; i < allJoints.Length; i++)
        {
            initialJointRotations[i] = allJoints[i].transform.localRotation;
            initialJointPosition[i] = allJoints[i].transform.localPosition;
        }
    }




    private void OnGUI()
    {
        GUI.color = Color.white;

        // 显示操作提示
        GUILayout.BeginArea(new Rect(10, 10, 300, 120));
        GUILayout.Label("=== 机器人复位测试工具 ===");
        GUILayout.Label($"当前机器人位置: {robotRoot.transform.position:F2}");
        GUILayout.Label($"当前机器人旋转: {robotRoot.transform.rotation.eulerAngles:F1}°");
        GUILayout.Label($"按下方按钮或空格键重置机器人");
        GUILayout.EndArea();

        // 绘制重置按钮
        if (GUI.Button(new Rect(buttonPosition.x, buttonPosition.y, buttonSize.x, buttonSize.y), "重置机器人"))
        {
            ResetRobot();
        }
    }

    private void Update()
    {
        // 空格键触发重置
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ResetRobot();
        }
    }

    private void ResetRobot()
    {
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].enabled = false;
        }
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].transform.localRotation = initialJointRotations[i];
            allJoints[i].transform.localPosition = initialJointPosition[i];
        }
        for (int i = 0; i < allJoints.Length; i++)
        {
            allJoints[i].enabled = true;
        }
    }
}
