using UnityEngine;
using UnityEditor;

/// <summary>
/// 训练场景设置工具
/// 用于快速设置训练场景
/// </summary>
public class TrainingSceneSetup : EditorWindow
{
    [MenuItem("Window/AI Robot/Training Scene Setup")]
    public static void Open()
    {
        GetWindow<TrainingSceneSetup>("Training Scene Setup");
    }

    private void OnGUI()
    {
        GUILayout.Label("训练场景设置", EditorStyles.boldLabel);
        GUILayout.Space(10);

        GUILayout.Label("步骤1: 创建训练环境", EditorStyles.boldLabel);
        GUILayout.Label("1. 使用 Articulation Robot Generator 生成机器人");
        GUILayout.Label("2. 将机器人放到场景中合适位置");

        if (GUILayout.Button("添加训练管理器"))
        {
            CreateTrainingManager();
        }

        GUILayout.Space(10);

        GUILayout.Label("步骤2: 配置平衡训练", EditorStyles.boldLabel);
        if (GUILayout.Button("添加平衡Agent到机器人"))
        {
            AddBalanceAgent();
        }

        GUILayout.Space(10);

        GUILayout.Label("步骤3: 配置移动训练", EditorStyles.boldLabel);
        if (GUILayout.Button("添加移动Agent到机器人"))
        {
            AddMovementAgent();
        }

        GUILayout.Space(10);

        GUILayout.Label("步骤4: 地面设置", EditorStyles.boldLabel);
        if (GUILayout.Button("创建训练地面"))
        {
            CreateTrainingGround();
        }

        GUILayout.Space(10);

        GUILayout.Label("训练提示", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "1. 先训练平衡（阶段1），让机器人学会站立\n" +
            "2. 平衡训练达标后，切换到移动训练（阶段2）\n" +
            "3. 使用TensorBoard监控训练进度\n" +
            "4. 定期保存检查点",
            MessageType.Info
        );
    }

    private void CreateTrainingManager()
    {
        GameObject managerObj = new GameObject("TrainingManager");
        managerObj.AddComponent<TrainingManager>();
        Selection.activeGameObject = managerObj;
        Debug.Log("已创建 TrainingManager");
    }

    private void AddBalanceAgent()
    {
        var robot = GameObject.Find("GeneratedArticulationRobot");
        if (robot == null)
        {
            EditorUtility.DisplayDialog("错误", "未找到 GeneratedArticulationRobot，请先生成机器人。", "确定");
            return;
        }

        var agent = robot.AddComponent<RobotBalanceAgent>();
        var root = robot.GetComponentInChildren<ArticulationBody>();
        UnityEditor.SerializedObject so = new UnityEditor.SerializedObject(agent);
        so.FindProperty("robotRoot").objectReferenceValue = root;
        so.ApplyModifiedProperties();

        Selection.activeGameObject = robot.gameObject;
        Debug.Log("已添加 RobotBalanceAgent 到机器人");
    }

    private void AddMovementAgent()
    {
        var robot = GameObject.Find("GeneratedArticulationRobot");
        if (robot == null)
        {
            EditorUtility.DisplayDialog("错误", "未找到 GeneratedArticulationRobot，请先生成机器人。", "确定");
            return;
        }

        var agent = robot.AddComponent<RobotMovementAgent>();
        var root = robot.GetComponentInChildren<ArticulationBody>();
        UnityEditor.SerializedObject so = new UnityEditor.SerializedObject(agent);
        so.FindProperty("robotRoot").objectReferenceValue = root;
        so.ApplyModifiedProperties();

        agent.enabled = false; // 初始禁用

        Selection.activeGameObject = robot.gameObject;
        Debug.Log("已添加 RobotMovementAgent 到机器人");
    }

    private void CreateTrainingGround()
    {
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.name = "TrainingGround";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(10, 1, 10);

        Renderer renderer = ground.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = new Material(Shader.Find("Standard"));
            renderer.material.color = new Color(0.3f, 0.3f, 0.3f, 1f);
        }

        Selection.activeGameObject = ground;
        Debug.Log("已创建训练地面");
    }
}
