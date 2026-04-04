using System;
using System.Drawing;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

/// <summary>
/// 在编辑器里快速生成一套 ArticulationBody 机器人（方块拼接）。
/// </summary>
public class ArticulationRobotGeneratorWindow : EditorWindow
{
    private const string RobotRootName = "GeneratedArticulationRobot";
    private const string ConfigFileName = "ArticulationRobotConfig.json";
    private const string ConfigFolderName = "RobotGeneratorConfig";

    [Header("基础尺寸")]
    [SerializeField] private float robotHeight = 1.75f; // 整体缩放因子

    // 以下所有值基于 robotHeight = 1.0f 的归一化值
    [SerializeField] private float legLengthNormalized = 0.515f; // 腿长约占身高的51.5%

    [Tooltip("躯干宽度（左右肩宽），归一化值。")]
    [SerializeField] private float torsoWidthNormalized = 0.17f;

    [Tooltip("躯干厚度（前后胸腹厚度），归一化值。")]
    [SerializeField] private float torsoDepthNormalized = 0.114f;

    [Tooltip("臀部宽度，归一化值。")]
    [SerializeField] private float pelvisWidthNormalized = 0.17f;

    [Tooltip("手臂总长度（上臂+前臂），归一化值。")]
    [SerializeField] private float armLengthNormalized = 0.428f;

    [Tooltip("头部大小（直径），归一化值。")]
    [SerializeField] private float headSizeNormalized = 0.103f;

    [Tooltip("脚板尺寸（长×宽×高），归一化值。")]
    [SerializeField] private Vector3 footSizeNormalized = new Vector3(0.06f, 0.0286f, 0.15f);

    [Header("质量与阻尼")]
    [SerializeField] private float totalMassKg = 60.0f;
    [SerializeField] private float angularDamping = 2.0f;
    [SerializeField] private float jointFriction = 0.5f;

    [Header("关节驱动参数")]
    [SerializeField] private float passiveJointForceLimit = 1e6f;

    [Header("关节刚度参数 (AI 训练用固定值)")]
    [Tooltip("全局刚度缩放因子 - 控制所有关节的整体刚度")]
    [SerializeField] private float globalStiffnessScale = 1.0f;

    [Tooltip("髋关节刚度 - 需要大力气支撑身体")]
    [SerializeField] private float hipJointStiffness = 1000.0f;

    [Tooltip("膝关节刚度 - 主要承重关节")]
    [SerializeField] private float kneeJointStiffness = 1000.0f;

    [Tooltip("踝关节刚度 - 平衡控制")]
    [SerializeField] private float ankleJointStiffness = 800.0f;

    [Tooltip("肩关节刚度 - 手臂活动")]
    [SerializeField] private float shoulderJointStiffness = 300.0f;

    [Tooltip("肘关节刚度 - 精细控制")]
    [SerializeField] private float elbowJointStiffness = 300.0f;

    [Tooltip("躯干关节刚度 - 身体平衡")]
    [SerializeField] private float torsoJointStiffness = 500.0f;

    [Tooltip("颈部关节刚度 - 头部转动")]
    [SerializeField] private float neckJointStiffness = 300.0f;

    [Header("关节旋转限制(度) - Lower, Upper")]
    // 臀部：前后、左右、扭转
    [SerializeField] private Vector2 pelvisSwingY = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 pelvisSwingZ = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 pelvisTwist = new Vector2(-45f, 45f);

    // 躯干：前后、左右、扭转
    [SerializeField] private Vector2 torsoSwingY = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 torsoSwingZ = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 torsoTwist = new Vector2(-45f, 45f);

    // 颈部：前后、左右、扭转
    [SerializeField] private Vector2 neckSwingY = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 neckSwingZ = new Vector2(-45f, 45f);
    [SerializeField] private Vector2 neckTwist = new Vector2(-45f, 45f);

    // 髋关节：xDrive=前后摆动，yDrive=扭转，zDrive=左右摆动
    [SerializeField] private Vector2 hipTwist = new Vector2(-110.0f, 30.0f);  // 前后摆动(X轴)
    [SerializeField] private Vector2 hipSwingY = new Vector2(0.0f, 0.0f);  // 扭转(Y轴)
    [SerializeField] private Vector2 hipSwingZ = new Vector2(-40.0f, 5.0f);  // 左右摆动(Z轴)

    // 膝关节：只能向前弯曲150度
    [SerializeField] private Vector2 kneeTwist = new Vector2(0f, 150f);  // 弯曲(X轴)

    // 踝关节：前后45度，左右20度，扭转15度
    [SerializeField] private Vector2 ankleTwist = new Vector2(-45.0f, 45.0f);  // 前后摆动(X轴)
    [SerializeField] private Vector2 ankleSwingY = new Vector2(0.0f, 0.0f);  // 扭转(Y轴)
    [SerializeField] private Vector2 ankleSwingZ = new Vector2(-20.0f, 20.0f);  // 左右摆动(Z轴)

    // 肩关节：前后180度，左右90度，扭转180度
    [SerializeField] private Vector2 shoulderTwist = new Vector2(-90.0f, 90.0f);  // 前后摆动(X轴)
    [SerializeField] private Vector2 shoulderSwingY = new Vector2(-90.0f, 90.0f);  // 扭转(Y轴)
    [SerializeField] private Vector2 shoulderSwingZ = new Vector2(-90.0f, 0.0f);    // 左右摆动(Z轴)

    // 肘关节：只能向前弯曲150度
    [SerializeField] private Vector2 elbowTwist = new Vector2(0f, 150f);  // 弯曲(X轴)

    [Header("生成姿态")]
    [Tooltip("膝盖初始弯曲角度（度），用于生成时的初始姿态")]
    [SerializeField] private float initialKneeBendAngle = 20f;

    [SerializeField] private Vector3 spawnPosition = new Vector3(0, 0.0f, 0);
    [SerializeField] private bool replaceExisting = true;

    // 滚动位置
    private Vector2 scrollPosition;

    [MenuItem("Window/AI Robot/Articulation Robot Generator")]
    public static void Open()
    {
        GetWindow<ArticulationRobotGeneratorWindow>("Articulation Robot Generator");
    }

    private void OnGUI()
    {
        scrollPosition = EditorGUILayout.BeginScrollView(scrollPosition);

        EditorGUILayout.Space(6);

        // 基础尺寸
        EditorGUILayout.LabelField("基础尺寸", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        robotHeight = EditorGUILayout.FloatField("机器高度", robotHeight, GUILayout.Width(200));
        legLengthNormalized = EditorGUILayout.FloatField("腿长", legLengthNormalized, GUILayout.Width(200));
        torsoWidthNormalized = EditorGUILayout.FloatField("躯干宽度", torsoWidthNormalized, GUILayout.Width(200));
        torsoDepthNormalized = EditorGUILayout.FloatField("躯干厚度", torsoDepthNormalized, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
        pelvisWidthNormalized = EditorGUILayout.FloatField("臀部宽度", pelvisWidthNormalized, GUILayout.Width(200));
        armLengthNormalized = EditorGUILayout.FloatField("手臂长度", armLengthNormalized, GUILayout.Width(200));
        headSizeNormalized = EditorGUILayout.FloatField("头部大小", headSizeNormalized, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        footSizeNormalized = EditorGUILayout.Vector3Field("脚板尺寸", footSizeNormalized);

        // 质量与阻尼
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("质量与阻尼", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        totalMassKg = EditorGUILayout.FloatField("总重量(kg)", totalMassKg, GUILayout.Width(200));
        angularDamping = EditorGUILayout.FloatField("关节阻尼", angularDamping, GUILayout.Width(200));
        jointFriction = EditorGUILayout.FloatField("关节摩擦", jointFriction, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        passiveJointForceLimit = EditorGUILayout.FloatField("关节驱动力上限", passiveJointForceLimit);

        // 关节刚度参数
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("关节刚度参数 (AI 训练用固定值)", EditorStyles.boldLabel);

        globalStiffnessScale = EditorGUILayout.Slider("全局刚度缩放", globalStiffnessScale, 0.1f, 3.0f);

        EditorGUILayout.BeginHorizontal();
        hipJointStiffness = EditorGUILayout.FloatField("髋关节刚度", hipJointStiffness, GUILayout.Width(200));
        kneeJointStiffness = EditorGUILayout.FloatField("膝关节刚度", kneeJointStiffness, GUILayout.Width(200));
        ankleJointStiffness = EditorGUILayout.FloatField("踝关节刚度", ankleJointStiffness, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();
        shoulderJointStiffness = EditorGUILayout.FloatField("肩关节刚度", shoulderJointStiffness, GUILayout.Width(200));
        elbowJointStiffness = EditorGUILayout.FloatField("肘关节刚度", elbowJointStiffness, GUILayout.Width(200));
        torsoJointStiffness = EditorGUILayout.FloatField("躯干关节刚度", torsoJointStiffness, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        neckJointStiffness = EditorGUILayout.FloatField("颈部关节刚度", neckJointStiffness);

        // 关节旋转限制
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("关节旋转限制(度) - Lower, Upper", EditorStyles.boldLabel);

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("臀部", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        pelvisSwingY = EditorGUILayout.Vector2Field("前后", pelvisSwingY, GUILayout.Width(200));
        pelvisSwingZ = EditorGUILayout.Vector2Field("左右", pelvisSwingZ, GUILayout.Width(200));
        pelvisTwist = EditorGUILayout.Vector2Field("扭转", pelvisTwist, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("躯干", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        torsoSwingY = EditorGUILayout.Vector2Field("前后", torsoSwingY, GUILayout.Width(200));
        torsoSwingZ = EditorGUILayout.Vector2Field("左右", torsoSwingZ, GUILayout.Width(200));
        torsoTwist = EditorGUILayout.Vector2Field("扭转", torsoTwist, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("颈部", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        neckSwingY = EditorGUILayout.Vector2Field("前后", neckSwingY, GUILayout.Width(200));
        neckSwingZ = EditorGUILayout.Vector2Field("左右", neckSwingZ, GUILayout.Width(200));
        neckTwist = EditorGUILayout.Vector2Field("扭转", neckTwist, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("髋关节", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        hipTwist = EditorGUILayout.Vector2Field("前后(X)", hipTwist, GUILayout.Width(200));
        hipSwingY = EditorGUILayout.Vector2Field("扭转(Y)", hipSwingY, GUILayout.Width(200));
        hipSwingZ = EditorGUILayout.Vector2Field("左右(Z)", hipSwingZ, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("膝关节", EditorStyles.miniBoldLabel);
        kneeTwist = EditorGUILayout.Vector2Field("弯曲(X)", kneeTwist);

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("踝关节", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        ankleTwist = EditorGUILayout.Vector2Field("前后(X)", ankleTwist, GUILayout.Width(200));
        ankleSwingY = EditorGUILayout.Vector2Field("扭转(Y)", ankleSwingY, GUILayout.Width(200));
        ankleSwingZ = EditorGUILayout.Vector2Field("左右(Z)", ankleSwingZ, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("肩关节", EditorStyles.miniBoldLabel);
        EditorGUILayout.BeginHorizontal();
        shoulderTwist = EditorGUILayout.Vector2Field("前后(X)", shoulderTwist, GUILayout.Width(200));
        shoulderSwingY = EditorGUILayout.Vector2Field("扭转(Y)", shoulderSwingY, GUILayout.Width(200));
        shoulderSwingZ = EditorGUILayout.Vector2Field("左右(Z)", shoulderSwingZ, GUILayout.Width(200));
        EditorGUILayout.EndHorizontal();

        EditorGUILayout.Space(4);
        EditorGUILayout.LabelField("肘关节", EditorStyles.miniBoldLabel);
        elbowTwist = EditorGUILayout.Vector2Field("弯曲(X)", elbowTwist);

        // 生成选项
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("生成姿态", EditorStyles.boldLabel);
        initialKneeBendAngle = EditorGUILayout.FloatField("膝盖初始弯曲角度(度)", initialKneeBendAngle);

        // 生成选项
        EditorGUILayout.Space(15);
        EditorGUILayout.LabelField("生成选项", EditorStyles.boldLabel);
        spawnPosition = EditorGUILayout.Vector3Field("生成位置", spawnPosition);
        replaceExisting = EditorGUILayout.Toggle("替换已存在机器人", replaceExisting);

        EditorGUILayout.Space(10);

        // 配置管理按钮
        EditorGUILayout.LabelField("配置管理", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();

        if (GUILayout.Button("保存配置", GUILayout.Height(40), GUILayout.Width(150)))
        {
            SaveConfig();
        }

        if (GUILayout.Button("读取配置", GUILayout.Height(40), GUILayout.Width(150)))
        {
            LoadConfig();
        }

        if (GUILayout.Button("恢复默认", GUILayout.Height(40), GUILayout.Width(150)))
        {
            ResetToDefault();
        }

        if (GUILayout.Button("生成机器人", GUILayout.Height(40), GUILayout.Width(150)))
        {
            Generate();
        }

        EditorGUILayout.EndHorizontal();

        EditorGUILayout.EndScrollView();
    }

    private void Generate()
    {
        if (!Application.isPlaying)
            Undo.IncrementCurrentGroup();

        // 将归一化值乘以 robotHeight 得到实际尺寸
        float legLength = legLengthNormalized * robotHeight;
        float torsoWidth = torsoWidthNormalized * robotHeight;
        float torsoDepth = torsoDepthNormalized * robotHeight;
        float pelvisWidth = pelvisWidthNormalized * robotHeight;
        float armLength = armLengthNormalized * robotHeight;
        float headSize = headSizeNormalized * robotHeight;
        Vector3 footSize = footSizeNormalized * robotHeight;

        // 计算各部件高度（基于真实人体比例：7头身比例）
        float pelvisHeight = robotHeight * 0.15f; // 骨盆约占总高度15%
        float torsoHeight = robotHeight * 0.25f; // 躯干约占总高度25%
        float neckHeight = robotHeight * 0.04f; // 颈部约占总高度4%
        float headHeight = robotHeight * 0.14f; // 头部约占总高度14%

        float upperLegLength = legLength * 0.52f;
        float lowerLegLength = legLength - upperLegLength;

        float upperArmLength = armLength * 0.45f;
        float lowerArmLength = armLength - upperArmLength;

        // 腿和手臂的粗细（基于真实人体比例）
        // 成年人大腿粗细约为身高的0.06，小腿约为大腿的0.7
        float thighThickness = robotHeight * 0.06f;
        float shinThickness = thighThickness * 0.7f;

        // 成年人上臂粗细约为身高的0.05，前臂约为上臂的0.8
        float upperArmThickness = robotHeight * 0.05f;
        float lowerArmThickness = upperArmThickness * 0.8f;

        // 成年人手掌宽度约为身高的0.06
        float handWidth = robotHeight * 0.06f;
        float handThickness = handWidth * 0.6f;

        // 左右分布（乘以 robotHeight 归一化）
        float legOffsetX = robotHeight * 0.04f; // 腿间距约为身高的4%（两脚分开站立）
        float shoulderOffsetX = torsoWidth * 0.5f; // 肩关节在躯干两侧边缘

        // 实体方块尺寸（用于视觉和碰撞）
        Vector3 pelvisVisualSize = new Vector3(pelvisWidth, pelvisHeight, torsoDepth);
        Vector3 torsoVisualSize = new Vector3(torsoWidth, torsoHeight, torsoDepth);
        Vector3 neckVisualSize = new Vector3(torsoWidth * 0.3f, neckHeight, torsoDepth * 0.3f);
        Vector3 headVisualSize = new Vector3(headSize, headHeight, headSize * 0.8f);
        Vector3 upperLegVisualSize = new Vector3(thighThickness, upperLegLength, thighThickness);
        Vector3 lowerLegVisualSize = new Vector3(shinThickness, lowerLegLength, shinThickness);
        Vector3 upperArmVisualSize = new Vector3(upperArmThickness, upperArmLength, upperArmThickness);
        Vector3 lowerArmVisualSize = new Vector3(lowerArmThickness, lowerArmLength, lowerArmThickness);

        // 体积占比 -> 质量分配（基于实体方块尺寸）
        float Volume(Vector3 s) => Mathf.Max(0.000001f, s.x * s.y * s.z);
        float pelvisVol = Volume(pelvisVisualSize);
        float torsoVol = Volume(torsoVisualSize);
        float neckVol = Volume(neckVisualSize);
        float headVol = Volume(headVisualSize);
        float upperLegVol = Volume(upperLegVisualSize);
        float lowerLegVol = Volume(lowerLegVisualSize);
        float footVol = Volume(footSize);
        float upperArmVol = Volume(upperArmVisualSize);
        float lowerArmVol = Volume(lowerArmVisualSize);

        float totalVol = pelvisVol + torsoVol + neckVol + headVol +
                         2f * (upperLegVol + lowerLegVol + footVol) +
                         2f * (upperArmVol + lowerArmVol);
        float MassByVol(float vol) => totalMassKg * (vol / totalVol);

        float pelvisMass = MassByVol(pelvisVol);
        float torsoMass = MassByVol(torsoVol);
        float neckMass = MassByVol(neckVol);
        float headMass = MassByVol(headVol);
        float upperLegMass = MassByVol(upperLegVol);
        float lowerLegMass = MassByVol(lowerLegVol);
        float footMass = MassByVol(footVol);
        float upperArmMass = MassByVol(upperArmVol);
        float lowerArmMass = MassByVol(lowerArmVol);

        var existingRoot = GameObject.Find(RobotRootName);
        GameObject root;
        if (existingRoot != null)
        {
            if (!replaceExisting) return;
            // 只删除子对象,保留根节点及其上的脚本和参数
            root = existingRoot;
            root.transform.position = spawnPosition;
            // 获取所有子对象
            int childCount = root.transform.childCount;
            for (int i = childCount - 1; i >= 0; i--)
            {
                Transform child = root.transform.GetChild(i);
                if (!Application.isPlaying)
                    Undo.DestroyObjectImmediate(child.gameObject);
                else
                    Destroy(child.gameObject);
            }
        }
        else
        {
            root = new GameObject(RobotRootName);
            root.transform.position = spawnPosition;
        }
        // 根节点（从地面开始，方便观察）
        // 地面(Feet) y=0, hipLevel=legLength
        float pelvisJointY = legLength + pelvisHeight * 0.5f + footSize.y; // 骨盆关节位置

        // 为了使层级清晰：pelvis 为 Articulation 根，其余作为子链接挂上去。
        // 骨盆关节节点（无实体）
        ArticulationBody pelvis = CreateJointNode(
            root.transform,
            "Pelvis_Joint",
            new Vector3(0, pelvisJointY, 0),
            Quaternion.identity,
            pelvisMass,
            jointType: ArticulationJointType.FixedJoint,
            isRoot: true,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: false,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: 0f,
            driveForceLimit: passiveJointForceLimit,
            lowerLimitRad: 0f,
            upperLimitRad: 0f
        );

        // 在骨盆关节节点下添加视觉/碰撞实体（居中）
        CreateVisualCollider(pelvis.transform, "Pelvis_Visual", pelvisVisualSize, Vector3.zero);

        // 躯干关节节点（在骨盆顶部，使用球形关节）
        ArticulationBody torso = CreateJointNode(
            pelvis.transform,
            "Torso_Joint",
            new Vector3(0, pelvisHeight * 0.5f, 0),
            Quaternion.identity,
            torsoMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: torsoJointStiffness * globalStiffnessScale,
            driveForceLimit: passiveJointForceLimit,
            lowerLimitRad: 0f,
            upperLimitRad: 0f,
            useSphericalJoint: true,
            swingLimit: pelvisSwingY,
            swingZLimit: pelvisSwingZ,
            twistLimit: pelvisTwist
        );

        // 在躯干关节节点下添加视觉/碰撞实体
        CreateVisualCollider(torso.transform, "Torso_Visual", torsoVisualSize, new Vector3(0, torsoHeight * 0.5f, 0));

        // 头部视觉实体（直接作为躯干的一部分）
        CreateVisualCollider(torso.transform, "Head_Visual", headVisualSize, new Vector3(0, torsoHeight + headHeight * 0.5f, 0));

        // 肩关节高度（在躯干实体内）
        float shoulderAttachLocalY = torsoHeight * 0.9f; // T-pose 肩膀在躯干顶部

        // 双腿：髋关节在骨盆底部，膝盖在大腿底部，脚踝在小腿底部
        float hipJointLocalYInPelvis = -pelvisHeight * 0.5f; // 髋关节在骨盆底部中心
        float kneeJointLocalYInPelvis = -pelvisHeight * 0.5f - upperLegLength; // 膝盖关节在髋关节下方
        float ankleJointLocalYInPelvis = -pelvisHeight * 0.5f - upperLegLength - lowerLegLength; // 踝关节在膝盖下方

        // 左右命名约定：Left = -X，Right = +X（符合人体解剖学）
        CreateLeg(pelvis.transform, "Left", -legOffsetX, upperLegVisualSize, lowerLegVisualSize, footSize,
            upperLegLength, lowerLegLength, upperLegMass, lowerLegMass, footMass,
            hipJointLocalYInPelvis, kneeJointLocalYInPelvis, ankleJointLocalYInPelvis,
            angularDamping, jointFriction, passiveJointForceLimit, globalStiffnessScale,
            hipJointStiffness, kneeJointStiffness, initialKneeBendAngle, ankleJointStiffness,
            hipTwist, hipSwingY, hipSwingZ, kneeTwist, ankleTwist, ankleSwingY, ankleSwingZ);
        CreateLeg(pelvis.transform, "Right", legOffsetX, upperLegVisualSize, lowerLegVisualSize, footSize,
            upperLegLength, lowerLegLength, upperLegMass, lowerLegMass, footMass,
            hipJointLocalYInPelvis, kneeJointLocalYInPelvis, ankleJointLocalYInPelvis,
            angularDamping, jointFriction, passiveJointForceLimit, globalStiffnessScale,
            hipJointStiffness, kneeJointStiffness, initialKneeBendAngle, ankleJointStiffness,
            hipTwist, Reverse(hipSwingY), Reverse(hipSwingZ), kneeTwist, ankleTwist, Reverse(ankleSwingY), Reverse(ankleSwingZ));

        CreateArm(torso.transform, "Left", -(shoulderOffsetX + upperArmVisualSize.x * 0.5f), upperArmVisualSize, lowerArmVisualSize,
            upperArmLength, lowerArmLength, upperArmMass, lowerArmMass,
            shoulderAttachLocalY,
            angularDamping, jointFriction, passiveJointForceLimit, globalStiffnessScale,
            shoulderJointStiffness, elbowJointStiffness,
            shoulderTwist, shoulderSwingY, shoulderSwingZ, elbowTwist);

        CreateArm(torso.transform, "Right", shoulderOffsetX + upperArmVisualSize.x * 0.5f, upperArmVisualSize, lowerArmVisualSize,
            upperArmLength, lowerArmLength, upperArmMass, lowerArmMass,
            shoulderAttachLocalY,
            angularDamping, jointFriction, passiveJointForceLimit, globalStiffnessScale,
            shoulderJointStiffness, elbowJointStiffness,
            shoulderTwist, Reverse(shoulderSwingY), Reverse(shoulderSwingZ), elbowTwist);

        // 让编辑器知道场景有变化
        EditorUtility.SetDirty(root);
    }

    private static Vector2 Reverse(Vector2 v) => new Vector2(-v.y, -v.x); // 反转旋转限制（适用于左右对称的关节）
    private static void CreateLeg(
        Transform pelvis,
        string side,
        float hipOffsetX,
        Vector3 upperLegVisualSize,
        Vector3 lowerLegVisualSize,
        Vector3 footSize,
        float upperLegLength,
        float lowerLegLength,
        float upperLegMass,
        float lowerLegMass,
        float footMass,
        float hipJointLocalY,
        float kneeJointLocalY,
        float ankleJointLocalY,
        float angularDamping,
        float jointFriction,
        float driveForceLimit,
        float globalScale,
        float hipStiffness,
        float kneeStiffness,
        float initialKneeBendAngleDeg,
        float ankleStiffness,
        Vector2 hipTwist,
        Vector2 hipSwingY,
        Vector2 hipSwingZ,
        Vector2 kneeTwist,
        Vector2 ankleTwist,
        Vector2 ankleSwingY,
        Vector2 ankleSwingZ)
    {
        // 计算膝盖弯曲的角度（转换为弧度）
        float kneeBendRad = initialKneeBendAngleDeg;

        // 髋关节节点（使用球形关节，支持三个轴向旋转）
        var hipJoint = CreateJointNode(
            pelvis,
            side + "_HipJoint",
            new Vector3(hipOffsetX, hipJointLocalY, 0),
            Quaternion.identity,
            upperLegMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: hipStiffness * globalScale,
            driveForceLimit: driveForceLimit,
            lowerLimitRad: 0f,
            upperLimitRad: 0f,
            useSphericalJoint: true,
            swingLimit: hipSwingY,
            swingZLimit: hipSwingZ,
            twistLimit: hipTwist
        );

        // 设置髋关节的初始前倾角度（绕x轴旋转）
        hipJoint.transform.localRotation = Quaternion.Euler(-initialKneeBendAngleDeg * 0.5f, 0, 0);

        // 在髋关节节点下添加大腿视觉/碰撞实体
        CreateVisualCollider(hipJoint.transform, side + "_Thigh_Visual", upperLegVisualSize, new Vector3(0, -upperLegLength * 0.5f, 0));

        // 膝关节节点（只允许向前弯曲，使用RevoluteJoint）
        var kneeJoint = CreateJointNode(
            hipJoint.transform,
            side + "_KneeJoint",
            new Vector3(0, kneeJointLocalY - hipJointLocalY, 0),
            Quaternion.identity,
            lowerLegMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: kneeStiffness * globalScale,
            driveForceLimit: driveForceLimit,
            lowerLimitRad: kneeTwist.x,
            upperLimitRad: kneeTwist.y
        );

        // 设置膝关节的初始弯曲角度（绕x轴旋转）
        kneeJoint.transform.localRotation = Quaternion.Euler(initialKneeBendAngleDeg, 0, 0);

        // 在膝关节节点下添加小腿视觉/碰撞实体
        CreateVisualCollider(kneeJoint.transform, side + "_Shin_Visual", lowerLegVisualSize, new Vector3(0, -lowerLegLength * 0.5f, 0));

        // 踝关节节点（使用球形关节）
        var ankleJoint = CreateJointNode(
            kneeJoint.transform,
            side + "_AnkleJoint",
            new Vector3(0, ankleJointLocalY - kneeJointLocalY, 0),
            Quaternion.identity,
            footMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: ankleStiffness * globalScale,
            driveForceLimit: driveForceLimit,
            lowerLimitRad: 0f,
            upperLimitRad: 0f,
            useSphericalJoint: true,
            swingLimit: ankleSwingY,
            swingZLimit: ankleSwingZ,
            twistLimit: ankleTwist
        );

        // 设置踝关节的初始后仰角度（绕x轴旋转）
        ankleJoint.transform.localRotation = Quaternion.Euler(-initialKneeBendAngleDeg * 0.5f, 0, 0);

        // 在踝关节节点下添加脚板视觉/碰撞实体
        CreateVisualCollider(ankleJoint.transform, side + "_Foot_Visual", footSize, new Vector3(0, -footSize.y * 0.5f, footSize.z * 0.3f));
    }

    private static void CreateArm(
        Transform torso,
        string side,
        float shoulderOffsetX,
        Vector3 upperArmVisualSize,
        Vector3 lowerArmVisualSize,
        float upperArmLength,
        float lowerArmLength,
        float upperArmMass,
        float lowerArmMass,
        float shoulderAttachLocalY,
        float angularDamping,
        float jointFriction,
        float driveForceLimit,
        float globalScale,
        float shoulderStiffness,
        float elbowStiffness,
        Vector2 shoulderTwist,
        Vector2 shoulderSwingY,
        Vector2 shoulderSwingZ,
        Vector2 elbowTwist)
    {
        // T-pose: 左臂向左(-X)平伸，右臂向右(+X)平伸
        float armDirection = (side == "Left") ? -1f : 1f;

        // 肩关节旋转
        Quaternion shoulderRotation = Quaternion.Euler(0, 0, 0);

        // 肩关节节点（使用球形关节）
        var shoulderJoint = CreateJointNode(
            torso,
            side + "_ShoulderJoint",
            new Vector3(shoulderOffsetX, shoulderAttachLocalY, 0),
            shoulderRotation,
            upperArmMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.Euler(0, 0, 0),
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: shoulderStiffness * globalScale,
            driveForceLimit: driveForceLimit,
            lowerLimitRad: 0f,
            upperLimitRad: 0f,
            useSphericalJoint: true,
            swingLimit: shoulderSwingY,
            swingZLimit: shoulderSwingZ,
            twistLimit: shoulderTwist
        );

        // 在肩关节节点下添加上臂视觉/碰撞实体（Y轴方向，因为关节已旋转90度）
        CreateVisualCollider(shoulderJoint.transform, side + "_UpperArm_Visual", upperArmVisualSize,
            new Vector3(0, -upperArmLength * 0.5f, 0));

        // 肘关节节点（只允许向前弯曲）
        var elbowJoint = CreateJointNode(
            shoulderJoint.transform,
            side + "_ElbowJoint",
            new Vector3(0, -upperArmLength, 0),
            Quaternion.identity,
            lowerArmMass,
            jointType: ArticulationJointType.RevoluteJoint,
            isRoot: false,
            anchorLocalPos: Vector3.zero,
            anchorLocalRot: Quaternion.identity,
            configureDrive: true,
            angularDamping: angularDamping,
            jointFriction: jointFriction,
            driveStiffness: elbowStiffness * globalScale,
            driveForceLimit: driveForceLimit,
            lowerLimitRad: elbowTwist.x,
            upperLimitRad: elbowTwist.y
        );

        // 在肘关节节点下添加下臂视觉/碰撞实体（Y轴方向）
        CreateVisualCollider(elbowJoint.transform, side + "_LowerArm_Visual", lowerArmVisualSize,
            new Vector3(0, -lowerArmLength * 0.5f, 0));

        // 在手掌位置创建空节点（用于判断手的位置）
        GameObject handEmptyNode = new GameObject(side + "_Hand_Empty");
        handEmptyNode.transform.SetParent(elbowJoint.transform, false);
        handEmptyNode.transform.localPosition = new Vector3(0, -lowerArmLength, 0);
        handEmptyNode.transform.localRotation = Quaternion.identity;
        handEmptyNode.transform.localScale = Vector3.one;
    }

    /// <summary>
    /// 创建关节节点（用于物理控制，无视觉和碰撞）
    /// 所有关节节点保持 (1,1,1) 缩放
    /// </summary>
    private static ArticulationBody CreateJointNode(
        Transform parent,
        string name,
        Vector3 localPos,
        Quaternion localRot,
        float massKg,
        ArticulationJointType jointType,
        bool isRoot,
        Vector3 anchorLocalPos,
        Quaternion anchorLocalRot,
        bool configureDrive,
        float angularDamping,
        float jointFriction,
        float driveStiffness,
        float driveForceLimit,
        float lowerLimitRad,
        float upperLimitRad,
        bool useSphericalJoint = false,
        Vector2? swingLimit = null,
        Vector2? swingZLimit = null,
        Vector2? twistLimit = null)
    {
        GameObject go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localRotation = localRot;
        go.transform.localScale = Vector3.one; // 所有节点保持 (1,1,1)

        // 关节节点不需要渲染器和碰撞体
        var renderer = go.GetComponent<Renderer>();
        if (renderer != null) DestroyImmediate(renderer);
        var collider = go.GetComponent<Collider>();
        if (collider != null) DestroyImmediate(collider);

        var articulation = go.AddComponent<ArticulationBody>();

        // 物理参数
        articulation.mass = massKg;
        articulation.useGravity = true;
        articulation.linearDamping = 0f;
        articulation.angularDamping = angularDamping;
        articulation.jointFriction = jointFriction;

        // 如果使用球形关节，将关节类型设置为 SphericalJoint
        articulation.jointType = useSphericalJoint ? ArticulationJointType.SphericalJoint : jointType;
        articulation.matchAnchors = true; 

        if (!isRoot)
        {
            articulation.anchorPosition = anchorLocalPos;
            articulation.anchorRotation = anchorLocalRot;
            //articulation.parentAnchorPosition = Vector3.zero;
            //articulation.parentAnchorRotation = Quaternion.identity;
        }
        else
        {
            articulation.anchorPosition = Vector3.zero;
            articulation.anchorRotation = Quaternion.identity;
        }

        // spherical joint：配置 swing 1, swing 2, twist
        if (configureDrive && useSphericalJoint && swingLimit.HasValue)
        {
            // Swing Y (前后摆动)
            var swingYDrive = articulation.yDrive;
            if (swingLimit.Value.x == 0f && swingLimit.Value.y == 0f)
            {
                articulation.swingYLock = ArticulationDofLock.LockedMotion;
            }
            else
            {
                articulation.swingYLock = ArticulationDofLock.LimitedMotion;
                swingYDrive.lowerLimit = swingLimit.Value.x;
                swingYDrive.upperLimit = swingLimit.Value.y;
                swingYDrive.stiffness = driveStiffness;
                swingYDrive.damping = angularDamping;
                swingYDrive.forceLimit = driveForceLimit;
                articulation.yDrive = swingYDrive;
            }

            // Swing Z (左右摆动)
            var swingZDrive = articulation.zDrive;
            if (swingZLimit.Value.x == 0f && swingZLimit.Value.y == 0f)
            {
                articulation.swingZLock = ArticulationDofLock.LockedMotion;
            }
            else
            {
                articulation.swingZLock = ArticulationDofLock.LimitedMotion;
                swingZDrive.lowerLimit = swingZLimit.Value.x;
                swingZDrive.upperLimit = swingZLimit.Value.y;
                swingZDrive.stiffness = driveStiffness;
                swingZDrive.damping = angularDamping;
                swingZDrive.forceLimit = driveForceLimit;
                articulation.zDrive = swingZDrive;
            }

            // Twist (扭转)
            var twistDrive = articulation.xDrive;
            if (twistLimit.Value.x == 0f && twistLimit.Value.y == 0f)
            {
                articulation.twistLock = ArticulationDofLock.LockedMotion;
            }
            else
            {
                articulation.twistLock = ArticulationDofLock.LimitedMotion;
                twistDrive.lowerLimit = twistLimit.Value.x;
                twistDrive.upperLimit = twistLimit.Value.y;
                twistDrive.stiffness = driveStiffness;
                twistDrive.damping = angularDamping;
                twistDrive.forceLimit = driveForceLimit;
                articulation.xDrive = twistDrive;
            }
        }
        // revolute joint：直接配置 xDrive（不依赖 ArticulationDriveAxis）
        else if (configureDrive && jointType == ArticulationJointType.RevoluteJoint)
        {
            var drive = articulation.xDrive;
            articulation.twistLock = ArticulationDofLock.LimitedMotion;
            drive.lowerLimit = lowerLimitRad;
            drive.upperLimit = upperLimitRad;
            drive.stiffness = driveStiffness;
            drive.damping = angularDamping;
            drive.forceLimit = driveForceLimit;
            drive.target = 0f;
            drive.targetVelocity = 0f;
            articulation.xDrive = drive;

            // 锁定 Y 和 Z 旋转轴
            var yDrive = articulation.yDrive;
            yDrive.stiffness = 0;
            yDrive.forceLimit = driveForceLimit;
            yDrive.lowerLimit = 0;
            yDrive.upperLimit = 0;
            articulation.yDrive = yDrive;

            var zDrive = articulation.zDrive;
            zDrive.stiffness = 0;
            zDrive.forceLimit = driveForceLimit;
            zDrive.lowerLimit = 0;
            zDrive.upperLimit = 0;
            articulation.zDrive = zDrive;
        }
        if (!go.TryGetComponent(out BodyHit hit))
        {
            go.AddComponent<BodyHit>();
        }
        return articulation;
    }

    /// <summary>
    /// 创建视觉和碰撞实体（用于显示和物理碰撞）
    /// 实体可以有任意缩放，不影响关节节点的物理计算
    /// 注意：移除 ArticulationBody 是因为物理质量由关节节点控制，避免重复计算
    /// </summary>
    private static void CreateVisualCollider(
        Transform parent,
        string name,
        Vector3 size,
        Vector3 localPos)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = size; // 实体可以缩放

        // 生成用的方块不需要渲染器（训练时更干净，也避免误导）
        var renderer = go.GetComponent<Renderer>();
        renderer.enabled = true;

        // 实体不需要 ArticulationBody，只需要 Collider
        var articulation = go.GetComponent<ArticulationBody>();
        DestroyImmediate(articulation);
    }

    /// <summary>
    /// 保存当前配置到JSON文件
    /// </summary>
    private void SaveConfig()
    {
        var config = CreateConfigFromFields();
        string json = JsonUtility.ToJson(config, true);
        string configPath = GetConfigPath();

        File.WriteAllText(configPath, json, Encoding.UTF8);
        Debug.Log($"配置已保存到: {configPath}");
        EditorUtility.DisplayDialog("保存成功", $"配置已保存到:\n{configPath}", "确定");
    }

    /// <summary>
    /// 从JSON文件读取配置
    /// </summary>
    private void LoadConfig()
    {
        string configPath = GetConfigPath();

        if (!File.Exists(configPath))
        {
            EditorUtility.DisplayDialog("配置文件不存在", "未找到配置文件，请先保存配置。", "确定");
            return;
        }

        try
        {
            string json = File.ReadAllText(configPath, Encoding.UTF8);
            var config = JsonUtility.FromJson<RobotConfig>(json);

            if (config == null)
            {
                EditorUtility.DisplayDialog("读取失败", "配置文件格式错误。", "确定");
                return;
            }

            ApplyConfigToFields(config);

            Repaint();
            Debug.Log($"配置已从 {configPath} 读取成功");
            //EditorUtility.DisplayDialog("读取成功", "配置已成功加载！", "确定");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"读取配置失败: {e.Message}");
            EditorUtility.DisplayDialog("读取失败", $"读取配置时出错:\n{e.Message}", "确定");
        }
    }

    /// <summary>
    /// 恢复默认配置
    /// </summary>
    private void ResetToDefault()
    {
        bool confirm = EditorUtility.DisplayDialog("确认恢复", "确定要恢复所有参数到默认值吗？", "确定", "取消");
        if (!confirm) return;

        // 基础尺寸
        robotHeight = 1.75f;
        legLengthNormalized = 0.515f;
        torsoWidthNormalized = 0.17f;
        torsoDepthNormalized = 0.114f;
        pelvisWidthNormalized = 0.17f;
        armLengthNormalized = 0.428f;
        headSizeNormalized = 0.103f;
        footSizeNormalized = new Vector3(0.06f, 0.0286f, 0.15f);

        // 质量与阻尼
        totalMassKg = 30f;
        angularDamping = 2f;
        jointFriction = 0.5f;
        passiveJointForceLimit = 1e6f;

        // 关节刚度参数
        globalStiffnessScale = 1.0f;
        hipJointStiffness = 30000f;
        kneeJointStiffness = 20000f;
        ankleJointStiffness = 5000f;
        shoulderJointStiffness = 10000f;
        elbowJointStiffness = 5000f;
        torsoJointStiffness = 10000f;
        neckJointStiffness = 3000f;

        // 关节旋转限制 - 臀部
        pelvisSwingY = new Vector2(-45f, 45f);
        pelvisSwingZ = new Vector2(-45f, 45f);
        pelvisTwist = new Vector2(-45f, 45f);

        // 躯干
        torsoSwingY = new Vector2(-45f, 45f);
        torsoSwingZ = new Vector2(-45f, 45f);
        torsoTwist = new Vector2(-45f, 45f);

        // 颈部
        neckSwingY = new Vector2(-45f, 45f);
        neckSwingZ = new Vector2(-45f, 45f);
        neckTwist = new Vector2(-45f, 45f);

        // 髋关节
        hipTwist = new Vector2(-20f, 90f);
        hipSwingY = new Vector2(-30f, 30f);
        hipSwingZ = new Vector2(-30f, 30f);

        // 膝关节
        kneeTwist = new Vector2(0f, 150f);

        // 踝关节
        ankleTwist = new Vector2(-45f, 45f);
        ankleSwingY = new Vector2(-15f, 15f);
        ankleSwingZ = new Vector2(-20f, 20f);

        // 肩关节
        shoulderTwist = new Vector2(-180f, 180f);
        shoulderSwingY = new Vector2(-180f, 180f);
        shoulderSwingZ = new Vector2(-90f, 90f);

        // 肘关节
        elbowTwist = new Vector2(0f, 150f);

        // 生成选项
        initialKneeBendAngle = 20f;
        spawnPosition = new Vector3(0, 0.0f, 0);
        replaceExisting = true;

        Repaint();
        Debug.Log("已恢复默认配置");
        EditorUtility.DisplayDialog("恢复成功", "所有参数已恢复到默认值！", "确定");
    }

    /// <summary>
    /// 获取配置文件保存路径
    /// </summary>
    private string GetConfigPath()
    {
        // 在项目根目录下的Resources文件夹中保存配置
        string configPath = Path.Combine(Application.dataPath, "..", ConfigFolderName, ConfigFileName);
        configPath = Path.GetFullPath(configPath);

        // 确保目录存在
        string directory = Path.GetDirectoryName(configPath);
        if (!Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }

        return configPath;
    }

    /// <summary>
    /// 机器人配置数据类
    /// </summary>
    [Serializable]
    private class RobotConfig
    {
        // 基础尺寸
        public float robotHeight = 1.75f;
        public float legLengthNormalized = 0.515f;
        public float torsoWidthNormalized = 0.17f;
        public float torsoDepthNormalized = 0.114f;
        public float pelvisWidthNormalized = 0.17f;
        public float armLengthNormalized = 0.428f;
        public float headSizeNormalized = 0.103f;
        public float footSizeNormalized_x = 0.06f;
        public float footSizeNormalized_y = 0.0286f;
        public float footSizeNormalized_z = 0.15f;

        // 质量与阻尼
        public float totalMassKg = 30f;
        public float angularDamping = 2f;
        public float jointFriction = 0.5f;
        public float passiveJointForceLimit = 1e6f;

        // 关节刚度参数
        public float globalStiffnessScale = 1.0f;
        public float hipJointStiffness = 30000f;
        public float kneeJointStiffness = 20000f;
        public float ankleJointStiffness = 5000f;
        public float shoulderJointStiffness = 10000f;
        public float elbowJointStiffness = 5000f;
        public float torsoJointStiffness = 10000f;
        public float neckJointStiffness = 3000f;

        // 关节旋转限制 - 臀部
        public float pelvisSwingY_x = -45f;
        public float pelvisSwingY_y = 45f;
        public float pelvisSwingZ_x = -45f;
        public float pelvisSwingZ_y = 45f;
        public float pelvisTwist_x = -45f;
        public float pelvisTwist_y = 45f;

        // 躯干
        public float torsoSwingY_x = -45f;
        public float torsoSwingY_y = 45f;
        public float torsoSwingZ_x = -45f;
        public float torsoSwingZ_y = 45f;
        public float torsoTwist_x = -45f;
        public float torsoTwist_y = 45f;

        // 颈部
        public float neckSwingY_x = -45f;
        public float neckSwingY_y = 45f;
        public float neckSwingZ_x = -45f;
        public float neckSwingZ_y = 45f;
        public float neckTwist_x = -45f;
        public float neckTwist_y = 45f;

        // 髋关节
        public float hipTwist_x = -20f;
        public float hipTwist_y = 90f;
        public float hipSwingY_x = -30f;
        public float hipSwingY_y = 30f;
        public float hipSwingZ_x = -30f;
        public float hipSwingZ_y = 30f;

        // 膝关节
        public float kneeTwist_x = 0f;
        public float kneeTwist_y = 150f;

        // 踝关节
        public float ankleTwist_x = -45f;
        public float ankleTwist_y = 45f;
        public float ankleSwingY_x = -15f;
        public float ankleSwingY_y = 15f;
        public float ankleSwingZ_x = -20f;
        public float ankleSwingZ_y = 20f;

        // 肩关节
        public float shoulderTwist_x = -180f;
        public float shoulderTwist_y = 180f;
        public float shoulderSwingY_x = -180f;
        public float shoulderSwingY_y = 180f;
        public float shoulderSwingZ_x = -90f;
        public float shoulderSwingZ_y = 90f;

        // 肘关节
        public float elbowTwist_x = 0f;
        public float elbowTwist_y = 150f;

        // 生成姿态
        public float initialKneeBendAngle = 20f;

        // 生成选项
        public float spawnPosition_x = 0;
        public float spawnPosition_y = 0;
        public float spawnPosition_z = 0;
        public bool replaceExisting = true;
    }

    /// <summary>
    /// 从RobotConfig转换参数到窗口字段
    /// </summary>
    private void ApplyConfigToFields(RobotConfig config)
    {
        // 基础尺寸
        robotHeight = config.robotHeight;
        legLengthNormalized = config.legLengthNormalized;
        torsoWidthNormalized = config.torsoWidthNormalized;
        torsoDepthNormalized = config.torsoDepthNormalized;
        pelvisWidthNormalized = config.pelvisWidthNormalized;
        armLengthNormalized = config.armLengthNormalized;
        headSizeNormalized = config.headSizeNormalized;
        footSizeNormalized = new Vector3(config.footSizeNormalized_x, config.footSizeNormalized_y, config.footSizeNormalized_z);

        // 质量与阻尼
        totalMassKg = config.totalMassKg;
        angularDamping = config.angularDamping;
        jointFriction = config.jointFriction;
        passiveJointForceLimit = config.passiveJointForceLimit;

        // 关节刚度参数
        globalStiffnessScale = config.globalStiffnessScale;
        hipJointStiffness = config.hipJointStiffness;
        kneeJointStiffness = config.kneeJointStiffness;
        ankleJointStiffness = config.ankleJointStiffness;
        shoulderJointStiffness = config.shoulderJointStiffness;
        elbowJointStiffness = config.elbowJointStiffness;
        torsoJointStiffness = config.torsoJointStiffness;
        neckJointStiffness = config.neckJointStiffness;

        // 关节旋转限制 - 臀部
        pelvisSwingY = new Vector2(config.pelvisSwingY_x, config.pelvisSwingY_y);
        pelvisSwingZ = new Vector2(config.pelvisSwingZ_x, config.pelvisSwingZ_y);
        pelvisTwist = new Vector2(config.pelvisTwist_x, config.pelvisTwist_y);

        // 躯干
        torsoSwingY = new Vector2(config.torsoSwingY_x, config.torsoSwingY_y);
        torsoSwingZ = new Vector2(config.torsoSwingZ_x, config.torsoSwingZ_y);
        torsoTwist = new Vector2(config.torsoTwist_x, config.torsoTwist_y);

        // 颈部
        neckSwingY = new Vector2(config.neckSwingY_x, config.neckSwingY_y);
        neckSwingZ = new Vector2(config.neckSwingZ_x, config.neckSwingZ_y);
        neckTwist = new Vector2(config.neckTwist_x, config.neckTwist_y);

        // 髋关节
        hipTwist = new Vector2(config.hipTwist_x, config.hipTwist_y);
        hipSwingY = new Vector2(config.hipSwingY_x, config.hipSwingY_y);
        hipSwingZ = new Vector2(config.hipSwingZ_x, config.hipSwingZ_y);

        // 膝关节
        kneeTwist = new Vector2(config.kneeTwist_x, config.kneeTwist_y);

        // 踝关节
        ankleTwist = new Vector2(config.ankleTwist_x, config.ankleTwist_y);
        ankleSwingY = new Vector2(config.ankleSwingY_x, config.ankleSwingY_y);
        ankleSwingZ = new Vector2(config.ankleSwingZ_x, config.ankleSwingZ_y);

        // 肩关节
        shoulderTwist = new Vector2(config.shoulderTwist_x, config.shoulderTwist_y);
        shoulderSwingY = new Vector2(config.shoulderSwingY_x, config.shoulderSwingY_y);
        shoulderSwingZ = new Vector2(config.shoulderSwingZ_x, config.shoulderSwingZ_y);

        // 肘关节
        elbowTwist = new Vector2(config.elbowTwist_x, config.elbowTwist_y);

        // 生成姿态
        initialKneeBendAngle = config.initialKneeBendAngle;

        // 生成选项
        spawnPosition = new Vector3(config.spawnPosition_x, config.spawnPosition_y, config.spawnPosition_z);
        replaceExisting = config.replaceExisting;
    }

    /// <summary>
    /// 从窗口字段转换参数到RobotConfig
    /// </summary>
    private RobotConfig CreateConfigFromFields()
    {
        return new RobotConfig
        {
            // 基础尺寸
            robotHeight = robotHeight,
            legLengthNormalized = legLengthNormalized,
            torsoWidthNormalized = torsoWidthNormalized,
            torsoDepthNormalized = torsoDepthNormalized,
            pelvisWidthNormalized = pelvisWidthNormalized,
            armLengthNormalized = armLengthNormalized,
            headSizeNormalized = headSizeNormalized,
            footSizeNormalized_x = footSizeNormalized.x,
            footSizeNormalized_y = footSizeNormalized.y,
            footSizeNormalized_z = footSizeNormalized.z,

            // 质量与阻尼
            totalMassKg = totalMassKg,
            angularDamping = angularDamping,
            jointFriction = jointFriction,
            passiveJointForceLimit = passiveJointForceLimit,

            // 关节刚度参数
            globalStiffnessScale = globalStiffnessScale,
            hipJointStiffness = hipJointStiffness,
            kneeJointStiffness = kneeJointStiffness,
            ankleJointStiffness = ankleJointStiffness,
            shoulderJointStiffness = shoulderJointStiffness,
            elbowJointStiffness = elbowJointStiffness,
            torsoJointStiffness = torsoJointStiffness,
            neckJointStiffness = neckJointStiffness,

            // 关节旋转限制 - 臀部
            pelvisSwingY_x = pelvisSwingY.x,
            pelvisSwingY_y = pelvisSwingY.y,
            pelvisSwingZ_x = pelvisSwingZ.x,
            pelvisSwingZ_y = pelvisSwingZ.y,
            pelvisTwist_x = pelvisTwist.x,
            pelvisTwist_y = pelvisTwist.y,

            // 躯干
            torsoSwingY_x = torsoSwingY.x,
            torsoSwingY_y = torsoSwingY.y,
            torsoSwingZ_x = torsoSwingZ.x,
            torsoSwingZ_y = torsoSwingZ.y,
            torsoTwist_x = torsoTwist.x,
            torsoTwist_y = torsoTwist.y,

            // 颈部
            neckSwingY_x = neckSwingY.x,
            neckSwingY_y = neckSwingY.y,
            neckSwingZ_x = neckSwingZ.x,
            neckSwingZ_y = neckSwingZ.y,
            neckTwist_x = neckTwist.x,
            neckTwist_y = neckTwist.y,

            // 髋关节
            hipTwist_x = hipTwist.x,
            hipTwist_y = hipTwist.y,
            hipSwingY_x = hipSwingY.x,
            hipSwingY_y = hipSwingY.y,
            hipSwingZ_x = hipSwingZ.x,
            hipSwingZ_y = hipSwingZ.y,

            // 膝关节
            kneeTwist_x = kneeTwist.x,
            kneeTwist_y = kneeTwist.y,

            // 踝关节
            ankleTwist_x = ankleTwist.x,
            ankleTwist_y = ankleTwist.y,
            ankleSwingY_x = ankleSwingY.x,
            ankleSwingY_y = ankleSwingY.y,
            ankleSwingZ_x = ankleSwingZ.x,
            ankleSwingZ_y = ankleSwingZ.y,

            // 肩关节
            shoulderTwist_x = shoulderTwist.x,
            shoulderTwist_y = shoulderTwist.y,
            shoulderSwingY_x = shoulderSwingY.x,
            shoulderSwingY_y = shoulderSwingY.y,
            shoulderSwingZ_x = shoulderSwingZ.x,
            shoulderSwingZ_y = shoulderSwingZ.y,

            // 肘关节
            elbowTwist_x = elbowTwist.x,
            elbowTwist_y = elbowTwist.y,

            // 生成姿态
            initialKneeBendAngle = initialKneeBendAngle,

            // 生成选项
            spawnPosition_x = spawnPosition.x,
            spawnPosition_y = spawnPosition.y,
            spawnPosition_z = spawnPosition.z,
            replaceExisting = replaceExisting
        };
    }
}

