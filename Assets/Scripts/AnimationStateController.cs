using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 动画状态控制器 - 通过单个浮点值控制多个动画的混合
/// 自动从 Animator 获取动画信息
/// </summary>
[RequireComponent(typeof(Animator))]
public class AnimationStateController : MonoBehaviour
{
    [Header("Blend Control")]
    [Tooltip("混合值：0~1控制clip1-clip2, 1~2控制clip2-clip3, 依次类推")]
    [SerializeField] private float blendValue = 0f;

    [Header("Animation Clips")]
    [Tooltip("手动配置BlendTree中的所有动画片段顺序")]
    [SerializeField] private AnimationClip[] clips;

    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = true;

    private Animator animator;
    /// <summary>
    /// 设置/获取混合值
    /// </summary>
    public float BlendValue
    {
        get => blendValue;
        set
        {
            blendValue = value;
            animator?.SetFloat("Blend", blendValue);
        }
    }

    /// <summary>
    /// 当前正在播放的动画片段
    /// </summary>
    public AnimationClip CurrentClip => GetCurrentClip();

    /// <summary>
    /// 当前动画的平均速度
    /// </summary>
    public Vector3 AnimationVelocity => GetCurrentAnimationVelocity();

    /// <summary>
    /// 当前动画的速度标量值
    /// </summary>
    public float AnimationSpeed => AnimationVelocity.magnitude;

    /// <summary>
    /// 当前动画的XZ平面速度
    /// </summary>
    public float AnimationHorizontalSpeed => new Vector3(AnimationVelocity.x, 0, AnimationVelocity.z).magnitude;

    /// <summary>
    /// 当前动画名称
    /// </summary>
    public string CurrentClipName => CurrentClip != null ? CurrentClip.name : "";

    Transform orientationCube;
    public Transform hips;
    private void Awake()
    {
        animator = GetComponent<Animator>();
        hips = animator.GetBoneTransform(HumanBodyBones.Hips);
        if (orientationCube == null)
        {
            orientationCube = new GameObject("OrientationCube").transform;
        }
    }

    private void Start()
    {
        if (animator != null)
            animator.SetFloat("Blend", blendValue);
    }

    private void Update()
    {
        if (animator != null)
            animator.SetFloat("Blend", blendValue);
    }

    public void GetDiff(Transform orientationCube, Animator animator, HumanBodyBones[] allJoints, out float disDiff, out float angleDiff)
    {
        this.orientationCube.position = hips.position;
        disDiff = 0;
        angleDiff = 0;
        foreach (var item in allJoints)
        {
            //ai控制的骨架状态
            Transform transform = animator.GetBoneTransform(item);
            Vector3 localPoint = orientationCube.InverseTransformPoint(transform.position);
            Quaternion localRot = Quaternion.Inverse(transform.rotation) * orientationCube.rotation;
            //动画控制的骨架状态
            transform = this.animator.GetBoneTransform(item);
            Vector3 thislocalPoint = this.orientationCube.InverseTransformPoint(transform.position);
            Quaternion thislocalRot = Quaternion.Inverse(transform.rotation) * this.orientationCube.rotation;

            disDiff += Vector3.Distance(localPoint, thislocalPoint);
            angleDiff += Quaternion.Angle(localRot, thislocalRot);
        }
        disDiff /= allJoints.Length;
    }

    /// <summary>
    /// 获取当前播放的动画片段
    /// </summary>
    public AnimationClip GetCurrentClip()
    {
        if (animator == null) return null;

        var clipInfos = animator.GetCurrentAnimatorClipInfo(0);
        if (clipInfos != null && clipInfos.Length > 0)
        {
            return clipInfos[0].clip;
        }
        return null;
    }

    /// <summary>
    /// 获取基于blendValue的混合速度
    /// blendValue: 0~1控制clip1-clip2, 1~2控制clip2-clip3, 以此类推
    /// </summary>
    public Vector3 GetCurrentAnimationVelocity()
    {
        if (clips == null || clips.Length == 0)
            return Vector3.zero;

        float interval = blendValue;
        int count = clips.Length;

        // 边界处理
        if (interval <= 0)
            return clips[0].averageSpeed;

        if (interval >= count - 1)
            return clips[count - 1].averageSpeed;

        // 在相邻动画之间插值
        int index = Mathf.FloorToInt(interval);
        float localBlend = interval - index;

        return Vector3.Lerp(clips[index].averageSpeed, clips[index + 1].averageSpeed, localBlend);
    }

    /// <summary>
    /// 根据速度自动设置 Blend 值
    /// </summary>
    public void SetRandomBlend()
    {
        if (clips == null || clips.Length <= 1)
            return;

        BlendValue = Random.Range(0f, clips.Length - 1);
    }

#if UNITY_EDITOR
    private void OnGUI()
    {
        if (!showDebugInfo) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("<color=yellow>=== Animation State Controller ===</color>");
        GUILayout.Label($"Blend Value: {blendValue:F2}");

        GUILayout.Label($"<color=cyan>--- Current Animation ---</color>");
        GUILayout.Label($"Clip: {CurrentClipName}");
        GUILayout.Label($"Velocity: {AnimationVelocity}");
        GUILayout.Label($"Speed: {AnimationSpeed:F2}");
        GUILayout.Label($"Horizontal: {AnimationHorizontalSpeed:F2}");

        GUILayout.EndArea();
    }
#endif
}
