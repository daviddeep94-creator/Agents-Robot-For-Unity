using UnityEngine;

[System.Serializable]
public struct FloatRange
{
    public float min;
    public float max;

    public FloatRange(float min, float max)
    {
        this.min = min;
        this.max = max;
    }

    /// <summary>
    /// 随机取一个区间内的值
    /// </summary>
    public float RandomValue()
    {
        return Random.Range(min, max);
    }

    /// <summary>
    /// 返回区间长度
    /// </summary>
    public float Length()
    {
        return max - min;
    }

    /// <summary>
    /// 限制数值在区间内
    /// </summary>
    public float Clamp(float value)
    {
        return Mathf.Clamp(value, min, max);
    }

    public override string ToString()
    {
        return $"[{min}, {max}]";
    }
}