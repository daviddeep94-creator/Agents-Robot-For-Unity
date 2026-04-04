using UnityEngine;

public class MinMaxSliderAttribute : PropertyAttribute
{
    public float minLimit;
    public float maxLimit;

    public MinMaxSliderAttribute(float min, float max)
    {
        this.minLimit = min;
        this.maxLimit = max;
    }
}