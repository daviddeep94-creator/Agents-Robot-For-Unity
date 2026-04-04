#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(MinMaxSliderAttribute))]
public class MinMaxSliderDrawer : PropertyDrawer
{
    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        MinMaxSliderAttribute attr = (MinMaxSliderAttribute)attribute;

        SerializedProperty minProp = property.FindPropertyRelative("min");
        SerializedProperty maxProp = property.FindPropertyRelative("max");

        if (minProp == null || maxProp == null)
        {
            EditorGUI.LabelField(position, label.text, "Use FloatRange with MinMaxSlider");
            return;
        }

        float min = minProp.floatValue;
        float max = maxProp.floatValue;

        EditorGUI.BeginProperty(position, label, property);

        position = EditorGUI.PrefixLabel(position, label);

        float fieldWidth = 50f;
        float spacing = 5f;

        Rect minFieldRect = new Rect(position.x, position.y, fieldWidth, position.height);
        Rect maxFieldRect = new Rect(position.x + position.width - fieldWidth, position.y, fieldWidth, position.height);
        Rect sliderRect = new Rect(
            position.x + fieldWidth + spacing,
            position.y,
            position.width - fieldWidth * 2 - spacing * 2,
            position.height
        );

        // 输入框
        min = EditorGUI.FloatField(minFieldRect, min);
        max = EditorGUI.FloatField(maxFieldRect, max);

        // 滑条
        EditorGUI.MinMaxSlider(sliderRect, ref min, ref max, attr.minLimit, attr.maxLimit);

        // 约束（关键）
        min = Mathf.Clamp(min, attr.minLimit, max);
        max = Mathf.Clamp(max, min, attr.maxLimit);

        minProp.floatValue = min;
        maxProp.floatValue = max;

        EditorGUI.EndProperty();
    }
}
#endif