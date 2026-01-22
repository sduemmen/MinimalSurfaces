using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;

public static class RuntimeUI {
    static readonly Dictionary<string, string> _textBuffer = new Dictionary<string, string>();

    public static bool DrawObjectFields(object target) {
        if (target == null) return false;

        bool changedAny = false;

        Type type = target.GetType();
        FieldInfo[] fields = type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .Where(f => !f.IsStatic)
            .Where(f => f.IsPublic || Attribute.IsDefined(f, typeof(SerializeField)))
            .Where(f => !Attribute.IsDefined(f, typeof(NonSerializedAttribute)))
            .ToArray();

        foreach (FieldInfo field in fields) {
            if (DrawField(target, field)) {
                changedAny = true;
            }
        }

        return changedAny;
    }

    static bool DrawField(object target, FieldInfo field) {
        Type t = field.FieldType;
        string label = ObjectNames.NicifyVariableName(field.Name);

        object value = field.GetValue(target);
        bool changed = false;

        GUILayout.BeginHorizontal();
        GUILayout.Label(label, GUILayout.Width(220f));

        try {
            if (t == typeof(int)) {
                int v = (int)value;
                string key = MakeKey(target, field);

                if (!_textBuffer.TryGetValue(key, out string s)) {
                    s = v.ToString(CultureInfo.InvariantCulture);
                }

                s = GUILayout.TextField(s, GUILayout.Width(120f));
                _textBuffer[key] = s;

                if (int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed)) {
                    if (parsed != v) {
                        field.SetValue(target, parsed);
                        changed = true;
                    }
                }
            } else if (t == typeof(float)) {
                float v = (float)value;
                string key = MakeKey(target, field);

                if (!_textBuffer.TryGetValue(key, out string s)) {
                    s = v.ToString(CultureInfo.InvariantCulture);
                }

                s = GUILayout.TextField(s, GUILayout.Width(120f));
                _textBuffer[key] = s;

                string normalized = s.Replace(',', '.');

                if (float.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)) {
                    if (!Mathf.Approximately(parsed, v)) {
                        field.SetValue(target, parsed);
                        changed = true;
                    }
                }
            } else if (t == typeof(bool)) {
                bool v = (bool)value;
                bool next = GUILayout.Toggle(v, "");
                if (next != v) {
                    field.SetValue(target, next);
                    changed = true;
                }
            } else if (t == typeof(string)) {
                string v = (string)value ?? "";
                string next = GUILayout.TextField(v, GUILayout.Width(220f));
                if (!string.Equals(next, v, StringComparison.Ordinal)) {
                    field.SetValue(target, next);
                    changed = true;
                }
            } else if (t.IsEnum) {
                if (DrawEnumPopup(target, field, (Enum)value)) {
                    changed = true;
                }
            } else if (t == typeof(Vector3)) {
                Vector3 v = (Vector3)value;
                Vector3 next = DrawVector3(target, field, v);
                if (next != v) {
                    field.SetValue(target, next);
                    changed = true;
                }
            } else {
                GUILayout.Label($"({t.Name})", GUILayout.Width(120f));
            }
        } catch (Exception) {
            GUILayout.Label("ERR", GUILayout.Width(40f));
        } finally {
            GUILayout.EndHorizontal();
        }

        return changed;
    }

    static string MakeKey(object target, FieldInfo field) {
        int id = target is UnityEngine.Object uo ? uo.GetInstanceID() : target.GetHashCode();
        return $"{id}:{target.GetType().FullName}:{field.Name}";
    }

    static bool DrawEnumPopup(object target, FieldInfo field, Enum current) {
        string[] names = Enum.GetNames(field.FieldType);
        int idx = Array.IndexOf(names, current.ToString());
        int prevIdx = idx;

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("<", GUILayout.Width(24f))) {
            idx = (idx - 1 + names.Length) % names.Length;
        }

        GUILayout.Label(names[Mathf.Clamp(idx, 0, names.Length - 1)], GUILayout.Width(160f));
        if (GUILayout.Button(">", GUILayout.Width(24f))) {
            idx = (idx + 1) % names.Length;
        }

        GUILayout.EndHorizontal();

        if (idx != prevIdx) {
            Enum next = (Enum)Enum.Parse(field.FieldType, names[idx]);
            field.SetValue(target, next);
            return true;
        }

        return false;
    }

    static Vector3 DrawVector3(object target, FieldInfo parentField, Vector3 v) {
        GUILayout.BeginVertical();

        v.x = DrawFloatInline(target, parentField, "X", v.x);
        v.y = DrawFloatInline(target, parentField, "Y", v.y);
        v.z = DrawFloatInline(target, parentField, "Z", v.z);

        GUILayout.EndVertical();
        return v;
    }

    static float DrawFloatInline(object target, FieldInfo parentField, string axis, float value) {
        GUILayout.BeginHorizontal();
        GUILayout.Label(axis, GUILayout.Width(18f));

        string key = $"{MakeKey(target, parentField)}:{axis}";
        if (!_textBuffer.TryGetValue(key, out string s)) {
            s = value.ToString(CultureInfo.InvariantCulture);
        }

        s = GUILayout.TextField(s, GUILayout.Width(120f));
        _textBuffer[key] = s;

        string normalized = s.Replace(',', '.');
        if (float.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)) {
            value = parsed;
        }

        GUILayout.EndHorizontal();
        return value;
    }
}