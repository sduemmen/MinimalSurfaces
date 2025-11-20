using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    public Vector3 target = Vector3.zero;
    public float rotationSpeed = 0.001f;
    public float cameraDistance = 5f;

    public float minThetaDeg = 40f;
    public float maxThetaDeg = 140f;

    void Update() {
        float t = (Mathf.Sin(Time.time * rotationSpeed) + 1) * 0.5f;
        float theta = Mathf.Lerp(minThetaDeg * Mathf.Deg2Rad, maxThetaDeg * Mathf.Deg2Rad, t);
        float phi = Time.time * rotationSpeed;

        float x = Mathf.Sin(theta) * Mathf.Cos(phi) * cameraDistance;
        float y = Mathf.Cos(theta) * cameraDistance;
        float z = Mathf.Sin(theta) * Mathf.Sin(phi) * cameraDistance;
        transform.position = new Vector3(x, y, z);
        transform.LookAt(target);
    }
}