using UnityEngine;

public class MainCamera : MonoBehaviour {
    public bool shouldRotate;
    public Vector3 target = Vector3.zero;
    public float rotationSpeed = 0.001f;
    public float cameraDistance = 5f;

    public float minCameraDistance = 1f;
    public float maxCameraDistance = 20f;

    public float minThetaDeg = 40f;
    public float maxThetaDeg = 140f;

    public float theta = Mathf.PI / 2;
    public float phi;

    void Update() {
        if (shouldRotate) {
            float t = (Mathf.Sin(Time.time * rotationSpeed) + 1) * 0.5f;
            RotateCamera(thetaRad: Mathf.Lerp(minThetaDeg * Mathf.Deg2Rad, maxThetaDeg * Mathf.Deg2Rad, t), phiRad: Time.time * rotationSpeed);
        } else {
            RotateCamera(theta, phi);
        }
    }

    void RotateCamera(float thetaRad, float phiRad) {
        float x = Mathf.Sin(thetaRad) * Mathf.Cos(phiRad) * cameraDistance;
        float y = Mathf.Cos(thetaRad) * cameraDistance;
        float z = Mathf.Sin(thetaRad) * Mathf.Sin(phiRad) * cameraDistance;
        transform.position = new Vector3(x, y, z);
        transform.LookAt(target);
    }
}