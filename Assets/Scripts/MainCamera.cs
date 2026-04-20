using UnityEngine;

/// <summary>
/// Controls the camera on a spherical orbit around the current surface
/// </summary>
public class MainCamera : MonoBehaviour {
    /// <summary>
    /// Enables time-based automatic rotation.
    /// </summary>
    public bool shouldRotate;

    /// <summary>
    /// World-space point observed by the camera.
    /// </summary>
    public Vector3 target = Vector3.zero;

    /// <summary>
    /// Angular speed used for automatic rotation
    /// </summary>
    public float rotationSpeed = 0.001f;

    /// <summary>
    /// Radius of the camera orbit
    /// </summary>
    public float cameraDistance = 5f;

    /// <summary>
    /// Lower bound for <see cref="cameraDistance"/>.
    /// </summary>
    public float minCameraDistance = 1f;

    /// <summary>
    /// Upper bound for <see cref="cameraDistance"/>
    /// </summary>
    public float maxCameraDistance = 20f;

    /// <summary>
    /// Minimum polar angle for automatic motion, in degrees.
    /// </summary>
    public float minThetaDeg = 40f;

    /// <summary>
    /// Maximum polar angle for automatic motion, in degrees
    /// </summary>
    public float maxThetaDeg = 140f;

    /// <summary>
    /// Manual polar angle
    /// </summary>
    public float theta = Mathf.PI / 2;

    /// <summary>
    /// Manual azimuth angle
    /// </summary>
    public float phi;

    void Update() {
        if (shouldRotate) {
            float t = (Mathf.Sin(Time.time * rotationSpeed) + 1) * 0.5f;
            RotateCamera(Mathf.Lerp(minThetaDeg * Mathf.Deg2Rad, maxThetaDeg * Mathf.Deg2Rad, t), Time.time * rotationSpeed);
        } else {
            RotateCamera(theta, phi);
        }
    }

    /// <summary>
    /// Maps spherical coordinates to Cartesian coordinates and points the camera to <see cref="target"/>
    /// </summary>
    /// <param name="thetaRad">Polar angle in radians</param>
    /// <param name="phiRad">Azimuth angle in radians</param>
    void RotateCamera(float thetaRad, float phiRad) {
        float x = Mathf.Sin(thetaRad) * Mathf.Cos(phiRad) * cameraDistance;
        float y = Mathf.Cos(thetaRad) * cameraDistance;
        float z = Mathf.Sin(thetaRad) * Mathf.Sin(phiRad) * cameraDistance;
        transform.position = new Vector3(x, y, z);
        transform.LookAt(target);
    }
}