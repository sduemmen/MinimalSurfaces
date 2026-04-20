using Meshes;
using System;
using System.Globalization;
using System.IO;
using System.Linq;
using Solver;
using UnityEngine;

/// <summary>
/// Logs error against the analytic catenoid for cylinder meshes.
/// </summary>
public class CatenoidAccuracyLogger {
    float _a;
    bool _initialized;
    StreamWriter _maxErrorWriter;
    StreamWriter _meanErrorWriter;

    double f(double a, double h) {
        return a * Math.Cosh(h / a);
    }

    double g(double a, double r, double h) {
        return f(a, h) - r;
    }

    double g_prime(double a, double h) {
        return Math.Cosh(h / a) - h / a * Math.Sinh(h / a);
    }

    /// <summary>
    /// Initializes log files and determines the catenoid parameter.
    /// </summary>
    /// <param name="r">Boundary radius.</param>
    /// <param name="h">Half-height.</param>
    /// <param name="solver">Solver used for filename annotation.</param>
    public void Initialize(float r, float h, MinimalSurfaceSolverBase solver) {
        _initialized = false;

        const float maxRatio = 0.66274342f;
        if (h / r > maxRatio) return;

        float? a1 = Newton(r, h, h * 0.5f);
        float? a2 = Newton(r, h, r);

        if (a1.HasValue && a2.HasValue) {
            _a = Mathf.Max(a1.Value, a2.Value);
        } else if (a1.HasValue) {
            _a = a1.Value;
        } else if (a2.HasValue) {
            _a = a2.Value;
        } else {
            return;
        }

        string dir = Path.Combine(Application.dataPath, "CatenoidAccuracyLogs");
        if (!Directory.Exists(dir)) {
            Directory.CreateDirectory(dir);
        }

        string filePath = Path.Combine(dir, $"{solver.GetType()}_{DateTime.Now:yyyyMMdd_HHmmss}");
        _maxErrorWriter = new StreamWriter(filePath + "_MaxError.csv", false);
        _meanErrorWriter = new StreamWriter(filePath + "_MeanError.csv", false);
        _maxErrorWriter.WriteLine("Iteration,maxError");
        _meanErrorWriter.WriteLine("Iteration,meanError");

        _initialized = true;
    }

    /// <summary>
    /// Writes one log row for maximum and mean error.
    /// </summary>
    /// <param name="meshData">Current mesh state.</param>
    /// <param name="k">Iteration index.</param>
    public void Step(MeshData meshData, int k) {
        if (!_initialized) return;

        WriteMaxError(meshData, k);
        WriteMeanError(meshData, k);
    }

    /// <summary>
    /// Writes the current maximum absolute error over all vertices.
    /// </summary>
    /// <param name="meshData">Current mesh state.</param>
    /// <param name="k">Iteration index.</param>
    void WriteMaxError(MeshData meshData, int k) {
        double maxError = meshData.vertices.Select(p => CalculateError(p)).Max();

        _maxErrorWriter.WriteLine($"{k},{maxError.ToString(CultureInfo.InvariantCulture)}");
        _maxErrorWriter.Flush();
    }

    /// <summary>
    /// Writes the current mean absolute error over all vertices.
    /// </summary>
    /// <param name="meshData">Current mesh state.</param>
    /// <param name="k">Iteration index.</param>
    void WriteMeanError(MeshData meshData, int k) {
        double meanError = meshData.vertices.Sum(p => CalculateError(p)) / meshData.vertices.Length;

        _meanErrorWriter.WriteLine($"{k},{meanError.ToString(CultureInfo.InvariantCulture)}");
        _meanErrorWriter.Flush();
    }

    /// <summary>
    /// Computes radial deviation of one vertex from the analytic catenoid at matching height.
    /// </summary>
    /// <param name="p">Vertex position.</param>
    /// <returns>Absolute radial error.</returns>
    double CalculateError(Vector3 p) {
        double distance = Math.Sqrt(p.x * p.x + p.z * p.z);
        double catenoid = _a * Math.Cosh(p.y / _a);
        return Math.Abs(distance - catenoid);
    }

    /// <summary>
    /// Flushes and closes all file streams.
    /// </summary>
    public void Close() {
        if (!_initialized) return;

        _maxErrorWriter.Flush();
        _maxErrorWriter.Dispose();
        _meanErrorWriter.Flush();
        _meanErrorWriter.Dispose();
        _initialized = false;
    }

    /// <summary>
    /// Solves <c>g(a,r,h)=0</c> with Newton iteration.
    /// </summary>
    /// <param name="r">radius</param>
    /// <param name="h">Half-height</param>
    /// <param name="a0">Initial guess</param>
    /// <param name="tol">Stopping tolerance</param>
    /// <param name="maxIter">maximum number of iterations</param>
    /// <returns>a or <see langword="null"/> if iteration fails.</returns>
    float? Newton(float r, float h, float a0, float tol = 1e-10f, int maxIter = 100) {
        double a = a0;

        for (int i = 0; i < maxIter; i++) {
            double ga = g(a, r, h);
            double gpa = g_prime(a, h);

            if (Math.Abs(gpa) < 1e-14) {
                return null;
            }

            double aNew = a - ga / gpa;

            if (Math.Abs(aNew - a) < tol) {
                return (float)aNew;
            }

            a = aNew;
        }

        return null;
    }
}