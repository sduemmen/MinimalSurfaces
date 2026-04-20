using System.Collections;
using Meshes;
using Solver;
using UnityEngine;

/// <summary>
/// Controls mesh generation, solver stepping, and material visualization.
/// </summary>
public class MinimalSurface : MonoBehaviour {
    /// <summary>
    /// Material preset used to render the current surface.
    /// </summary>
    public enum SurfaceMaterialMode {
        /// <summary>
        /// Lit material with lighting.
        /// </summary>
        Lit,

        /// <summary>
        /// Unlit material without lighting.
        /// </summary>
        Unlit,

        /// <summary>
        /// Vertex-color visualization of signed mean curvature.
        /// </summary>
        MeanCurvature,

        /// <summary>
        /// Soap film-like material.
        /// </summary>
        Soap
    }

    [SerializeField] MeshFilter _meshFilter;
    [SerializeField] MeshRenderer _meshRenderer;
    [SerializeField] Material _litMaterial;
    [SerializeField] Material _unlitMaterial;
    [SerializeField] Material _meanCurvatureMaterial;
    [SerializeField] Material _soapMaterial;
    [SerializeField] Material _wireframeMaterial;
    [SerializeField] public SurfaceMaterialMode materialMode = SurfaceMaterialMode.MeanCurvature;

    /// <summary>
    /// If enabled, overlays a wireframe pass on top of the base material.
    /// </summary>
    public bool showWireframe = true;

    /// <summary>
    /// solver iterations per second
    /// </summary>
    public int stepsPerSecond = 60;

    /// <summary>
    /// Maximum number of iterations before stopping.
    /// </summary>
    public int maxSteps = 5000;

    /// <summary>
    /// Lower bound for <see cref="stepsPerSecond"/>.
    /// </summary>
    public int minStepsPerSecond = 1;

    /// <summary>
    /// Upper bound for <see cref="stepsPerSecond"/>.
    /// </summary>
    public int maxStepsPerSecond = 240;

    /// <summary>
    /// Lower bound for <see cref="maxSteps"/>.
    /// </summary>
    public int minMaxSteps = 100;

    /// <summary>
    /// Upper bound for <see cref="maxSteps"/>.
    /// </summary>
    public int maxMaxSteps = 100000;

    MeshData _currentMeshData;
    Coroutine _currentSolveRoutine;
    MinimalSurfaceSolverBase _currentSolver;
    int _convergedAfterSteps;
    int _currentStepCount;
    bool _isPaused;
    bool _isConverged;
    MeshGeneratorBase _currentGenerator;
    CatenoidAccuracyLogger _catenoidAccuracyLogger = new CatenoidAccuracyLogger();

    public float StepInterval {
        get => 1f / stepsPerSecond;
    }

    public int ConvergedAfterSteps {
        get => _convergedAfterSteps;
    }

    public int CurrentStepCount {
        get => _currentStepCount;
    }

    public bool IsPaused {
        get => _isPaused;
    }

    public bool IsConverged {
        get => _isConverged;
    }

    public bool IsSolving {
        get => _currentSolveRoutine != null;
    }

    public MeshData CurrentMeshData {
        get => _currentMeshData;
    }

    public Mesh CurrentMesh {
        get => _currentMeshData?.mesh;
    }

    public SurfaceMaterialMode MaterialMode {
        get => materialMode;
        set => SetMaterialMode(value);
    }

    void Awake() {
        if (_meshFilter == null) {
            _meshFilter = GetComponent<MeshFilter>();
        }

        if (_meshRenderer == null) {
            _meshRenderer = GetComponent<MeshRenderer>();
        }

        ApplyMaterial();
    }

    /// <summary>
    /// Toggles pause state of the active solver loop.
    /// </summary>
    public void TogglePause() {
        _isPaused = !_isPaused;
    }

    /// <summary>
    /// Generates a new mesh using the provided generator and assigns it to the surface.
    /// </summary>
    /// <param name="generator">Generator instance used to create mesh data.</param>
    public void GenerateMesh(MeshGeneratorBase generator) {
        if (generator == null) return;

        _currentMeshData = generator.Generate();
        if (_currentMeshData != null && _currentMeshData.mesh != null) {
            _meshFilter.sharedMesh = _currentMeshData.mesh;
        }
    }

    /// <summary>
    /// Starts a new solve
    /// </summary>
    /// <param name="generator">Mesh generator to use</param>
    /// <param name="solver">Solver to use</param>
    public void StartSolve(MeshGeneratorBase generator, MinimalSurfaceSolverBase solver) {
        CancelSolve();

        if (generator == null || solver == null) return;

        // generate mesh if not already present
        if (_currentMeshData == null || _currentMeshData.mesh == null) {
            GenerateMesh(generator);
        }

        // if no mesh was generated, abort
        if (_currentMeshData == null || _currentMeshData.mesh == null) return;

        _currentStepCount = 0;
        _isPaused = false;
        _isConverged = false;
        _currentSolver = solver;
        _currentGenerator = generator;
        _currentSolver.Initialize(_currentMeshData, this);

#if UNITY_EDITOR // only log catenoid accuracy in editor
        if (_currentGenerator is HoledCylinderGenerator cylinderGenerator) {
            float r = cylinderGenerator.radius;
            float h = cylinderGenerator.height * 0.5f;
            _catenoidAccuracyLogger.Initialize(r, h, _currentSolver);
            _catenoidAccuracyLogger.Step(_currentMeshData, 0);
        }
#endif

        _currentSolveRoutine = StartCoroutine(RunSolve());
    }

    /// <summary>
    /// Coroutine that advances the active solver at the configured step interval.
    /// </summary>
    /// <returns>IEnumerator used for coroutine scheduling</returns>
    IEnumerator RunSolve() {
        while (_currentStepCount < maxSteps && !_isConverged) {
            if (!_isPaused) {
                _isConverged = _currentSolver.Step(_currentMeshData, this);
                _currentStepCount++;
#if UNITY_EDITOR
                if (_currentGenerator is HoledCylinderGenerator) {
                    _catenoidAccuracyLogger.Step(_currentMeshData, _currentStepCount);
                }
#endif
            }

            yield return new WaitForSeconds(StepInterval);
        }

        if (_currentStepCount >= maxMaxSteps) {
            _isConverged = true;
        }
#if UNITY_EDITOR
        _catenoidAccuracyLogger.Close();
#endif
        _convergedAfterSteps = _currentStepCount >= maxMaxSteps ? maxSteps : _currentStepCount;
        _currentSolveRoutine = null;
    }

    /// <summary>
    /// Replaces the active solver and continues solving from the current mesh state.
    /// </summary>
    /// <param name="solver">New solver instance.</param>
    public void SwitchSolver(MinimalSurfaceSolverBase solver) {
        if (solver == null) return;

        _currentSolver = solver;
        _currentStepCount = 0;
        _isConverged = false;

        _currentSolver.Initialize(_currentMeshData, this);

        if (!IsSolving) {
            _currentSolveRoutine = StartCoroutine(RunSolve());
        }
    }

    /// <summary>
    /// Stops the current solve and resets state.
    /// </summary>
    public void CancelSolve() {
        if (!IsSolving) return;

        StopCoroutine(_currentSolveRoutine);
        _currentSolveRoutine = null;
        _isPaused = false;
        _isConverged = false;
#if UNITY_EDITOR
        _catenoidAccuracyLogger.Close();
#endif
    }

    /// <summary>
    /// Changes the surface material mode and applies it immediately
    /// </summary>
    /// <param name="mode">Requested material mode.</param>
    public void SetMaterialMode(SurfaceMaterialMode mode) {
        materialMode = mode;
        ApplyMaterial();
    }

    /// <summary>
    /// Applies the currently selected material stack to the mesh renderer
    /// </summary>
    void ApplyMaterial() {
        if (_meshRenderer == null) return;

        _meshRenderer.materials = BuildMaterialArrayFromCurrentMaterial();
    }

    /// <summary>
    /// Builds the material array, optionally including the wireframe overlay.
    /// </summary>
    /// <returns>Material array used by the mesh renderer.</returns>
    Material[] BuildMaterialArrayFromCurrentMaterial() {
        Material[] materials = showWireframe
            ? new[] { GetCurrentMaterial(), _wireframeMaterial }
            : new[] { GetCurrentMaterial() };
        return materials;
    }

    /// <summary>
    /// Resolves the base material for the current material mode
    /// </summary>
    /// <returns>Material corresponding to the current material mode</returns>
    Material GetCurrentMaterial() {
        switch (materialMode) {
            case SurfaceMaterialMode.Lit:
                return _litMaterial;
            case SurfaceMaterialMode.Unlit:
                return _unlitMaterial;
            case SurfaceMaterialMode.Soap:
                return _soapMaterial;
            case SurfaceMaterialMode.MeanCurvature:
                return _meanCurvatureMaterial;
            default:
                return _unlitMaterial;
        }
    }
}