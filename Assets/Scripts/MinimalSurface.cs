﻿using System.Collections;
using Meshes;
using Solver;
using UnityEngine;

public class MinimalSurface : MonoBehaviour {
    public enum SurfaceMaterialMode {
        Lit,
        Unlit,
        MeanCurvature,
        Soap
    }

    [SerializeField] MeshFilter _meshFilter;
    [SerializeField] MeshRenderer _meshRenderer;
    [SerializeField] Material _litMaterial;
    [SerializeField] Material _unlitMaterial;
    [SerializeField] Material _meanCurvatureMaterial;
    [SerializeField] Material _soapMaterial;
    [SerializeField] Material _wireframeMaterial;
    [SerializeField] SurfaceMaterialMode _materialMode = SurfaceMaterialMode.MeanCurvature;
    public bool showWireframe = true;

    public int stepsPerSecond = 60;
    public int maxSteps = 5000;

    public int minStepsPerSecond = 1;
    public int maxStepsPerSecond = 240;
    public int minMaxSteps = 100;
    public int maxMaxSteps = 100000;

    MeshData _currentMeshData;
    Coroutine _currentSolveRoutine;
    MinimalSurfaceSolverBase _currentSolver;
    int _convergedAfterSteps;
    int _currentStepCount;
    bool _isPaused;
    bool _isConverged;

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

    public Mesh CurrentMesh {
        get => _currentMeshData?.mesh;
    }

    public SurfaceMaterialMode MaterialMode {
        get => _materialMode;
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

    public void TogglePause() {
        _isPaused = !_isPaused;
    }

    public void GenerateMesh(MeshGeneratorBase generator) {
        if (generator == null) return;

        _currentMeshData = generator.Generate();
        if (_currentMeshData != null && _currentMeshData.mesh != null) {
            _meshFilter.sharedMesh = _currentMeshData.mesh;
        }
    }

    public void StartSolve(MeshGeneratorBase generator, MinimalSurfaceSolverBase solver) {
        CancelSolve();

        if (generator == null || solver == null) return;

        if (_currentMeshData == null || _currentMeshData.mesh == null) {
            GenerateMesh(generator);
        }

        if (_currentMeshData == null || _currentMeshData.mesh == null) return;

        _currentStepCount = 0;
        _isPaused = false;
        _isConverged = false;
        _currentSolver = solver;
        _currentSolver.Initialize(_currentMeshData, this);
        _currentSolveRoutine = StartCoroutine(RunSolve());
    }

    IEnumerator RunSolve() {
        while (_currentStepCount < maxSteps && !_isConverged) {
            if (!_isPaused) {
                _isConverged = _currentSolver.Step(_currentMeshData, this);
                _currentStepCount++;
            }

            yield return new WaitForSeconds(StepInterval);
        }

        if (_currentStepCount >= maxMaxSteps) {
            _isConverged = true;
        }

        _convergedAfterSteps = _currentStepCount >= maxMaxSteps ? maxSteps : _currentStepCount;
        _currentSolveRoutine = null;
    }

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

    public void CancelSolve() {
        if (!IsSolving) return;

        StopCoroutine(_currentSolveRoutine);
        _currentSolveRoutine = null;
        _isPaused = false;
        _isConverged = false;
    }

    public void SetMaterialMode(SurfaceMaterialMode mode) {
        _materialMode = mode;
        ApplyMaterial();
    }

    void ApplyMaterial() {
        if (_meshRenderer == null) return;
        _meshRenderer.materials = BuildMaterialArrayFromCurrentMaterial();
    }

    Material[] BuildMaterialArrayFromCurrentMaterial() {
        Material[] materials = showWireframe 
            ? new[] {GetCurrentMaterial(), _wireframeMaterial}
            : new[] {GetCurrentMaterial()};
        return materials;
    }
    
    Material GetCurrentMaterial() {
        switch (_materialMode) {
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