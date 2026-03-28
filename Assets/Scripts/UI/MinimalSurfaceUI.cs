using System;
using System.Collections.Generic;
using Meshes;
using System.IO;
using Solver;
using ThirdParty.StandaloneFileBrowser;
using ThirdParty.Stl;
using UnityEngine;
using UnityEngine.InputSystem;

namespace UI {
    public class MinimalSurfaceUI : MonoBehaviour {
        [SerializeField] MainCamera _mainCamera;
        [SerializeField] MinimalSurface _surface;

        [SerializeField] List<MeshGeneratorBase> _generators = new List<MeshGeneratorBase>();
        [SerializeField] List<MinimalSurfaceSolverBase> _solvers = new List<MinimalSurfaceSolverBase>();

        [SerializeField] Rect _panelRect = new Rect(12, 12, 520, 1080);
        [SerializeField] KeyCode _toggleUIKey = KeyCode.T;
        
        bool shouldHideUI = false;

        int _selectedGeneratorIdx;
        int _selectedSolverIdx;

        MeshGeneratorBase _generator;
        MinimalSurfaceSolverBase _solver;

        Vector2 _scroll;

        void Awake() {
            if (_surface == null) {
                _surface = FindFirstObjectByType<MinimalSurface>();
            }

            if (_mainCamera == null) {
                _mainCamera = FindFirstObjectByType<MainCamera>();
            }

            EnsureInstances();
        }

        void Start() {
            _surface.GenerateMesh(_generator);
        }

        void Update() {
            if (Keyboard.current.tKey.wasPressedThisFrame) {
                shouldHideUI = !shouldHideUI;
            }

            if (Keyboard.current.sKey.wasPressedThisFrame) {
                CaptureScreenshot();
            }
        }

        void CaptureScreenshot() {
            Camera captureCamera = _mainCamera.GetComponent<Camera>();

            string screenshotDir = Path.Combine(Application.dataPath, "Screenshots");
            if (!Directory.Exists(screenshotDir)) {
                Directory.CreateDirectory(screenshotDir);
            }

            string fileName = $"screenshot_{DateTime.Now:yyyyMMdd_HHmmssfff}.png";
            string filePath = Path.Combine(screenshotDir, fileName);

            const int width = 2160;
            const int height = 2160;

            RenderTexture rt = new RenderTexture(width, height, 24);
            Texture2D texture = new Texture2D(width, height, TextureFormat.RGBA32, false);

            RenderTexture previousActive = RenderTexture.active;
            RenderTexture previousTarget = captureCamera.targetTexture;

            try {
                captureCamera.targetTexture = rt;
                captureCamera.Render();

                RenderTexture.active = rt;
                texture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
                texture.Apply();

                byte[] png = texture.EncodeToPNG();
                File.WriteAllBytes(filePath, png);
                Debug.Log($"Saved screenshot: {filePath}");
            } finally {
                captureCamera.targetTexture = previousTarget;
                RenderTexture.active = previousActive;
                Destroy(rt);
                Destroy(texture);
            }
        }

        void EnsureInstances() {
            _generator = GetOrCreateInstance(_generators, _selectedGeneratorIdx, _generator);
            _solver = GetOrCreateInstance(_solvers, _selectedSolverIdx, _solver);
        }

        static T GetOrCreateInstance<T>(List<T> list, int index, T currentInstance) where T : ScriptableObject {
            if (list == null || list.Count == 0) return null;

            index = Mathf.Clamp(index, 0, list.Count - 1);
            T selected = list[index];

            if (selected == null) return null;

            if (currentInstance == null || currentInstance.name != selected.name + " (Runtime)") {
                T clone = Instantiate(selected);
                clone.name = selected.name + " (Runtime)";
                return clone;
            }

            return currentInstance;
        }

        void OnGUI() {
            if (shouldHideUI) return;
            
            GUILayout.BeginArea(_panelRect, GUI.skin.window);
            _scroll = GUILayout.BeginScrollView(_scroll);

            GUILayout.Space(10);

            // Material
            GUILayout.Label("Material", GUI.skin.box);
            MinimalSurface.SurfaceMaterialMode nextMaterial = RuntimeUI.DrawMaterialSelector(_surface.MaterialMode);
            if (nextMaterial != _surface.MaterialMode) {
                _surface.SetMaterialMode(nextMaterial);
            }

            GUILayout.Space(10);

            // Camera parameters
            GUILayout.Label("Camera", GUI.skin.box);
            _mainCamera.shouldRotate = GUILayout.Toggle(_mainCamera.shouldRotate, "Rotate Camera");
            _mainCamera.cameraDistance = RuntimeUI.DrawFloatSlider("Camera Distance", _mainCamera.cameraDistance, _mainCamera.minCameraDistance, _mainCamera.maxCameraDistance);

            GUI.enabled = !_mainCamera.shouldRotate;
            _mainCamera.theta = RuntimeUI.DrawFloatSlider("Theta", _mainCamera.theta, 0, Mathf.PI);
            _mainCamera.phi = RuntimeUI.DrawFloatSlider("Phi", _mainCamera.phi, 0, 2 * Mathf.PI);
            GUI.enabled = true;
            
            GUILayout.Space(10);

            // Solve parameters
            GUILayout.Label("Solve Parameters", GUI.skin.box);
            _surface.stepsPerSecond = RuntimeUI.DrawIntSlider("Steps per Second", _surface.stepsPerSecond, _surface.minStepsPerSecond, _surface.maxStepsPerSecond);
            _surface.maxSteps = RuntimeUI.DrawIntSlider("Max Steps", _surface.maxSteps, _surface.minMaxSteps, _surface.maxMaxSteps);

            GUILayout.Space(10);

            // Generator
            GUI.enabled = !_surface.IsSolving;
            GUILayout.BeginHorizontal(GUI.skin.box);
            GUILayout.Label("Mesh Generator:", GUILayout.Width(120f));
            int prevGenerator = _selectedGeneratorIdx;
            _selectedGeneratorIdx = RuntimeUI.DrawAssetList(_generators, _selectedGeneratorIdx);
            bool generatorChanged = _selectedGeneratorIdx != prevGenerator;
            GUILayout.EndHorizontal();

            bool generatorSettingsChanged = RuntimeUI.DrawObjectFields(_generator);

            if (_generator is StlMeshLoader stlMeshLoader) {
                GUILayout.Space(5);

                if (GUILayout.Button("Load (.stl)", GUILayout.Width(120f))) {
                    ExtensionFilter[] filters = { new ExtensionFilter("STL", "stl") };
                    string[] paths = StandaloneFileBrowser.OpenFilePanel("Load STL", "", filters, false);
                    if (paths.Length != 0) {
                        stlMeshLoader.FilePath = paths[0];
                        _surface.GenerateMesh(_generator);
                    }
                }
            }

            GUILayout.Space(10);

            // Solver
            GUI.enabled = true;
            GUILayout.BeginHorizontal(GUI.skin.box);
            GUILayout.Label("Solver:", GUILayout.Width(120f));
            int prevSolver = _selectedSolverIdx;
            _selectedSolverIdx = RuntimeUI.DrawAssetList(_solvers, _selectedSolverIdx);
            bool solverChanged = _selectedSolverIdx != prevSolver;
            GUILayout.EndHorizontal();
            RuntimeUI.DrawObjectFields(_solver);

            EnsureInstances();

            // if ((generatorChanged || generatorSettingsChanged) && !_surface.IsSolving) {
            //     _surface.GenerateMesh(_generator);
            // }

            if (solverChanged && _surface.IsSolving) {
                _surface.SwitchSolver(_solver);
            }

            GUILayout.EndScrollView();

            GUILayout.Space(5);
            DrawControlButtons();

            GUILayout.EndArea();
        }

        void DrawControlButtons() {
            GUILayout.BeginHorizontal();

            GUI.enabled = !_surface.IsSolving;
            if (GUILayout.Button("Generate Mesh", GUILayout.Height(32f))) {
                EnsureInstances();
                _surface.GenerateMesh(_generator);
            }

            if (GUILayout.Button("Solve", GUILayout.Height(32f))) {
                EnsureInstances();
                _surface.StartSolve(_generator, _solver);
            }

            GUI.enabled = _surface.IsSolving;
            string pauseResumeText = _surface.IsPaused ? "Resume" : "Pause";
            if (GUILayout.Button(pauseResumeText, GUILayout.Height(32f))) {
                _surface.TogglePause();
            }

            if (GUILayout.Button("Cancel Solve", GUILayout.Height(32f))) {
                _surface.CancelSolve();
            }

            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUI.enabled = _surface.CurrentMesh != null;
            GUILayout.Space(6);
            if (GUILayout.Button("Export (.stl)", GUILayout.Height(32f))) {
                ExtensionFilter[] filters = { new ExtensionFilter("STL", "stl") };
                string path = StandaloneFileBrowser.SaveFilePanel("Export STL", "", _surface.CurrentMesh.name, filters);

                if (!string.IsNullOrEmpty(path)) {
                    Exporter.WriteFile(path, _surface.CurrentMesh, FileType.Binary, true);
                }
            }

            GUI.enabled = true;
            GUILayout.Space(5);
            string status;
            if (_surface.IsPaused) {
                status = "Status: Paused";
            } else if (_surface.IsSolving) {
                status = $"Status: Solving ({_surface.CurrentStepCount} steps)";
            } else if (_surface.IsConverged) {
                status = $"Status: Converged after {_surface.ConvergedAfterSteps} steps";
            } else {
                status = "Status: Idle";
            }

            GUILayout.Label(status);
        }
    }
}
