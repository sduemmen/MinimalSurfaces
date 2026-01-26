using System.Collections.Generic;
using Meshes;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Laplacian Smoothing")]
    public class LaplacianSmoothingSolver : MinimalSurfaceSolverBase {
        public float dt = 1f;
        public float convergenceTolerance = 1e-05f;

        Vector3[] _displacement;
        bool _initialized;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
            _displacement = new Vector3[meshData.vertices.Length];
            _initialized = true;
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            if (!_initialized) {
                Debug.LogError("Solver has not been initialized.");
                return true;
            }

            for (int p = 0; p < meshData.vertices.Length; p++) {
                _displacement[p] = Vector3.zero;
                List<(int, int)> adjacentVertexPairs = meshData.triangleNeighborPairsByVertex[p];

                if (meshData.fixedVertices[p] || adjacentVertexPairs.Count == 0) continue;

                Vector3 c = Vector3.zero;
                foreach ((int a, int b) in adjacentVertexPairs) {
                    c += meshData.vertices[a] + meshData.vertices[b];
                }

                c /= 2f * adjacentVertexPairs.Count;
                _displacement[p] = c - meshData.vertices[p];
            }

            float maxGrad = 0f;

            for (int p = 0; p < meshData.vertices.Length; p++) {
                if (meshData.fixedVertices[p]) continue;

                float d = _displacement[p].magnitude;
                if (d > maxGrad) {
                    maxGrad = d;
                }

                meshData.vertices[p] += dt * _displacement[p];
            }

            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();

            return maxGrad < convergenceTolerance;
        }
    }
}