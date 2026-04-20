using System.Collections.Generic;
using CSparse.Storage;
using Meshes;
using UnityEngine;

namespace Solver {
    /// <summary>
    /// Explicit Euler solver with Laplacian smoothing.
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Laplacian Smoothing Matrix")]
    public class LaplacianSmoothingSolver : MinimalSurfaceSolverBase {
        /// <summary>
        /// Time-step size.
        /// </summary>
        public float dt = 1f;

        /// <summary>
        /// Convergence threshold for the maximum displacement norm.
        /// </summary>
        public float epsilon = 1e-05f;

        CompressedColumnStorage<double> _L;

        double[] _xs;
        double[] _ys;
        double[] _zs;
        double[] _dx;
        double[] _dy;
        double[] _dz;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            BuildLaplaceMatrix(meshData); // because the weights are constant, we can pre-compute the matrix

            int n = meshData.vertices.Length;
            _xs = new double[n];
            _ys = new double[n];
            _zs = new double[n];
            _dx = new double[n];
            _dy = new double[n];
            _dz = new double[n];
        }

        /// <summary>
        /// Explicit smoothing step
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">Minimal Surface controller.</param>
        /// <returns><see langword="true"/> when the maximum displacement norm is below <see cref="epsilon"/>.</returns>
        public override bool Step(MeshData meshData, MinimalSurface context) {
            int n = meshData.vertices.Length;

            // copy current vertex components into the arrays
            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                _xs[i] = v.x;
                _ys[i] = v.y;
                _zs[i] = v.z;
            }

            // perform the matrix-vector product componentwise for x's, y's, z's
            _L.Multiply(_xs, _dx);
            _L.Multiply(_ys, _dy);
            _L.Multiply(_zs, _dz);

            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                // compute displacement vector from dx, dy, dz
                Vector3 displacement = new Vector3((float)_dx[i], (float)_dy[i], (float)_dz[i]);

                float d = Mathf.Sqrt(Vector3.Dot(displacement, displacement));
                if (d > maxGrad) {
                    maxGrad = d;
                }

                meshData.vertices[i] += dt * displacement;
            }

            UpdateMesh(meshData, context);
            return maxGrad < epsilon;
        }

        /// <summary>
        /// Builds the uniform Laplacian matrix from 1-ring connectivity.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        void BuildLaplaceMatrix(MeshData meshData) {
            int n = meshData.vertices.Length;
            int entryCount = 0;
            for (int i = 0; i < n; i++) {
                HashSet<int> neighbors = meshData.neighbors[i];
                if (meshData.fixedVertices[i] || neighbors.Count == 0) continue;

                entryCount += neighbors.Count + 1;
            }

            CoordinateStorage<double> L = new CoordinateStorage<double>(n, n, Mathf.Max(1, entryCount));

            for (int i = 0; i < n; i++) {
                HashSet<int> neighbors = meshData.neighbors[i];
                if (meshData.fixedVertices[i] || neighbors.Count == 0) continue;

                L.At(i, i, -1.0);
                foreach (int j in neighbors) {
                    L.At(i, j, 1.0 / neighbors.Count);
                }
            }

            _L = CompressedColumnStorage<double>.OfIndexed(L, true);
        }
    }
}