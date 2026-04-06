using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using Meshes;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Laplacian Smoothing Matrix")]
    public class LaplacianSmoothingSolver : MinimalSurfaceSolverBase {
        public float dt = 1f;
        public float convergenceTolerance = 1e-05f;

        Matrix<float> L;

        Vector<float> xs;
        Vector<float> ys;
        Vector<float> zs;
        Vector<float> dx;
        Vector<float> dy;
        Vector<float> dz;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            BuildLaplaceMatrix(meshData);

            int n = meshData.vertices.Length;
            xs = Vector<float>.Build.Dense(n);
            ys = Vector<float>.Build.Dense(n);
            zs = Vector<float>.Build.Dense(n);
            dx = Vector<float>.Build.Dense(n);
            dy = Vector<float>.Build.Dense(n);
            dz = Vector<float>.Build.Dense(n);
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            int n = meshData.vertices.Length;
            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                xs[i] = v.x;
                ys[i] = v.y;
                zs[i] = v.z;
            }

            L.Multiply(xs, dx);
            L.Multiply(ys, dy);
            L.Multiply(zs, dz);

            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                Vector3 displacement = new Vector3(dx[i], dy[i], dz[i]);

                float d = Mathf.Sqrt(Vector3.Dot(displacement, displacement));
                if (d > maxGrad) {
                    maxGrad = d;
                }

                meshData.vertices[i] += dt * displacement;
            }

            UpdateMesh(meshData, context);
            return maxGrad < convergenceTolerance;
        }

        void BuildLaplaceMatrix(MeshData meshData) {
            int n = meshData.vertices.Length;
            L = Matrix<float>.Build.Sparse(n, n);

            for (int i = 0; i < meshData.vertices.Length; i++) {
                HashSet<int> neighbors = meshData.neighbors[i];
                if (meshData.fixedVertices[i] || neighbors.Count == 0) continue;

                L[i, i] = -1f;
                foreach (int j in neighbors) {
                    L[i, j] = 1f / neighbors.Count;
                }
            }
        }
    }
}