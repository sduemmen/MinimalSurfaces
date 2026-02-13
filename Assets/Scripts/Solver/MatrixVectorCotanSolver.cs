using Meshes;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Matrix Vector Cotan")]
    public class MatrixVectorCotanSolver : MinimalSurfaceSolverBase {
        public float dt = 0.0005f;
        public float convergenceTolerance = 0.015f;

        Matrix<float> invM;
        Matrix<float> L;

        public override void Initialize(MeshData meshData, MinimalSurface context) {
            return;
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            int vertexCount = meshData.vertices.Length;
            invM = Matrix<float>.Build.Sparse(vertexCount, vertexCount);
            L = Matrix<float>.Build.Sparse(vertexCount, vertexCount);

            BuildMatrices(meshData);
            
            Vector<float> x = Vector<float>.Build.Dense(vertexCount);
            Vector<float> y = Vector<float>.Build.Dense(vertexCount);
            Vector<float> z = Vector<float>.Build.Dense(vertexCount);

            for (int i = 0; i < vertexCount; i++) {
                Vector3 v = meshData.vertices[i];
                x[i] = v.x;
                y[i] = v.y;
                z[i] = v.z;
            }

            Vector<float> dx = invM * L * x;
            Vector<float> dy = invM * L * y;
            Vector<float> dz = invM * L * z;

            float maxGrad = 0f;
            for (int p = 0; p < vertexCount; p++) {
                if (meshData.fixedVertices[p]) continue;

                Vector3 displacement = new Vector3(dx[p], dy[p], dz[p]);

                float d = Mathf.Sqrt(Vector3.Dot(displacement, displacement));
                if (d > maxGrad) {
                    maxGrad = d;
                }

                meshData.vertices[p] -= dt * displacement;
            }

            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();
            return maxGrad < convergenceTolerance;
        }

        void BuildMatrices(MeshData meshData) {
            for (int p = 0; p < meshData.vertices.Length; p++) {
                if (meshData.fixedVertices[p]) continue;

                float Ap = 0f;

                foreach ((int a, int b) in meshData.triangleNeighborPairsByVertex[p]) {
                    Vector3 ab = meshData.vertices[a] - meshData.vertices[b];
                    Vector3 pa = meshData.vertices[p] - meshData.vertices[a];
                    Vector3 pb = meshData.vertices[p] - meshData.vertices[b];
                    Vector3 abxpb = Vector3.Cross(ab, pb);
                    float A = Mathf.Sqrt(Vector3.Dot(abxpb, abxpb));

                    if (A < 1e-04f) continue;

                    float cota = Vector3.Dot(ab, pb) / A;
                    float cotb = -Vector3.Dot(ab, pa) / A;

                    L[p, p] += cota + cotb;
                    L[p, a] += -cotb;
                    L[p, b] += -cota;

                    if (Vector3.Dot(pa, pb) > 0) {
                        if (cota > 0 && cotb > 0) {
                            Ap += (cota * Vector3.Dot(pb, pb) + cotb * Vector3.Dot(pa, pa)) / 8;
                        } else {
                            Ap += 0.25f * A;
                        }
                    } else {
                        Ap += 0.5f * A;
                    }
                }

                invM[p, p] = 1f / Ap;
            }
        }
    }
}
