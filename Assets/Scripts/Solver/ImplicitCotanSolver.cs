using Meshes;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/ImplicitCotan")]
    public class ImplicitCotanSolver : MinimalSurfaceSolverBase {
        public float dt = 0.0005f;
        public float convergenceTolerance = 0.015f;

        Matrix<float> invM;
        Matrix<float> W;

        Vector<float> x0;
        Vector<float> y0;
        Vector<float> z0;
        
        [SerializeField] Gradient meanCurvatureColorGradient;
        

        public override void Initialize(MeshData meshData, MinimalSurface context) {
            int n = meshData.vertices.Length;
            x0 = Vector<float>.Build.Dense(n);
            y0 = Vector<float>.Build.Dense(n);
            z0 = Vector<float>.Build.Dense(n);
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            BuildLaplaceMatrix(meshData);

            int n = meshData.vertices.Length;

            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                x0[i] = v.x;
                y0[i] = v.y;
                z0[i] = v.z;
            }

            Matrix<float> L = invM * W;
            Matrix<float> I = Matrix<float>.Build.SparseIdentity(n);
            Matrix<float> A = I - dt * L;
            
            Vector<float> x1 = A.Solve(x0);
            Vector<float> y1 = A.Solve(y0);
            Vector<float> z1 = A.Solve(z0);

            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;
                
                Vector3 displacement = new Vector3(x1[i] - x0[i], y1[i] - y0[i], z1[i] - z0[i]) / dt;

                float d = displacement.magnitude;
                if (d > maxGrad) {
                    maxGrad = d;
                }

                meshData.vertices[i] = new Vector3(x1[i], y1[i], z1[i]);
            }

            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();
            ColorByMeanCurvature(meshData, x1, y1, z1);
            return maxGrad < convergenceTolerance;
        }
        
        void ColorByMeanCurvature(MeshData meshData, Vector<float> x1, Vector<float> y1, Vector<float> z1) {
            int n = meshData.vertices.Length;
            float[] H = new float[n];
            Color[] colors = new Color[n];
            Vector3[] normals = meshData.mesh.normals;

            float maxAbs = 0f;
            for (int i = 0; i < n; i++) {
                H[i] = 0.5f * Vector3.Dot(new Vector3(x1[i] - x0[i], y1[i] - y0[i], z1[i] - z0[i]) / dt, normals[i]);
                maxAbs = Mathf.Max(maxAbs, Mathf.Abs(H[i]));
            }
            
            maxAbs = Mathf.Max(maxAbs, 1e-05f);

            for (int i = 0; i < n; i++) {
                float t = 0.5f + 0.5f * H[i] / maxAbs;
                colors[i] = meanCurvatureColorGradient.Evaluate(t);
            }
            
            meshData.mesh.SetColors(colors);
        }

        void BuildLaplaceMatrix(MeshData meshData) {
            int n = meshData.vertices.Length;
            invM = Matrix<float>.Build.Sparse(n, n);
            W = Matrix<float>.Build.Sparse(n, n);

            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                float A_i = 0f;

                foreach ((int a, int b) in meshData.triangleNeighborPairsByVertex[i]) {
                    Vector3 x_i = meshData.vertices[i];
                    Vector3 x_a = meshData.vertices[a];
                    Vector3 x_b = meshData.vertices[b];
                    Vector3 ba = x_a - x_b;
                    Vector3 ai = x_i - x_a;
                    Vector3 bi = x_i - x_b;
                    float A = 0.5f * Vector3.Cross(ba, bi).magnitude;

                    if (A < 1e-03f) continue;

                    float cota = CotanBetween(ai, -ba);
                    float cotb = CotanBetween(bi, ba);

                    W[i, i] -= cota + cotb;
                    W[i, a] += cotb;
                    W[i, b] += cota;

                    bool obtuseAtI = Vector3.Dot(ai, bi) < 0f;
                    bool obtuseAtA = Vector3.Dot(ai, -ba) < 0f;
                    bool obtuseAtB = Vector3.Dot(bi, ba) < 0f;

                    if (obtuseAtI) {
                        A_i += 0.5f * A;
                    } else if (obtuseAtA || obtuseAtB) {
                        A_i += 0.25f * A;
                    } else {
                        A_i += (cotb * ai.sqrMagnitude + cota * bi.sqrMagnitude) / 8f;
                    }
                }

                invM[i, i] = 1f / (2f * A_i);
            }
        }

        float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}
