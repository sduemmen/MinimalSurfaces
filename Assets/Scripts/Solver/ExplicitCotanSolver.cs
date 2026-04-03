using System;
using Meshes;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Matrix Vector Cotan")]
    public class ExplicitCotanSolver : MinimalSurfaceSolverBase {
        public float dt = 0.0005f;
        public float convergenceTolerance = 0.015f;

        Matrix<float> invM;
        Matrix<float> W;
        
        Vector<float> xs;
        Vector<float> ys;
        Vector<float> zs;
        Vector<float> dx;
        Vector<float> dy;
        Vector<float> dz;
        
        [SerializeField] Gradient meanCurvatureColorGradient;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
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
            BuildLaplaceMatrix(meshData);
            
            int n = meshData.vertices.Length;

            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                xs[i] = v.x;
                ys[i] = v.y;
                zs[i] = v.z;
            }
            
            Matrix<float> L = invM * W;

            L.Multiply(xs, dx);
            L.Multiply(ys, dy);
            L.Multiply(zs, dz);
            
            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                Vector3 displacement = new Vector3(dx[i], dy[i], dz[i]);
                maxGrad = Mathf.Max(maxGrad, displacement.magnitude);

                meshData.vertices[i] += dt * displacement;
            }

            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();
            ColorByMeanCurvature(meshData);
            return maxGrad < convergenceTolerance;
        }

        void ColorByMeanCurvature(MeshData meshData) {
            int n = meshData.vertices.Length;
            float[] H = new float[n];
            Color[] colors = new Color[n];
            Vector3[] normals = meshData.mesh.normals;

            float maxAbs = 0f;
            for (int i = 0; i < n; i++) {
                H[i] = 0.5f * Vector3.Dot(new Vector3(dx[i], dy[i], dz[i]), normals[i]);
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
            
            
            for (int i = 0; i < meshData.vertices.Length; i++) {
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

                    float cot_alpha = CotanBetween(ai, -ba);
                    float cot_beta = CotanBetween(bi, ba);

                    W[i, i] -= cot_alpha + cot_beta;
                    W[i, a] += cot_beta;
                    W[i, b] += cot_alpha;
                    
                    bool obtuseAtI = Vector3.Dot(ai, bi) < 0;
                    bool obtuseAtA = Vector3.Dot(ai, -ba) < 0;
                    bool obtuseAtB = Vector3.Dot(bi, ba) < 0;

                    if (obtuseAtI) {
                        A_i += 0.5f * A;
                    } else if (obtuseAtA || obtuseAtB) {
                        A_i += 0.25f * A;
                    } else {
                        A_i += (cot_beta * ai.sqrMagnitude + cot_alpha * bi.sqrMagnitude) / 8;
                    }
                }

                if (A_i < 1e-03f) continue;
                invM[i, i] = 1f / (2f * A_i);
            }
        }
        
        float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}
