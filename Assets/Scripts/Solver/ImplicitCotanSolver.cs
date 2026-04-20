using CSparse;
using CSparse.Double.Factorization;
using CSparse.Storage;
using Meshes;
using UnityEngine;

namespace Solver {
    /// <summary>
    /// Implicit Euler solver with cotangent Laplace-Beltrami discretization.
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/ImplicitCotan")]
    public class ImplicitCotanSolver : MinimalSurfaceSolverBase {
        /// <summary>
        /// Time-step size.
        /// </summary>
        public float dt = 0.0005f;

        /// <summary>
        /// Convergence threshold for the maximum displacement norm.
        /// </summary>
        public float epsilon = 0.015f;

        CompressedColumnStorage<double> _invM;
        CompressedColumnStorage<double> _W;

        double[] _x0;
        double[] _y0;
        double[] _z0;

        /// <summary>
        /// Allocates coordinate buffers and initializes visualization state.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">Minimal surface controller.</param>
        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            int n = meshData.vertices.Length;
            _x0 = new double[n];
            _y0 = new double[n];
            _z0 = new double[n];
        }

        /// <summary>
        /// implicit step 
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">Minimal surface controller.</param>
        /// <returns><see langword="true"/> when the maximum displacement norm falls below <see cref="epsilon"/>.</returns>
        public override bool Step(MeshData meshData, MinimalSurface context) {
            BuildLaplaceMatrix(meshData);

            int n = meshData.vertices.Length;

            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                _x0[i] = v.x;
                _y0[i] = v.y;
                _z0[i] = v.z;
            }

            CompressedColumnStorage<double> L = _invM.Multiply(_W);
            CompressedColumnStorage<double> I = CompressedColumnStorage<double>.CreateIdentity(n);

            CoordinateStorage<double> a = new CoordinateStorage<double>(n, n, I.NonZerosCount + L.NonZerosCount);
            I.EnumerateIndexed((i, j, value) => a.At(i, j, value));
            L.EnumerateIndexed((i, j, value) => a.At(i, j, -dt * value));
            CompressedColumnStorage<double> A = CompressedColumnStorage<double>.OfIndexed(a, true);

            SparseLU lu = SparseLU.Create(A, ColumnOrdering.MinimumDegreeAtPlusA, 1.0);

            // solve the linear system componentwise for x's, y's, z's
            double[] x1 = new double[n];
            double[] y1 = new double[n];
            double[] z1 = new double[n];
            lu.Solve(_x0, x1);
            lu.Solve(_y0, y1);
            lu.Solve(_z0, z1);

            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                Vector3 displacement = new Vector3((float)(x1[i] - _x0[i]), (float)(y1[i] - _y0[i]), (float)(z1[i] - _z0[i])) / dt;
                maxGrad = Mathf.Max(maxGrad, displacement.magnitude);

                meshData.vertices[i] = new Vector3((float)x1[i], (float)y1[i], (float)z1[i]);
            }

            UpdateMesh(meshData, context);
            return maxGrad < epsilon;
        }

        /// <summary>
        /// Builds diagonal inverse mass matrix and cotangent weight matrix from current geometry.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        void BuildLaplaceMatrix(MeshData meshData) {
            int n = meshData.vertices.Length;

            int mEntries = 0;
            int wEntries = 0;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                mEntries++;
                wEntries += meshData.triangleNeighborPairsByVertex[i].Count * 3;
            }

            CoordinateStorage<double> invM = new CoordinateStorage<double>(n, n, mEntries);
            CoordinateStorage<double> W = new CoordinateStorage<double>(n, n, wEntries);

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

                    if (A < 1e-05f) continue;

                    float cota = CotanBetween(ai, -ba);
                    float cotb = CotanBetween(bi, ba);

                    W.At(i, i, -(cota + cotb));
                    W.At(i, a, cotb);
                    W.At(i, b, cota);

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

                if (A_i < 1e-05f) continue;

                invM.At(i, i, 1f / (2f * A_i));
            }

            _invM = CompressedColumnStorage<double>.OfIndexed(invM, true);
            _W = CompressedColumnStorage<double>.OfIndexed(W, true);
        }

        /// <summary>
        /// Computes <c>cot(theta)</c> between two vectors.
        /// </summary>
        /// <param name="u">First vector.</param>
        /// <param name="v">Second vector.</param>
        /// <returns><c>dot(u,v)/|u x v|</c>.</returns>
        float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}