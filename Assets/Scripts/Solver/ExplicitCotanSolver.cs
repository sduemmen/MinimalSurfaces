using CSparse.Storage;
using Meshes;
using UnityEngine;

namespace Solver {
    /// <summary>
    /// Explicit Euler solver using the cotangent Laplace-Beltrami discretization.
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Matrix Vector Cotan")]
    public class ExplicitCotanSolver : MinimalSurfaceSolverBase {
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

        double[] _xs;
        double[] _ys;
        double[] _zs;
        double[] _dx;
        double[] _dy;
        double[] _dz;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            BuildLaplaceMatrix(meshData);

            int n = meshData.vertices.Length;

            _xs = new double[n];
            _ys = new double[n];
            _zs = new double[n];
            _dx = new double[n];
            _dy = new double[n];
            _dz = new double[n];
        }

        /// <summary>
        /// Explicit step.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current Minimal surface object.</param>
        /// <returns><see langword="true"/> when the maximum displacement norm falls below <see cref="epsilon"/>.</returns>
        public override bool Step(MeshData meshData, MinimalSurface context) {
            BuildLaplaceMatrix(meshData);

            int n = meshData.vertices.Length;

            // copy current vertex components into the arrays
            for (int i = 0; i < n; i++) {
                Vector3 v = meshData.vertices[i];
                _xs[i] = v.x;
                _ys[i] = v.y;
                _zs[i] = v.z;
            }

            CompressedColumnStorage<double> L = _invM.Multiply(_W);

            // perform the matrix-vector product componentwise for x's, y's, z's
            L.Multiply(_xs, _dx);
            L.Multiply(_ys, _dy);
            L.Multiply(_zs, _dz);

            float maxGrad = 0f;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                // compute displacement vector from dx, dy, dz
                Vector3 displacement = new Vector3((float)_dx[i], (float)_dy[i], (float)_dz[i]);
                maxGrad = Mathf.Max(maxGrad, displacement.magnitude);

                meshData.vertices[i] += dt * displacement;
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

            int invEntries = 0;
            int wEntries = 0;
            for (int i = 0; i < n; i++) {
                if (meshData.fixedVertices[i]) continue;

                invEntries++;
                wEntries += meshData.triangleNeighborPairsByVertex[i].Count * 3;
            }

            CoordinateStorage<double> invM = new CoordinateStorage<double>(n, n, Mathf.Max(1, invEntries));
            CoordinateStorage<double> W = new CoordinateStorage<double>(n, n, Mathf.Max(1, wEntries));

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

                    if (A < 1e-05f) continue; // skip degenerate triangles

                    float cotAlpha = CotanBetween(ai, -ba);
                    float cotBeta = CotanBetween(bi, ba);

                    W.At(i, i, -(cotAlpha + cotBeta));
                    W.At(i, a, cotBeta);
                    W.At(i, b, cotAlpha);

                    bool obtuseAtI = Vector3.Dot(ai, bi) < 0;
                    bool obtuseAtA = Vector3.Dot(ai, -ba) < 0;
                    bool obtuseAtB = Vector3.Dot(bi, ba) < 0;

                    // mixed voronoi area
                    if (obtuseAtI) {
                        A_i += 0.5f * A;
                    } else if (obtuseAtA || obtuseAtB) {
                        A_i += 0.25f * A;
                    } else {
                        A_i += (cotBeta * ai.sqrMagnitude + cotAlpha * bi.sqrMagnitude) / 8f;
                    }
                }

                if (A_i < 1e-05f) continue;

                invM.At(i, i, 1f / (2f * A_i));
            }

            _invM = CompressedColumnStorage<double>.OfIndexed(invM, true);
            _W = CompressedColumnStorage<double>.OfIndexed(W, true);
        }

        float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}