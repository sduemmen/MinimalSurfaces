using Meshes;
using UnityEngine;

namespace Solver {
    /// <summary>
    /// Particle-based solver with damped leapfrog integration and pairwise interaction forces.
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Particle Dynamics")]
    public class ParticleDynamicsSolver : MinimalSurfaceSolverBase {
        /// <summary>
        /// Time-step size.
        /// </summary>
        public float dt = 0.0001f;

        /// <summary>
        /// Strength of the attractive term.
        /// </summary>
        public float G = 0.87f;

        /// <summary>
        /// Strength of the repulsive term.
        /// </summary>
        public float H = 0.59f;

        /// <summary>
        /// Exponent of the attractive distance term.
        /// </summary>
        public float p = 2f;

        /// <summary>
        /// Exponent of the repulsive distance term.
        /// </summary>
        public float q = 4f;

        /// <summary>
        /// Per-step velocity damping factor.
        /// </summary>
        public float zeta = 0.5f;

        /// <summary>
        /// Convergence threshold for the maximum per-step displacement.
        /// </summary>
        public float epsilon = 1e-05f;

        Vector3[] _velocities;
        Vector3[] _accelerations;

        /// <summary>
        /// Initializes state and computes the first half-step velocity.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current minimal surface.</param>
        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            _velocities = new Vector3[meshData.vertices.Length];
            _accelerations = new Vector3[meshData.vertices.Length];

            ComputeAccelerations(meshData);

            // v_{1/2} = v0 + 0.5*dt*a0
            for (int i = 0; i < meshData.vertices.Length; i++) {
                _velocities[i] = meshData.fixedVertices[i] ? Vector3.zero : 0.5f * dt * _accelerations[i];
            }
        }

        /// <summary>
        /// Advances one step.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current minimal surface.</param>
        /// <returns><see langword="true"/> when the maximum per-step displacement is below <see cref="epsilon"/>.</returns>
        public override bool Step(MeshData meshData, MinimalSurface context) {
            // a_k = F(r_k)
            ComputeAccelerations(meshData);

            // compute velocity using the new acceleration: v_{k+1/2} = (1-zeta) v_{k-1/2} + dt * a_k
            for (int i = 0; i < meshData.vertices.Length; i++) {
                if (meshData.fixedVertices[i]) {
                    _velocities[i] = Vector3.zero;
                    continue;
                }

                _velocities[i] *= 1 - zeta;
                _velocities[i] += dt * _accelerations[i];
            }

            // compute new position using the new velocity: r_{k+1} = r_k + dt * v_{k+1/2}
            float maxGrad = 0f;
            for (int i = 0; i < meshData.vertices.Length; i++) {
                if (meshData.fixedVertices[i]) continue;

                Vector3 r_k = meshData.vertices[i];
                meshData.vertices[i] += dt * _velocities[i];

                float d = (meshData.vertices[i] - r_k).magnitude;
                if (d > maxGrad) {
                    maxGrad = d;
                }
            }

            UpdateMesh(meshData, context);
            return maxGrad < epsilon;
        }

        /// <summary>
        /// Recomputes vertex accelerations from pairwise neighbor forces.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        void ComputeAccelerations(MeshData meshData) {
            for (int i = 0; i < meshData.vertices.Length; i++) {
                if (meshData.fixedVertices[i]) {
                    _accelerations[i] = Vector3.zero;
                    continue;
                }

                Vector3 F_i = Vector3.zero;
                Vector3 x_i = meshData.vertices[i];

                foreach (int j in meshData.neighbors[i]) {
                    Vector3 x_j = meshData.vertices[j];
                    Vector3 r_ij = x_i - x_j;

                    float rp = Mathf.Pow(r_ij.magnitude, p);
                    float rq = Mathf.Pow(r_ij.magnitude, q);
                    F_i += (-(G / rp) + H / rq) * r_ij.normalized;
                }

                _accelerations[i] = F_i;
            }
        }
    }
}