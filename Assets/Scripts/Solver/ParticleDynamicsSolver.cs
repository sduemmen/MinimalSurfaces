using Meshes;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/Particle Dynamics")]
    public class ParticleDynamicsSolver : MinimalSurfaceSolverBase {
        public float dt = 0.0001f;
        public float G = 0.87f;
        public float H = 0.59f;
        public float p = 2f;
        public float q = 4f;
        public float velocityDamping = 0.5f;
        public float maxParticleInteractionDistance = 1.311f;
        public float convergenceTolerance = 1e-05f;

        Vector3[] _velocities;
        Vector3[] _accelerations;


        public override void Initialize(MeshData meshData, MinimalSurface context) {
            base.Initialize(meshData, context);
            _velocities = new Vector3[meshData.vertices.Length];
            _accelerations = new Vector3[meshData.vertices.Length];

            ComputeAccelerations(meshData);

            // v_{1/2} = v0 + 0.5*dt*a0
            for (int i = 0; i < meshData.vertices.Length; i++) {
                _velocities[i] = meshData.fixedVertices[i] ? Vector3.zero : 0.5f * dt * _accelerations[i];
            }

            // for (int i = 0; i < meshData.neighbors.Length; i++) {
            //     HashSet<int> neighbors = meshData.neighbors[i];
            //     Vector3 x1 = meshData.vertices[i];
            //     foreach (int j in neighbors) {
            //         Vector3 x2 = meshData.vertices[j];
            //         Debug.Log(Vector3.Distance(x1, x2));
            //     }
            // }
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            // a_k = F(r_k)
            ComputeAccelerations(meshData);

            // v_{k+1/2} = v_{k-1/2} + dt * a_k
            for (int i = 0; i < meshData.vertices.Length; i++) {
                if (meshData.fixedVertices[i]) {
                    _velocities[i] = Vector3.zero;
                    continue;
                }

                _velocities[i] *= 1 - velocityDamping;
                _velocities[i] += dt * _accelerations[i];
            }

            // r_{k+1} = r_k + dt * v_{k+1/2}
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
            return maxGrad < convergenceTolerance;
        }

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

                // for (int j = 0; j < meshData.vertices.Length; j++) {
                //     if (i == j) continue;
                //     
                //     Vector3 x_j = meshData.vertices[j];
                //     Vector3 r_ij = x_i - x_j;
                //     
                //     if (r_ij.magnitude > maxParticleInteractionDistance) continue;
                //
                //     float rp = Mathf.Pow(r_ij.magnitude, p);
                //     float rq = Mathf.Pow(r_ij.magnitude, q);
                //     F_i += (-(G / rp) + (H / rq)) * r_ij.normalized;
                // }

                _accelerations[i] = F_i; // we assume unit mass m=1, so F=ma becomes a=F
            }
        }
    }
}