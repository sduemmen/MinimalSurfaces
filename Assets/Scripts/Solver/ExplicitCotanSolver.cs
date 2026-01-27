using Meshes;
using UnityEngine;

namespace Solver {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Solvers/ExplicitCotan")]
    public class ExplicitCotanSolver : MinimalSurfaceSolverBase {
        public float dt = 0.0005f;
        public float convergenceTolerance = 0.015f;

        Vector3[] _displacement;
        
        
        public override void Initialize(MeshData meshData, MinimalSurface context) {
            _displacement = new Vector3[meshData.vertices.Length];
        }

        public override bool Step(MeshData meshData, MinimalSurface context) {
            float maxGrad = 0f;
            for (int p = 0; p < meshData.vertices.Length; p++) {
                if (meshData.fixedVertices[p]) continue;

                _displacement[p] = Vector3.zero;
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
                    
                    _displacement[p] += pa * cotb + pb * cota;
                    if (Vector3.Dot(pa, pb) > 0) {
                        if (cota > 0 && cotb > 0) {
                            Ap += (cota*Vector3.Dot(pb, pb) + cotb*Vector3.Dot(pa, pa)) / 8;
                        } else {
                            Ap += 0.25f * A;
                        }
                    } else {
                        Ap += 0.5f * A;
                    }
                }

                _displacement[p] /= Ap;
                
                float d = Mathf.Sqrt(Vector3.Dot(_displacement[p], _displacement[p]));

                if (d > maxGrad) {
                    maxGrad = d;
                }
                
                meshData.vertices[p] -= dt * _displacement[p];
            }

            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();
            return maxGrad < convergenceTolerance;
        }
    }
}