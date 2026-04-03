using Meshes;
using UnityEngine;

namespace Solver {
    public abstract class MinimalSurfaceSolverBase : ScriptableObject, IMinimalSurfaceSolver {
        float maxAbsMeanCurvature;

        public virtual void Initialize(MeshData meshData, MinimalSurface context) {
            if (context.materialMode == MinimalSurface.SurfaceMaterialMode.MeanCurvature) {
                meshData.ColorVerticesByMeanCurvature(out maxAbsMeanCurvature);
            }
        }

        public abstract bool Step(MeshData meshData, MinimalSurface context);

        protected void UpdateMesh(MeshData meshData, MinimalSurface context) {
            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();

            if (context.materialMode == MinimalSurface.SurfaceMaterialMode.MeanCurvature) {
                meshData.ColorVerticesByMeanCurvature(maxAbsMeanCurvature);
            }
        }
    }
}