using Meshes;
using UnityEngine;

namespace Solver {
    /// <summary>
    /// Base class for minimal-surface solvers that operate on <see cref="MeshData"/>.
    /// </summary>
    public abstract class MinimalSurfaceSolverBase : ScriptableObject, IMinimalSurfaceSolver {
        float _maxAbsMeanCurvature;

        /// <summary>
        /// Initializes common visualization state before simulation starts.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current minimal surface.</param>
        public virtual void Initialize(MeshData meshData, MinimalSurface context) {
            if (context.materialMode == MinimalSurface.SurfaceMaterialMode.MeanCurvature) {
                meshData.ColorVerticesByMeanCurvature(out _maxAbsMeanCurvature);
            }
        }

        /// <summary>
        /// Advances the solver by one time step.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current minimal surface.</param>
        /// <returns><see langword="true"/> if the solver has converged.</returns>
        public abstract bool Step(MeshData meshData, MinimalSurface context);

        /// <summary>
        /// Writes updated vertices back to the Unity mesh and refreshes optional mean-curvature coloring.
        /// </summary>
        /// <param name="meshData">Current mesh state.</param>
        /// <param name="context">The current minimal surface.</param>
        protected void UpdateMesh(MeshData meshData, MinimalSurface context) {
            meshData.mesh.SetVertices(meshData.vertices);
            meshData.mesh.RecalculateNormals();

            if (context.materialMode == MinimalSurface.SurfaceMaterialMode.MeanCurvature) {
                meshData.ColorVerticesByMeanCurvature(_maxAbsMeanCurvature);
            }
        }
    }
}