using Meshes;

namespace Solver {
    /// <summary>
    /// Interface for numerical solvers that operate on <see cref="MeshData"/>.
    /// </summary>
    public interface IMinimalSurfaceSolver {
        /// <summary>
        /// Prepares solver state for a new mesh.
        /// </summary>
        /// <param name="meshData">Current geometry and connectivity.</param>
        /// <param name="context">Minimal surface controller that owns simulation settings.</param>
        void Initialize(MeshData meshData, MinimalSurface context);

        /// <summary>
        /// Performs one numerical step and updates <paramref name="meshData"/>.
        /// </summary>
        /// <param name="meshData">Current geometry and connectivity.</param>
        /// <param name="context">Minimal surface controller that owns simulation settings.</param>
        /// <returns><see langword="true"/> if the solver considers the state converged.</returns>
        bool Step(MeshData meshData, MinimalSurface context);
    }
}