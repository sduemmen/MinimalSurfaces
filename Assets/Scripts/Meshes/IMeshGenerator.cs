namespace Meshes {
    /// <summary>
    /// Produces a triangulated surface.
    /// </summary>
    public interface IMeshGenerator {
        /// <summary>
        /// Generates mesh geometry and precomputes the mesh connectivity required by the solvers.
        /// </summary>
        /// <returns>
        /// A populated <see cref="MeshData"/> instance.
        /// </returns>
        MeshData Generate();
    }
}