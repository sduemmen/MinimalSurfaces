using System.Collections;
using Meshes;

namespace Solver {
    public interface IMinimalSurfaceSolver {
        void Initialize(MeshData meshData, MinimalSurface context);
        bool Step(MeshData meshData, MinimalSurface context);
    }
}