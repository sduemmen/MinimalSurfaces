using System.Collections;
using Meshes;
using UnityEngine;

namespace Solver {
    public abstract class MinimalSurfaceSolverBase : ScriptableObject, IMinimalSurfaceSolver {
        public abstract void Initialize(MeshData meshData, MinimalSurface context);

        public abstract bool Step(MeshData meshData, MinimalSurface context);
    }
}