using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    /// <summary>
    /// Base class for mesh generators.
    /// </summary>
    public abstract class MeshGeneratorBase : ScriptableObject, IMeshGenerator {
        /// <summary>
        /// Generates initial mesh.
        /// </summary>
        /// <returns>Generated mesh and connectivity information.</returns>
        public abstract MeshData Generate();

        /// <summary>
        /// Builds 1-ring neighborhoods, incident triangle neighbor pairs, and marks boundary vertices as fixed.
        /// </summary>
        /// <param name="vertices">Vertex positions.</param>
        /// <param name="triangles">Triangle index buffer.</param>
        /// <param name="neighbors">For each vertex, store a set of adjacent vertices.</param>
        /// <param name="triangleNeighborPairsByVertex">
        /// For each vertex <c>i</c>, store a list of pairs <c>(j, k)</c> from incident triangles <c>(i, j, k)</c>.
        /// </param>
        /// <param name="fixedVertices">Fixed/Boundary vertices.</param>
        protected void ComputeMeshConnectivity(Vector3[] vertices, int[] triangles, out HashSet<int>[] neighbors, out List<(int, int)>[] triangleNeighborPairsByVertex, out bool[] fixedVertices) {
            neighbors = new HashSet<int>[vertices.Length];
            triangleNeighborPairsByVertex = new List<(int, int)>[vertices.Length];

            for (int i = 0; i < vertices.Length; i++) {
                neighbors[i] = new HashSet<int>();
                triangleNeighborPairsByVertex[i] = new List<(int, int)>();
            }

            HashSet<(int, int)> edges = new HashSet<(int, int)>();

            for (int i = 0; i < triangles.Length; i += 3) {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];

                neighbors[a].Add(b);
                neighbors[a].Add(c);
                neighbors[b].Add(a);
                neighbors[b].Add(c);
                neighbors[c].Add(a);
                neighbors[c].Add(b);

                triangleNeighborPairsByVertex[a].Add((b, c));
                triangleNeighborPairsByVertex[b].Add((c, a));
                triangleNeighborPairsByVertex[c].Add((a, b));

                edges.Add((a, b));
                edges.Add((b, c));
                edges.Add((c, a));
            }

            fixedVertices = new bool[vertices.Length];
            foreach ((int x, int y) in edges) {
                if (!edges.Contains((y, x))) {
                    // A directed edge without opposite direction indicates a boundary edge.
                    fixedVertices[x] = true;
                    fixedVertices[y] = true;
                }
            }
        }
    }
}