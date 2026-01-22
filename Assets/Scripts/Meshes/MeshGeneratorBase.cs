using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    public abstract class MeshGeneratorBase : ScriptableObject, IMeshGenerator {
        public abstract MeshData Generate();

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
                    fixedVertices[x] = true;
                    fixedVertices[y] = true;
                }
            }
        }
    }
}