using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    public class MeshData {
        public Mesh mesh;
        public Vector3[] vertices;
        public int[] triangles;
        public HashSet<int>[] neighbors;
        public List<(int, int)>[] triangleNeighborPairsByVertex;
        public bool[] fixedVertices;

        public MeshData(Mesh mesh, Vector3[] vertices, int[] triangles, HashSet<int>[] neighbors, List<(int, int)>[] triangleNeighborPairsByVertex, bool[] fixedVertices) {
            this.mesh = mesh;
            this.vertices = vertices;
            this.triangles = triangles;
            this.neighbors = neighbors; // For each vertex i, store a list of its 1-ring neighbors
            this.triangleNeighborPairsByVertex = triangleNeighborPairsByVertex; // For each vertex i, stores a list of (j, k) pairs such that (i, j, k) forms a triangle (i.e. the two other vertices adjacent to i in each incident triangle)
            this.fixedVertices = fixedVertices; // For each vertex i, indicates whether it is fixed in place
        }
    }
}