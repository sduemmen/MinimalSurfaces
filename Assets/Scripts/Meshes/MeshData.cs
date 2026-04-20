using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    /// <summary>
    /// Stores mesh and connectivity information.
    /// </summary>
    public class MeshData {
        /// <summary>
        /// Unity mesh instance visualized in the scene
        /// </summary>
        public Mesh mesh;

        /// <summary>
        /// Vertex positions
        /// </summary>
        public Vector3[] vertices;

        /// <summary>
        /// Triangle index buffer, grouped by triples
        /// </summary>
        public int[] triangles;

        /// <summary>
        /// For each vertex (index) <c>i</c>, store the corresponding 1-ring neighbor set
        /// </summary>
        public HashSet<int>[] neighbors;

        /// <summary>
        /// For each vertex (index) <c>i</c>, store pairs <c>(a,b)</c> such that <c>(i,a,b)</c> is an incident triangle
        /// </summary>
        public List<(int, int)>[] triangleNeighborPairsByVertex;

        /// <summary>
        /// Marks constrained boundary vertices.
        /// </summary>
        public bool[] fixedVertices;

        /// <summary>
        /// Creates a new MeshData object.
        /// </summary>
        /// <param name="mesh">Unity mesh instance</param>
        /// <param name="vertices">Vertex positions.</param>
        /// <param name="triangles">Triangle index buffer</param>
        /// <param name="neighbors">1-ring neighborhood per vertex</param>
        /// <param name="triangleNeighborPairsByVertex">Incident triangle neighbor pairs per vertex</param>
        /// <param name="fixedVertices">Boundary constraint mask</param>
        public MeshData(Mesh mesh, Vector3[] vertices, int[] triangles, HashSet<int>[] neighbors, List<(int, int)>[] triangleNeighborPairsByVertex, bool[] fixedVertices) {
            this.mesh = mesh;
            this.vertices = vertices;
            this.triangles = triangles;
            this.neighbors = neighbors;
            this.triangleNeighborPairsByVertex = triangleNeighborPairsByVertex;
            this.fixedVertices = fixedVertices;
        }

        /// <summary>
        /// Colors vertices by signed mean curvature and discard the new computed maximum absolute curvature
        /// </summary>
        /// <param name="maxAbsH">Optional fixed normalization scale</param>
        public void ColorVerticesByMeanCurvature(float maxAbsH = 0) {
            ColorVerticesByMeanCurvature(out _, maxAbsH);
        }

        /// <summary>
        /// Colors vertices by signed mean curvature using a Laplace-Beltrami estimate.
        /// </summary>
        /// <param name="currentMaxAbsH">Returns the maximum absolute curvature found in the current mesh state.</param>
        /// <param name="maxAbsH"> Optional fixed normalization scale. If zero, the maximum absolute curvature of this frame is used </param>
        public void ColorVerticesByMeanCurvature(out float currentMaxAbsH, float maxAbsH = 0) {
            currentMaxAbsH = 0f;
            if (mesh == null || vertices == null || triangleNeighborPairsByVertex == null) return;

            int n = vertices.Length;
            float[] H = new float[n]; // for a vertex (index) i, store its signed mean curvature

            for (int i = 0; i < n; i++) {
                float A_i = 0f;
                Vector3 beltrami = Vector3.zero;

                foreach ((int a, int b) in triangleNeighborPairsByVertex[i]) {
                    Vector3 x_i = vertices[i];
                    Vector3 x_a = vertices[a];
                    Vector3 x_b = vertices[b];
                    Vector3 ba = x_a - x_b;
                    Vector3 ai = x_i - x_a;
                    Vector3 bi = x_i - x_b;

                    float A = 0.5f * Vector3.Cross(ba, bi).magnitude;
                    if (A < 1e-08) continue;

                    float cotAlpha = CotanBetween(ai, -ba);
                    float cotBeta = CotanBetween(bi, ba);

                    beltrami += cotBeta * (x_a - x_i);
                    beltrami += cotAlpha * (x_b - x_i);

                    bool obtuseAtI = Vector3.Dot(ai, bi) < 0f;
                    bool obtuseAtA = Vector3.Dot(ai, -ba) < 0f;
                    bool obtuseAtB = Vector3.Dot(bi, ba) < 0f;

                    // mixed Voronoi area
                    if (obtuseAtI) {
                        A_i += 0.5f * A;
                    } else if (obtuseAtA || obtuseAtB) {
                        A_i += 0.25f * A;
                    } else {
                        A_i += (cotBeta * ai.sqrMagnitude + cotAlpha * bi.sqrMagnitude) / 8f;
                    }
                }

                if (A_i < 1e-08) continue;

                beltrami /= 2f * A_i;
                H[i] = 0.5f * Vector3.Dot(beltrami, mesh.normals[i]);
            }

            // Approximate the mean curvature of fixed/boundary vertices by the average of neighbor mean curvature values
            for (int i = 0; i < n; i++) {
                if (!fixedVertices[i]) continue;

                float sum = 0f;
                int count = 0;
                foreach (int neighbor in neighbors[i]) {
                    if (fixedVertices[neighbor]) continue;

                    sum += H[neighbor];
                    count++;
                }

                if (count > 0) {
                    H[i] = sum / count;
                }
            }

            for (int i = 0; i < n; i++) {
                currentMaxAbsH = Mathf.Max(currentMaxAbsH, Mathf.Abs(H[i]));
            }

            if (maxAbsH == 0) {
                maxAbsH = currentMaxAbsH;
            }

            Color[] colors = new Color[n];
            for (int i = 0; i < n; i++) {
                float normalizedH = H[i] / maxAbsH;
                colors[i] = normalizedH < 0f
                    ? Color.Lerp(Color.white, Color.blue, -normalizedH)
                    : Color.Lerp(Color.white, Color.red, normalizedH);
            }

            mesh.SetColors(colors);
        }

        /// <summary>
        /// Computes <c>cot(theta)</c> between two vectors.
        /// </summary>
        /// <param name="u">First vector</param>
        /// <param name="v">Second vector</param>
        /// <returns><c>u.v/|u x v|</c></returns>
        static float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}