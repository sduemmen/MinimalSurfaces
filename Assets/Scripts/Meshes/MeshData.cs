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

        public void ColorVerticesByMeanCurvature(float maxAbsH = 0) {
            ColorVerticesByMeanCurvature(out _, maxAbsH);
        }

        public void ColorVerticesByMeanCurvature(out float currentMaxAbsH, float maxAbsH = 0) {
            currentMaxAbsH = 0f;
            if (mesh == null || vertices == null || triangleNeighborPairsByVertex == null) return;

            int n = vertices.Length;
            float[] H = new float[n];

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

        static float CotanBetween(Vector3 u, Vector3 v) {
            return Vector3.Dot(u, v) / Vector3.Cross(u, v).magnitude;
        }
    }
}