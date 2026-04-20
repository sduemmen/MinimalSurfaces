using System;
using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    /// <summary>
    /// Generates a catenoid mesh
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Generators/Catenoid")]
    public class CatenoidGenerator : MeshGeneratorBase {
        /// <summary>
        /// Catenoid parameter in <c>r(y) = a cosh(y / a)</c>
        /// </summary>
        public float a = 0.5f;

        /// <summary>
        /// Total extent in the vertical direction
        /// </summary>
        public float height = 2f;

        /// <summary>
        /// Euler rotation applied after generation
        /// </summary>
        public Vector3 rotationEuler;

        /// <summary>
        /// Number of segments per ring
        /// </summary>
        public int N_x = 16;

        /// <summary>
        /// Number of strips between bottom and top ring
        /// </summary>
        public int N_y = 7;

        /// <summary>
        /// Offsets successive rings by half a segment
        /// </summary>
        public bool offsetVertices;

        const float TAU = Mathf.PI * 2f;

        /// <summary>
        /// Builds a triangulated catenoid
        /// </summary>
        /// <returns>Generated mesh data.</returns>
        public override MeshData Generate() {
            float halfHeight = height * 0.5f;

            int vertexCount = (N_y + 1) * N_x;
            List<Vector3> vertices = new List<Vector3>(vertexCount);
            List<int> triangles = new List<int>(N_y * N_x * 6);

            float dTheta = TAU / N_x;
            Quaternion rotation = Quaternion.Euler(rotationEuler);

            for (int yIndex = 0; yIndex < N_y + 1; yIndex++) {
                float t = yIndex / (float)N_y;
                float y = Mathf.Lerp(-halfHeight, halfHeight, t);
                double radius = a * Math.Cosh(y / a);

                for (int thetaIndex = 0; thetaIndex < N_x; thetaIndex++) {
                    float theta = thetaIndex * dTheta;

                    if (offsetVertices) {
                        theta += yIndex * dTheta * 0.5f;
                    }

                    double x = radius * Mathf.Cos(theta);
                    double z = radius * Mathf.Sin(theta);
                    Vector3 p = new Vector3((float)x, y, (float)z);
                    vertices.Add(rotation * p);
                }
            }

            // Connect each pair of consecutive rings with two triangles per angular segment
            for (int yIndex = 0; yIndex < N_y; yIndex++) {
                int ringStart = yIndex * N_x;
                int nextRingStart = (yIndex + 1) * N_x;

                for (int thetaIndex = 0; thetaIndex < N_x; thetaIndex++) {
                    int current = ringStart + thetaIndex;
                    int next = ringStart + (thetaIndex + 1) % N_x;
                    int above = nextRingStart + thetaIndex;
                    int aboveNext = nextRingStart + (thetaIndex + 1) % N_x;

                    triangles.Add(current);
                    triangles.Add(above);
                    triangles.Add(next);

                    triangles.Add(next);
                    triangles.Add(above);
                    triangles.Add(aboveNext);
                }
            }

            int[] triangleArray = triangles.ToArray();
            ComputeMeshConnectivity(vertices.ToArray(), triangleArray, out HashSet<int>[] neighbors, out List<(int, int)>[] triangleNeighborPairsByVertex, out bool[] fixedVertices);

            Mesh mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangleArray, 0);
            mesh.RecalculateNormals();

            return new MeshData(mesh, mesh.vertices, mesh.triangles, neighbors, triangleNeighborPairsByVertex, fixedVertices);
        }
    }
}