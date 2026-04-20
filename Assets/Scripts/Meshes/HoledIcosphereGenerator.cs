using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Meshes {
    /// <summary>
    /// Generates an icosphere with holes.
    /// </summary>
    [CreateAssetMenu(menuName = "Minimal Surfaces/Generators/Holed Icosphere")]
    public class HoledIcosphereGenerator : MeshGeneratorBase {
        /// <summary>
        /// Strategy used to place hole centers on the sphere.
        /// </summary>
        public enum HoleLayout {
            /// <summary>
            /// Samples hole centers independently on the unit sphere.
            /// </summary>
            Random,

            /// <summary>
            /// Distributes hole centers uniformly on the XZ-plane.
            /// </summary>
            XZ_Plane
        }

        /// <summary>
        /// Radius of the initial icosphere before hole carving.
        /// </summary>
        [Header("Sphere Settings")] public float initialSphereRadius = 1f;

        /// <summary>
        /// Number of icosphere subdivision passes.
        /// </summary>
        public int subdivisions = 5;

        /// <summary>
        /// Sphere center used for analytic boundary realignment.
        /// </summary>
        public Vector3 sphereCenter;

        /// <summary>
        /// Placement mode for hole centers.
        /// </summary>
        [Header("Holes")] public HoleLayout holeLayout = HoleLayout.Random;

        /// <summary>
        /// Number of holes to carve.
        /// </summary>
        public int holeCount = 2;

        /// <summary>
        /// Radius used for vertex removal around each hole center.
        /// </summary>
        public float holeRadius = 0.1f;

        const float TAU = Mathf.PI * 2f;

        /// <summary>
        /// Builds the holed sphere.
        /// </summary>
        /// <returns>Generated mesh data.</returns>
        public override MeshData Generate() {
            Mesh mesh = IcoSphere.Create(initialSphereRadius, subdivisions);

            List<Vector3> holePositions = ComputeHolePositions();
            RemoveVerticesNearHoles(mesh, holePositions, holeRadius, out List<Vector3> remainingVertices, out int[] triangles);
            ComputeMeshConnectivity(remainingVertices.ToArray(), triangles, out HashSet<int>[] neighbors, out List<(int, int)>[] adjacency, out bool[] fixedVertices);
            AlignFixedVerticesToHoles(holePositions, remainingVertices, fixedVertices);

            mesh.Clear();
            mesh.SetVertices(remainingVertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();

            return new MeshData(mesh, mesh.vertices, mesh.triangles, neighbors, adjacency, fixedVertices);
        }

        /// <summary>
        /// Computes hole center positions according to <see cref="holeLayout"/>.
        /// </summary>
        /// <returns>Hole center positions in mesh-local space.</returns>
        List<Vector3> ComputeHolePositions() {
            List<Vector3> positions = new List<Vector3>();
            switch (holeLayout) {
                case HoleLayout.Random:
                    for (int i = 0; i < holeCount; i++) {
                        positions.Add(Random.onUnitSphere * initialSphereRadius);
                    }

                    break;
                case HoleLayout.XZ_Plane:
                    for (int i = 0; i < holeCount; i++) {
                        float angle = TAU * (i / (float)holeCount);
                        positions.Add(new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * initialSphereRadius);
                    }

                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            return positions;
        }

        /// <summary>
        /// Removes vertices inside the hole radius.
        /// </summary>
        /// <param name="mesh">Input mesh.</param>
        /// <param name="holePositions">Hole center positions.</param>
        /// <param name="radius">Removal radius around each hole center.</param>
        /// <param name="remainingVertices">Resulting compact vertex list.</param>
        /// <param name="remainingTriangles">Resulting remapped triangle index buffer.</param>
        void RemoveVerticesNearHoles(Mesh mesh, List<Vector3> holePositions, float radius, out List<Vector3> remainingVertices, out int[] remainingTriangles) {
            if (holePositions == null || holePositions.Count == 0) {
                remainingVertices = new List<Vector3>(mesh.vertices);
                remainingTriangles = mesh.triangles;
                return;
            }

            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            bool[] removeVertex = new bool[vertices.Length];

            for (int i = 0; i < vertices.Length; i++) {
                Vector3 worldPos = vertices[i];
                foreach (Vector3 hole in holePositions) {
                    if (Vector3.Distance(worldPos, hole) <= radius) {
                        removeVertex[i] = true;
                        break;
                    }
                }
            }

            // only keep triangles which are not marked as removed
            List<int> newTriangles = new List<int>();
            for (int i = 0; i < triangles.Length; i += 3) {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];
                if (removeVertex[a] || removeVertex[b] || removeVertex[c]) continue;

                newTriangles.Add(a);
                newTriangles.Add(b);
                newTriangles.Add(c);
            }

            // remap triangle indices
            Dictionary<int, int> vertexMap = new Dictionary<int, int>();
            remainingVertices = new List<Vector3>();
            for (int i = 0; i < newTriangles.Count; i++) {
                int oldIndex = newTriangles[i];
                if (!vertexMap.TryGetValue(oldIndex, out int mapped)) {
                    mapped = remainingVertices.Count;
                    remainingVertices.Add(vertices[oldIndex]);
                    vertexMap[oldIndex] = mapped;
                }

                newTriangles[i] = mapped;
            }

            remainingTriangles = newTriangles.ToArray();
        }

        /// <summary>
        /// Projects fixed boundary vertices onto analytic hole circles on the sphere.
        /// </summary>
        /// <remarks>
        /// The boundary curve is obtained from the circle of intersection between the base sphere
        /// and the sphere centered at the chosen hole center with radius <see cref="holeRadius"/>.
        /// </remarks>
        /// <param name="holePositions">Hole center positions.</param>
        /// <param name="vertices">Vertex buffer.</param>
        /// <param name="fixedVertices">Fixed/Boundary vertices.</param>
        void AlignFixedVerticesToHoles(List<Vector3> holePositions, List<Vector3> vertices, bool[] fixedVertices) {
            if (holePositions == null || holePositions.Count == 0) {
                return;
            }

            float r1 = initialSphereRadius;
            float r2 = holeRadius;

            for (int p = 0; p < fixedVertices.Length; p++) {
                if (fixedVertices[p]) {
                    Vector3 nearestHolePos = Vector3.positiveInfinity;
                    float nearestHoleDistance = float.MaxValue;

                    foreach (Vector3 holePos in holePositions) {
                        float distanceFromHole = Vector3.Distance(holePos, vertices[p]);
                        if (distanceFromHole < nearestHoleDistance) {
                            nearestHoleDistance = distanceFromHole;
                            nearestHolePos = holePos;
                        }
                    }

                    Vector3 c1 = sphereCenter;
                    Vector3 c2 = nearestHolePos;
                    float d = Vector3.Distance(c1, c2);
                    if (d <= 1e-05) continue;

                    float h = 0.5f + (r1 * r1 - r2 * r2) / (2f * d * d);

                    Vector3 c_i = c1 + h * (c2 - c1);
                    float r_i = Mathf.Sqrt(r1 * r1 - Mathf.Pow(h * d, 2));

                    Vector3 intersectingPlaneNormal = (c_i - c1).normalized;
                    Vector3 projectedOnPlane = Vector3.ProjectOnPlane(vertices[p] - c_i, intersectingPlaneNormal);

                    vertices[p] = c_i + projectedOnPlane.normalized * r_i;
                }
            }
        }
    }
}