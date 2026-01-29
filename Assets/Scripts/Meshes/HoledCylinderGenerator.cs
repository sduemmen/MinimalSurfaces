using System.Collections.Generic;
using UnityEngine;

namespace Meshes {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Generators/Holed Cylinder")]
    public class HoledCylinderGenerator : MeshGeneratorBase {
        public float baseDiameter = 2f;
        public float height = 2f;
        public Vector3 rotationEuler;

        public int thetaSubdivisions = 16;
        public int ySubdivisions = 7;
        public bool offsetVertices;

        const float TAU = Mathf.PI * 2f;


        public override MeshData Generate() {
            float r = baseDiameter * 0.5f;
            float halfHeight = height * 0.5f;

            int vertexCount = (ySubdivisions + 1) * thetaSubdivisions;
            List<Vector3> vertices = new List<Vector3>(vertexCount);
            List<int> triangles = new List<int>(ySubdivisions * thetaSubdivisions * 6);

            float dTheta = TAU / thetaSubdivisions;
            Quaternion rotation = Quaternion.Euler(rotationEuler);

            for (int yIndex = 0; yIndex < ySubdivisions + 1; yIndex++) {
                float t = yIndex / (float)ySubdivisions;
                float y = Mathf.Lerp(-halfHeight, halfHeight, t);

                for (int thetaIndex = 0; thetaIndex < thetaSubdivisions; thetaIndex++) {
                    float theta = thetaIndex * dTheta;

                    if (offsetVertices) {
                        theta += yIndex * dTheta * 0.5f;
                    }

                    float x = Mathf.Cos(theta) * r;
                    float z = Mathf.Sin(theta) * r;
                    Vector3 p = new Vector3(x, y, z);
                    vertices.Add(rotation * p);
                }
            }

            for (int yIndex = 0; yIndex < ySubdivisions; yIndex++) {
                int ringStart = yIndex * thetaSubdivisions;
                int nextRingStart = (yIndex + 1) * thetaSubdivisions;

                for (int thetaIndex = 0; thetaIndex < thetaSubdivisions; thetaIndex++) {
                    int current = ringStart + thetaIndex;
                    int next = ringStart + (thetaIndex + 1) % thetaSubdivisions;
                    int above = nextRingStart + thetaIndex;
                    int aboveNext = nextRingStart + (thetaIndex + 1) % thetaSubdivisions;

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