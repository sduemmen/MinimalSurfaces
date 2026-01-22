using System.Collections.Generic;
using System.Linq;
using Parabox.Stl;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

namespace Meshes {
    [CreateAssetMenu(menuName = "Minimal Surfaces/Generators/STL Mesh Loader")]
    public class StlMeshLoader : MeshGeneratorBase {
        public string FilePath { get; set; }

        public override MeshData Generate() {
            if (string.IsNullOrEmpty(FilePath)) return null;

            Mesh mesh = Importer.Import(FilePath, CoordinateSpace.Left)[0];

            Vector3[] importedVertices = mesh.vertices;
            int[] importedTriangles = mesh.triangles;

            List<Vector3> vertices = new List<Vector3>();
            Dictionary<int, int> oldToNew = new Dictionary<int, int>();

            for (int i = 0; i < importedVertices.Length; i++) {
                Vector3 p = importedVertices[i];

                bool duplicateFound = false;
                for (int j = 0; j < vertices.Count; j++) {
                    if (Vector3.Distance(p, vertices[j]) < 1e-05) {
                        oldToNew[i] = j;
                        duplicateFound = true;
                        break;
                    }
                }

                if (!duplicateFound) {
                    oldToNew[i] = vertices.Count;
                    vertices.Add(p);
                }
            }

            int[] triangles = importedTriangles.Select(oldIndex => oldToNew[oldIndex]).ToArray();
            ComputeMeshConnectivity(vertices.ToArray(), triangles, out HashSet<int>[] neighbors, out List<(int, int)>[] adjacency, out bool[] fixedVertices);

            mesh.Clear();
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();

            return new MeshData(mesh, mesh.vertices, mesh.triangles, neighbors, adjacency, fixedVertices);
        }
    }
}