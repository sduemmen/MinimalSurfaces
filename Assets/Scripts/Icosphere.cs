using System.Collections.Generic;
using UnityEngine;

public static class IcoSphere
{
    public static Mesh Create(float radius = 1.0f, int subdivisions = 2) {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        CreateBaseIcosahedron(vertices, triangles);
        Subdivide(vertices, triangles, subdivisions);
        ProjectToSphere(vertices, radius);

        Mesh mesh = new Mesh();
        mesh.name = $"IcoSphere_R{radius}_Sub{subdivisions}";
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    static void CreateBaseIcosahedron(List<Vector3> vertices, List<int> triangles) {
        vertices.Clear();
        triangles.Clear();

        float t = (1.0f + Mathf.Sqrt(5.0f)) * 0.5f;

        vertices.Add(new Vector3(-1, t, 0));
        vertices.Add(new Vector3(1, t, 0));
        vertices.Add(new Vector3(-1, -t, 0));
        vertices.Add(new Vector3(1, -t, 0));

        vertices.Add(new Vector3(0, -1, t));
        vertices.Add(new Vector3(0, 1, t));
        vertices.Add(new Vector3(0, -1, -t));
        vertices.Add(new Vector3(0, 1, -t));

        vertices.Add(new Vector3(t, 0, -1));
        vertices.Add(new Vector3(t, 0, 1));
        vertices.Add(new Vector3(-t, 0, -1));
        vertices.Add(new Vector3(-t, 0, 1));

        int[] faces = {
            0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
            1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
            3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
            4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
        };

        for (int i = 0; i < faces.Length; i++) {
            triangles.Add(faces[i]);
        }
    }

    static void Subdivide(List<Vector3> vertices, List<int> triangles, int subdivisions) {
        Dictionary<EdgeKey, int> midpointCache = new Dictionary<EdgeKey, int>();

        for (int i = 0; i < subdivisions; i++) {
            List<int> newTriangles = new List<int>();

            for (int tri = 0; tri < triangles.Count; tri += 3) {
                int v1 = triangles[tri];
                int v2 = triangles[tri + 1];
                int v3 = triangles[tri + 2];

                int a = GetOrCreateMidpoint(vertices, midpointCache, v1, v2);
                int b = GetOrCreateMidpoint(vertices, midpointCache, v2, v3);
                int c = GetOrCreateMidpoint(vertices, midpointCache, v3, v1);

                newTriangles.Add(v1);
                newTriangles.Add(a);
                newTriangles.Add(c);

                newTriangles.Add(v2);
                newTriangles.Add(b);
                newTriangles.Add(a);

                newTriangles.Add(v3);
                newTriangles.Add(c);
                newTriangles.Add(b);

                newTriangles.Add(a);
                newTriangles.Add(b);
                newTriangles.Add(c);
            }

            triangles.Clear();
            triangles.AddRange(newTriangles);
            midpointCache.Clear();
        }
    }

    static int GetOrCreateMidpoint(List<Vector3> vertices, Dictionary<EdgeKey, int> cache, int indexA, int indexB) {
        EdgeKey key = new EdgeKey(indexA, indexB);

        if (cache.TryGetValue(key, out int midpointIndex)) {
            return midpointIndex;
        }

        Vector3 v1 = vertices[indexA];
        Vector3 v2 = vertices[indexB];
        Vector3 midpoint = (v1 + v2) * 0.5f;

        int newIndex = vertices.Count;
        vertices.Add(midpoint);
        cache[key] = newIndex;
        return newIndex;
    }

    static void ProjectToSphere(List<Vector3> vertices, float radius) {
        for (int i = 0; i < vertices.Count; i++) {
            vertices[i] = vertices[i].normalized * radius;
        }
    }

    struct EdgeKey
    {
        public int a;
        public int b;

        public EdgeKey(int indexA, int indexB) {
            if (indexA < indexB) {
                a = indexA;
                b = indexB;
            } else {
                a = indexB;
                b = indexA;
            }
        }

        public override bool Equals(object obj) {
            if (!(obj is EdgeKey)) {
                return false;
            }

            EdgeKey other = (EdgeKey)obj;
            return a == other.a && b == other.b;
        }

        public override int GetHashCode() {
            unchecked {
                return (a * 397) ^ b;
            }
        }
    }
}