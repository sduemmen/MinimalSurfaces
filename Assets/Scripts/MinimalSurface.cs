using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class MinimalSurface : MonoBehaviour
{
    public enum HoleLayout
    {
        Random,
        XZ_Plane,
    }
    
    [Header("Holes")] 
    [SerializeField] HoleLayout _holeLayout;
    [SerializeField] List<Vector3> _holePositions;
    [SerializeField] int _holeCount = 2;
    [SerializeField] float _holeRadius = 0.1f;

    [Header("Relaxation")] 
    public float dt = 1.0f;
    public float epsilon = 0.00001f;
    public int maxIterations = 5000;
    public float iterationsPerSecond = 60f;

    List<Vector3> _vertices;
    int[] _triangles;
    List<(int, int)>[] _adjacentTriangleVertices;
    HashSet<(int, int)> _edges;
    bool[] _fixed;
    Mesh _mesh;
    Mesh _sphereMesh;
    [SerializeField] MeshFilter _meshFilter;
    
    const float TAU = Mathf.PI * 2;

    void Awake() {
        _meshFilter = GetComponent<MeshFilter>();
        MakeNewMesh();
        StartRelaxation();
    }

    void MakeNewMesh() {
        _meshFilter.sharedMesh = IcoSphere.Create(1f, 4);
        _mesh = _meshFilter.sharedMesh;
    }

    void ComputeHolePositions() {
        _holePositions.Clear();

        switch (_holeLayout) {
            case HoleLayout.Random:
                for (int i = 0; i < _holeCount; i++) {
                    Vector3 holePosition = Random.onUnitSphere;
                    _holePositions.Add(holePosition);
                }
                break;
            case HoleLayout.XZ_Plane:
                for (int i = 0; i < _holeCount; i++) {
                    float angle = TAU * ((float) i / _holeCount);
                    Vector3 holePosition = new Vector3(Mathf.Cos(angle), this.transform.position.y, Mathf.Sin(angle));
                    _holePositions.Add(holePosition);
                }
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    [ContextMenu("Start Relaxation")]
    void StartRelaxation() {
        StopAllCoroutines();
        
        MakeNewMesh();
        ComputeHolePositions();
        RemoveVerticesNearHoles();
        ComputeVertexAdjacencyAndEdges();
        IdentifyFixedVertices();
        
        StartCoroutine(RunRelaxation());
    }

    IEnumerator RunRelaxation() {
        yield return RelaxationCoroutine();
    }

    void RemoveVerticesNearHoles() {
        if (_holePositions == null || _holePositions.Count == 0) {
            return;
        }
        
        Vector3[] vertices = _mesh.vertices;
        int[] triangles = _mesh.triangles;
        int initialVertexCount = vertices.Length;
        int initialTriangleCount = triangles.Length;

        // remove vertices in the vicinity of holes
        bool[] removeVertex = new bool[initialVertexCount];
        
        for (int i = 0; i < initialVertexCount; i++) {
            bool shouldRemove = false;
            Vector3 vertexWorldPos = _meshFilter.transform.TransformPoint(vertices[i]);
            
            foreach (Vector3 holePos in _holePositions) {
                float distanceFromHole = Vector3.Distance(vertexWorldPos, holePos);
                if (distanceFromHole <= _holeRadius) {
                    shouldRemove = true;
                    break;
                }
            }
            
            removeVertex[i] = shouldRemove;
        }

        // only keep triangles that don't include removed vertices
        List<int> newTriangles = new List<int>();
        
        for (int i = 0; i < initialTriangleCount; i += 3) {
            int a = triangles[i];
            int b = triangles[i + 1];
            int c = triangles[i + 2];

            if (removeVertex[a] || removeVertex[b] || removeVertex[c]) {
                continue;
            }

            newTriangles.Add(a);
            newTriangles.Add(b);
            newTriangles.Add(c);
        }

        // map old vertex indices to new ones
        Dictionary<int, int> vertexIndexMap = new Dictionary<int, int>();
        _vertices = new List<Vector3>();
        int newIndex = 0;

        for (int i = 0; i < newTriangles.Count; i++) {
            int oldIndex = newTriangles[i];
            
            if (!vertexIndexMap.TryGetValue(oldIndex, out int mappedIndex)) {
                Vector3 v = vertices[oldIndex];
                _vertices.Add(v);
                vertexIndexMap[oldIndex] = newIndex;
                newTriangles[i] = newIndex;
                newIndex++;
            } else {
                newTriangles[i] = mappedIndex;
            }
        }
        
        _triangles = newTriangles.ToArray();

        // finalize mesh
        _mesh.Clear();
        _mesh.SetVertices(_vertices);
        _mesh.triangles = _triangles;
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    void ComputeVertexAdjacencyAndEdges() {
        _adjacentTriangleVertices = new List<(int, int)>[_vertices.Count];
        _edges = new HashSet<(int, int)>();
        
        for (int i = 0; i < _vertices.Count; i++) {
            _adjacentTriangleVertices[i] = new List<(int, int)>();
        }

        for (int i = 0; i < _triangles.Length; i += 3) {
            int a = _triangles[i];
            int b = _triangles[i + 1];
            int c = _triangles[i + 2];

            _adjacentTriangleVertices[a].Add((b, c));
            _adjacentTriangleVertices[b].Add((c, a));
            _adjacentTriangleVertices[c].Add((a, b));

            _edges.Add((a, b));
            _edges.Add((b, c));
            _edges.Add((c, a));
        }
    }

    void IdentifyFixedVertices() {
        _fixed = new bool[_vertices.Count];

        foreach ((int, int) edge in _edges) {
            if (!_edges.Contains((edge.Item2, edge.Item1)))
            {
                _fixed[edge.Item1] = true;
                _fixed[edge.Item2] = true;
            }
        }
    }

    IEnumerator RelaxationCoroutine() {
        Vector3[] displacement = new Vector3[_vertices.Count];

        float timePerIteration = 1f / iterationsPerSecond;
        int iterations = 0;

        while (iterations < maxIterations) {
            for (int p = 0; p < _vertices.Count; p++) {
                displacement[p] = Vector3.zero;
                List<(int, int)> adjacentVertexPairs = _adjacentTriangleVertices[p];
                
                if (_fixed[p] || adjacentVertexPairs.Count == 0) {
                    continue;
                }

                Vector3 c = Vector3.zero;
                foreach ((int, int) pair in adjacentVertexPairs) {
                    c += _vertices[pair.Item1] + _vertices[pair.Item2];
                }

                c /= 2f * adjacentVertexPairs.Count;
                displacement[p] = c - _vertices[p];
            }

            float maxGrad = 0f;

            for (int p = 0; p < _vertices.Count; p++) {
                if (_fixed[p]) {
                    continue;
                }

                float d = displacement[p].magnitude;
                if (d > maxGrad) {
                    maxGrad = d;
                }

                _vertices[p] += dt * displacement[p];
            }

            _mesh.SetVertices(_vertices);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();

            iterations++;

            if (maxGrad < epsilon) {
                Debug.Log($"MinimalSurface: finished after {iterations} iterations");
                yield break;
            }

            yield return new WaitForSeconds(timePerIteration);
        }

        Debug.Log($"MinimalSurface: stopped after maxIterations={maxIterations}");
    }
}