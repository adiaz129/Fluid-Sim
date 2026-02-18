using System.Collections.Generic;
using UnityEngine;

public class SPHFluid2D
{
    private Vector2[] _predPos;
    private float[] _density;

    public float mass = 1;
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float minDensity = 1e-4f;
    private Dictionary<int, List<int>> grid;

    public SPHFluid2D(Vector2[] predictedPosition, float[] density)
    {
        Bind(predictedPosition, density);
    }

    public void Bind(Vector2[] predictedPosition, float[] density)
    {
        _predPos = predictedPosition;
        _density = density;

        if (grid == null)
            grid = new Dictionary<int, List<int>>(_predPos != null ? _predPos.Length : 0);
    }

    public float DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;
        var baseCell = cellCoord(samplePoint, smoothingRadius);

        for (int i = 0; i < neighborOffsets.Length; i++)
        {
            var cell = baseCell + neighborOffsets[i];
            int key = Hash(cell);

            if (!grid.TryGetValue(key, out var indices)) continue;

            for (int j = 0; j < indices.Count; j++)
            {
                int k = indices[j];
                float dist = (_predPos[k] - samplePoint).magnitude;
                float influence = SPHMath2D.SmoothingKernel(smoothingRadius, dist);
                resultDensity += mass * influence;
            }
        }
        return resultDensity;
    }

    public float ConvertDensityToPressure(float dens)
    {
        float densityError = dens - restDensity;
        float pressure = Mathf.Max(0f, densityError * stiffness); //change this when adding near pressure (as in remove max)
        return pressure;
    }

    public Vector2 CalculatePressureForce(int pointIndex)
    {
        Vector2 pressureForce = Vector2.zero;

        for (int i = 0; i < _predPos.Length; i++)
        {
            if (i == pointIndex) continue;

            Vector2 offset = _predPos[i] - _predPos[pointIndex];
            float dst = offset.magnitude;
            Vector2 dir = (dst < 1e-6f) ? SPHMath2D.PseudoRandomUnitVector(pointIndex, i) : offset / dst;

            float slope = SPHMath2D.SmoothingKernelDerivative(smoothingRadius, dst);
            float pressureI = ConvertDensityToPressure(_density[pointIndex]);
            float pressureJ = ConvertDensityToPressure(_density[i]);
            float realDensity = Mathf.Max(_density[i], minDensity);
            pressureForce += (pressureI + pressureJ) / 2f * mass * slope * dir / realDensity;
        }
        return pressureForce;
    }

    // make a function that finds what grid a position belongs in
    static Vector2Int cellCoord(Vector2 p, float cellSize)
    {
        return new Vector2Int(
            Mathf.FloorToInt(p.x / cellSize),
            Mathf.FloorToInt(p.y / cellSize)
        );
    }

    // make a function that hashes the grid cell and returns key
    static int Hash(Vector2Int c)
    {
        return c.x * 73856093 ^ c.y * 19349663;
    }

    // make a function that builds the grid
    public void BuildGrid()
    {
        grid.Clear();
        for (int i = 0; i < _predPos.Length; i++)
        {
            var cell = cellCoord(_predPos[i], smoothingRadius);
            int key = Hash(cell);
            if (!grid.TryGetValue(key, out var list))
            {
                list = new List<int>(16);
                grid[key] = list;
            }
            list.Add(i);
        }
    }

    static Vector2Int[] neighborOffsets = new Vector2Int[]
    {
        new(-1, -1), new(0, -1), new(1, -1),
        new(-1, 0), new(0, 0), new(1, 0),
        new(-1, 1), new(0, 1), new(1, 1),
    };
}