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
    public Dictionary<Vector2Int, List<int>> grid;

    public SPHFluid2D(Vector2[] predictedPosition, float[] density)
    {
        Bind(predictedPosition, density);
    }

    public void Bind(Vector2[] predictedPosition, float[] density)
    {
        _predPos = predictedPosition;
        _density = density;

        if (grid == null)
            grid = new Dictionary<Vector2Int, List<int>>();
    }

    public float DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;
        var baseCell = SpatialHash2D.CellCoord(samplePoint, smoothingRadius);

        for (int i = 0; i < neighborOffsets.Length; i++)
        {
            var cell = baseCell + neighborOffsets[i];

            if (!grid.TryGetValue(cell, out var indices)) continue;

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
        var baseCell = SpatialHash2D.CellCoord(_predPos[pointIndex], smoothingRadius);

        for (int i = 0; i < neighborOffsets.Length; i++)
        {
            var cell = baseCell + neighborOffsets[i];

            if (!grid.TryGetValue(cell, out var indices)) continue;

            for (int j = 0; j < indices.Count; j++)
            {
                int k = indices[j];
                if (pointIndex == k) continue;

                Vector2 offset = _predPos[k] - _predPos[pointIndex];
                float dst = offset.magnitude;
                Vector2 dir = (dst < 1e-6f) ? SPHMath2D.PseudoRandomUnitVector(pointIndex, k) : offset / dst;

                float slope = SPHMath2D.SmoothingKernelDerivative(smoothingRadius, dst);
                float pressureI = ConvertDensityToPressure(_density[pointIndex]);
                float pressureJ = ConvertDensityToPressure(_density[k]);
                float realDensity = Mathf.Max(_density[k], minDensity);
                pressureForce += (pressureI + pressureJ) / 2f * mass * slope * dir / realDensity;
            }
        }
        return pressureForce;
    }

    static readonly Vector2Int[] neighborOffsets = new Vector2Int[]
    {
        new(-1, -1), new(0, -1), new(1, -1),
        new(-1, 0), new(0, 0), new(1, 0),
        new(-1, 1), new(0, 1), new(1, 1),
    };
}