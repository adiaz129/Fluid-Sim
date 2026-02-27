using System.Collections.Generic;
using UnityEngine;

public class SPHFluid2D
{
    private Vector2[] _predPos;
    private float[] _density;
    private float[] _pressure;
    private Vector2[] _velocity;
    private int[] _neighbors;
    private SpatialHash2D _grid;


    public float mass = 1;
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float minDensity = 1e-4f;
    public float viscosityStrength = 0.5f;

    public SPHFluid2D(Vector2[] predictedPosition, float[] density, float[] pressure, Vector2[] velocity, int[] neighbors, SpatialHash2D grid)
    {
        Bind(predictedPosition, density, pressure, velocity, neighbors, grid);
    }

    public void Bind(Vector2[] predictedPosition, float[] density, float[] pressure, Vector2[] velocity, int[] neighbors, SpatialHash2D grid)
    {
        _predPos = predictedPosition;
        _density = density;
        _pressure = pressure;
        _velocity = velocity;
        _neighbors = neighbors;
        _grid = grid;
    }

    public float DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;
        var baseCell = _grid.CellCoord(samplePoint);
        float r2 = smoothingRadius * smoothingRadius;

        for (int i = 0; i < neighborOffsets.Length; i++)
        {
            var cell = baseCell + neighborOffsets[i];
            if (!_grid.IsCellInBounds(cell)) continue;
            int key = _grid.GetKey(cell);

            int start = _grid.cellStart[key];
            if (start < 0) continue;
            int end = _grid.cellEnd[key];

            for (int j = start; j < end; j++)
            {
                int k = _grid.spatialLookup[j].particleIndex;
                float sqrDist = (_predPos[k] - samplePoint).sqrMagnitude;
                if (sqrDist >= r2) continue;
                float dist = Mathf.Sqrt(sqrDist);
                float influence = SPHMath2D.Poly6Kernel(smoothingRadius, dist);
                resultDensity += mass * influence;
            }
        }
        return resultDensity;
    }

    public float ConvertDensityToPressure(float dens)
    {
        float densityError = dens - restDensity;
        float pressure = Mathf.Max(densityError * stiffness, 0f); //change this when adding near pressure (as in remove max)
        return pressure;
    }

    public Vector2 PressureAndViscosityAt(int pointIndex)
    {
        Vector2 totalForce = Vector2.zero; 
        var baseCell = _grid.CellCoord(_predPos[pointIndex]);
        float r2 = smoothingRadius * smoothingRadius;

        for (int i = 0; i < neighborOffsets.Length; i++)
        {
            var cell = baseCell + neighborOffsets[i];
            if (!_grid.IsCellInBounds(cell)) continue;
            int key = _grid.GetKey(cell);

            int start = _grid.cellStart[key];
            if (start < 0) continue;
            int end = _grid.cellEnd[key];
            
            for (int j = start; j < end; j++)
            {

                int k = _grid.spatialLookup[j].particleIndex;
                if (pointIndex == k) continue;

                Vector2 offset = _predPos[k] - _predPos[pointIndex];
                float sqrDist = offset.sqrMagnitude;
                
                if (sqrDist >= r2) continue;
                _neighbors[pointIndex]++;
                float dist = Mathf.Sqrt(sqrDist);
                float slope = SPHMath2D.SpikyGradient(smoothingRadius, dist);
                Vector2 dir = (dist < 1e-6f) ? SPHMath2D.PseudoRandomUnitVector(pointIndex, k) : offset / dist;

                float currDensity = Mathf.Max(_density[pointIndex], minDensity);
                float kDensity = Mathf.Max(_density[k], minDensity);

                float pressureTerm = (_pressure[pointIndex] + _pressure[k]) / (2 * currDensity * kDensity);
                totalForce += pressureTerm * mass * slope * dir;

                float laplacian = SPHMath2D.LaplacianKernel(smoothingRadius, dist);
                Vector2 velocityDiff = _velocity[k] - _velocity[pointIndex];

                totalForce += viscosityStrength * mass * velocityDiff / kDensity * laplacian;
            }
        }
        return totalForce;
    }

    static readonly Vector2Int[] neighborOffsets = new Vector2Int[]
    {
        new(-1, -1), new(0, -1), new(1, -1),
        new(-1, 0), new(0, 0), new(1, 0),
        new(-1, 1), new(0, 1), new(1, 1),
    };
}