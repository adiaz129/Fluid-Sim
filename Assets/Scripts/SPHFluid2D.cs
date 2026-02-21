using System.Collections.Generic;
using UnityEngine;

public class SPHFluid2D
{
    private Vector2[] _predPos;
    private float[] _density;
    private SpatialHash2D _grid;

    public float mass = 1;
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float minDensity = 1e-4f;

    public SPHFluid2D(Vector2[] predictedPosition, float[] density, SpatialHash2D grid)
    {
        Bind(predictedPosition, density, grid);
    }

    public void Bind(Vector2[] predictedPosition, float[] density, SpatialHash2D grid)
    {
        _predPos = predictedPosition;
        _density = density;
        _grid = grid;
    }

    public float DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;
        var baseCell = _grid.CellCoord(samplePoint);

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
                if (sqrDist >= smoothingRadius * smoothingRadius) continue;
                float dist = Mathf.Sqrt(sqrDist);
                float influence = SPHMath2D.SmoothingKernel(smoothingRadius, dist);
                resultDensity += mass * influence;
            }
        }
        return resultDensity;
    }

    public float ConvertDensityToPressure(float dens)
    {
        float densityError = dens - restDensity;
        float pressure = densityError * stiffness; //change this when adding near pressure (as in remove max)
        return pressure;
    }

    public Vector2 CalculatePressureForce(int pointIndex)
    {
        Vector2 pressureForce = Vector2.zero; 
        var baseCell = _grid.CellCoord(_predPos[pointIndex]);

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
                
                if (sqrDist >= smoothingRadius * smoothingRadius) continue;
                float dist = Mathf.Sqrt(sqrDist);
                float slope = SPHMath2D.SmoothingKernelDerivative(smoothingRadius, dist);
                Vector2 dir = (dist < 1e-6f) ? SPHMath2D.PseudoRandomUnitVector(pointIndex, k) : offset / dist;
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