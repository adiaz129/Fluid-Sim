using System;
using System.Collections.Generic;
using UnityEngine;

public class SPHFluid2D
{
    private Vector2[] _predPos;
    private float[] _density;
    private float[] _nearDensity;
    private float[] _pressure;
    private float[] _nearPressure;
    private Vector2[] _velocity;
    private int[] _neighbors;
    private float[] _normals;
    private SpatialHash2D _grid;


    public float mass = 1;
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float nearStiffness = 22;
    public float minDensity = 1e-4f;
    public float viscosityStrength = 0.5f;
    public float surfaceThreshold = 0.2f;
    public float surfTensMultiplier = 3f;


    public SPHFluid2D(Vector2[] predictedPosition, float[] density, float[] nearDensity, float[] pressure, float[] nearPressure, Vector2[] velocity, int[] neighbors, float[] normals, SpatialHash2D grid)
    {
        Bind(predictedPosition, density, nearDensity, pressure, nearPressure, velocity, neighbors, normals, grid);
    }

    public void Bind(Vector2[] predictedPosition, float[] density, float[] nearDensity, float[] pressure, float[] nearPressure, Vector2[] velocity, int[] neighbors, float[] normals, SpatialHash2D grid)
    {
        _predPos = predictedPosition;
        _density = density;
        _nearDensity = nearDensity;
        _pressure = pressure;
        _nearPressure = nearPressure;
        _velocity = velocity;
        _neighbors = neighbors;
        _normals = normals;
        _grid = grid;
    }

    public (float resultDensity, float resultNearDensity) DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;
        float resultNearDensity = 0;
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
                var (influence, nearInfluence) = SPHMath2D.Poly6_2D(sqrDist);
                resultDensity += mass * influence;
                resultNearDensity += mass * nearInfluence;
            }
        }
        return (resultDensity, resultNearDensity);
    }

    public (float pressure, float nearPressure) ConvertDensityToPressure(float dens, float nearDens)
    {
        float densityError = dens - restDensity;
        float pressure = densityError * stiffness;
        float nearPressure = nearDens * nearStiffness;
        return (pressure, nearPressure);
    }

    public Vector2 PressureAndViscosityAt(int pointIndex)
    {
        Vector2 totalForce = Vector2.zero;
        Vector2 normal = Vector2.zero;
        float massOverDensity;
        float curvature = 0f;
        var baseCell = _grid.CellCoord(_predPos[pointIndex]);
        float r2 = smoothingRadius * smoothingRadius;
        float currDensity = Mathf.Max(_density[pointIndex], minDensity);
        float currNearDensity = Mathf.Max(_nearDensity[pointIndex], minDensity);

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
                Vector2 slope = SPHMath2D.Poly6Grad_2D(sqrDist, offset);
                Vector2 nearSlope = SPHMath2D.NearPoly6Grad_2D(sqrDist, offset);

                float kDensity = Mathf.Max(_density[k], minDensity);
                float kNearDensity = Mathf.Max(_nearDensity[k], minDensity);

                float pressureTerm = (_pressure[pointIndex] + _pressure[k]) / (2 * currDensity * kDensity);
                totalForce += pressureTerm * mass * slope;

                float nearPressureTerm = (_nearPressure[pointIndex] + _nearPressure[k]) / (2 * currNearDensity * kNearDensity);
                totalForce += nearPressureTerm * mass * nearSlope;

                float laplacian = SPHMath2D.Poly6Laplacian_2D(sqrDist);
                Vector2 velocityDiff = _velocity[k] - _velocity[pointIndex];

                massOverDensity = mass / kDensity;
                normal += massOverDensity * slope;
                curvature += massOverDensity * laplacian;

                float vLaplacian = SPHMath2D.ViscosityLaplacian_2D(sqrDist);
                totalForce += viscosityStrength * mass * velocityDiff / kDensity * vLaplacian;
            }
        }
        _normals[pointIndex] = normal.sqrMagnitude;

        Vector2 surfaceForce = Vector2.zero;

        if (_normals[pointIndex] > Mathf.Max(surfaceThreshold, minDensity))
        {
            Vector2 nHat = normal / Mathf.Sqrt(_normals[pointIndex]);
            surfaceForce = -surfTensMultiplier * curvature * nHat;
            surfaceForce /= currDensity;
        }
        return totalForce + surfaceForce;
    }

    static readonly Vector2Int[] neighborOffsets = new Vector2Int[]
    {
        new(-1, -1), new(0, -1), new(1, -1),
        new(-1, 0), new(0, 0), new(1, 0),
        new(-1, 1), new(0, 1), new(1, 1),
    };
}