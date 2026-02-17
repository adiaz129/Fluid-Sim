using UnityEngine;

public class SPHSolver2D
{
    private Vector2[] _predPos;
    private float[] _density;

    public float mass = 1;
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float minDensity = 1e-4f;

    public SPHSolver2D(Vector2[] predictedPosition, float[] density)
    {
        Bind(predictedPosition, density);
    }

    public void Bind(Vector2[] predictedPosition, float[] density)
    {
        _predPos = predictedPosition;
        _density = density;
    }

    public float DensityAt(Vector2 samplePoint)
    {
        float resultDensity = 0;

        for (int i = 0; i < _predPos.Length; i++)
        {
            float dist = (_predPos[i] - samplePoint).magnitude;
            float influence = SPHMath2D.SmoothingKernel(smoothingRadius, dist);
            resultDensity += mass * influence;
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
}