using System;
using UnityEngine;

public static class SPHMath2D
{
    private static float h;
    private static float h2;
    private static float h4;
    private static float h5;
    private static float h8;
    private static float h10;
    public static void SetSmoothingRadius(float smoothingRadius)
    {
        h = smoothingRadius;
        h2 = h * h;
        h4 = h2 * h2;
        h5 = h4 * h;
        h8 = h4 * h4;
        h10 = h5 * h5;
    }
    public static (float influence, float nearInfluence) Poly6_2D(float r2)
    {
        float factor = 4f / (Mathf.PI * h8);
        float nearFactor = 5f / (Mathf.PI * h10);
        float term = h2 - r2;
 
        float term3 = term * term * term;

        return (factor * term3, nearFactor * term3 * term);
    }

    public static Vector2 Poly6Grad_2D(float r2, Vector2 rij)
    {
        float constant = 24f / (Mathf.PI * Mathf.Pow(h, 8));
        float factor = constant * Mathf.Pow(h2 - r2, 2);

        return -factor * rij;
    }

    public static Vector2 NearPoly6Grad_2D(float r2, Vector2 rij)
    {
        float term = h2 - r2;

        float factor = 40f / (Mathf.PI * Mathf.Pow(h, 10));

        return -factor * Mathf.Pow(term, 3) * rij;
    }


    public static float SpikyGrad_2D(float r)
    {
        return -10f / (Mathf.PI * h5) * (h - r) * (h - r);
    }

    public static float ViscosityLaplacian_2D(float r2)
    {
        float r = Mathf.Sqrt(r2);

        return 40f / (Mathf.PI * h5) * (h - r);
    }

    public static float Poly6Laplacian_2D(float r2)
    {
        float factor = 24f / (Mathf.PI * h8);
        return -factor * (h2 - r2) * (3f * h2 - 7f * r2);
    }

    public static Vector2 PseudoRandomUnitVector(int a, int b)
    {
        uint rand = (uint)(a * 73856093) ^ (uint)(b * 19349663) ^ 0x9E3779B9u;
        float t = rand / (float)uint.MaxValue * (2f * Mathf.PI);
        return new Vector2(Mathf.Cos(t), Mathf.Sin(t));
    }
}