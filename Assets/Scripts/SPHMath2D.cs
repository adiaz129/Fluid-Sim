using UnityEngine;

public static class SPHMath2D
{
    public static float Poly6_2D(float h, float r)
    {
        float h2 = h * h;
        float r2 = r * r;

        float h4 = h2 * h2;
        float h8 = h4 * h4;

        float factor = 4f / (Mathf.PI * h8);
        float term = h2 - r2;

        return factor * term * term * term;
    }

    public static Vector2 Poly6Grad_2D(float h, float r, Vector2 rij)
    {
        float constant = -24f / (Mathf.PI * Mathf.Pow(h, 8));
        float factor = constant * Mathf.Pow(h * h - r * r, 2);

        return factor * rij;
    }

    public static float SpikyGrad_2D(float h, float r)
    {
        float h5 = h * h * h * h * h;

        return -30f / (Mathf.PI * h5) * (h - r) * (h - r);
    }

    public static float ViscosityLaplacian_2D(float h, float r)
    {
        float h5 = h * h * h * h * h;

        return 20f / (Mathf.PI * h5) * (h - r);
    }

    public static float Poly6Laplacian_2D(float h, float r)
    {
        if (r >= h) return 0f;

        float h2 = h * h;
        float r2 = r * r;

        float h4 = h2 * h2;
        float h8 = h4 * h4;

        float factor = 24f / (Mathf.PI * h8);
        return factor * (h2 - r2) * (3f * r2 - h2);
    }

    public static Vector2 PseudoRandomUnitVector(int a, int b)
    {
        uint h = (uint)(a * 73856093) ^ (uint)(b * 19349663) ^ 0x9E3779B9u;
        float t = h / (float)uint.MaxValue * (2f * Mathf.PI);
        return new Vector2(Mathf.Cos(t), Mathf.Sin(t));
    }
}