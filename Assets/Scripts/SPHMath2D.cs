using UnityEngine;

public static class SPHMath2D
{
    public static float Poly6Kernel(float h, float r)
    {
        float h2 = h * h;
        float r2 = r * r;

        float diff = h2 - r2;
        float volume = (315f / (64f * Mathf.PI * Mathf.Pow(h, 9)));

        return volume * diff * diff * diff;
    }

    public static float SpikyGradient(float h, float r)
    {
        float coeff = -45f / (Mathf.PI * Mathf.Pow(h, 6));
        float diff = h - r;

        return coeff * diff * diff;
    }

    public static float LaplacianKernel(float h, float r)
    {
        return 45f / (Mathf.PI * Mathf.Pow(h, 6)) * (h - r);
    }

    public static Vector2 PseudoRandomUnitVector(int a, int b)
    {
        uint h = (uint)(a * 73856093) ^ (uint)(b * 19349663) ^ 0x9E3779B9u;
        float t = h / (float)uint.MaxValue * (2f * Mathf.PI);
        return new Vector2(Mathf.Cos(t), Mathf.Sin(t));
    }
}