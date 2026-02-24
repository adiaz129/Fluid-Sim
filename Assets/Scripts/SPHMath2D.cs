using UnityEngine;

public static class SPHMath2D
{
    public static float SmoothingKernel(float radius, float dist)
    {
        float volume = Mathf.PI * Mathf.Pow(radius, 4) / 6;
        return (radius - dist) * (radius - dist) / volume;
    }

    public static float SmoothingKernelDerivative(float radius, float dist)
    {
        float scale = 12f / (Mathf.PI * Mathf.Pow(radius, 4));
        return (dist - radius) * scale;
    }

    public static Vector2 PseudoRandomUnitVector(int a, int b)
    {
        uint h = (uint)(a * 73856093) ^ (uint)(b * 19349663) ^ 0x9E3779B9u;
        float t = h / (float)uint.MaxValue * (2f * Mathf.PI);
        return new Vector2(Mathf.Cos(t), Mathf.Sin(t));
    }
}