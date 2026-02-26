using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.U2D.IK;

public class Simulation2D : MonoBehaviour
{
    [Header("Spawn")]
    public int numParticles = 8000;
    public float particleSize = 0.04f;
    public float particleSpacing = 0.1f;
    public Mesh particleMesh;
    public Material particleMaterial;

    [Header("Bounds")]
    public Vector2 boundsSize = new Vector2(48f, 24f);
    public float collisionDamping = 0.6f;
    public GameObject boundsPrefab;

    [Header("SPH")]
    public Vector2 gravity = new Vector2(0, -9.81f);
    public float smoothingRadius = 0.35f;
    public float restDensity = 12f;
    public float stiffness = 400f;
    public float viscosityStrength = 0.5f;
    public float mass = 1;
    public float minDensity = 1e-4f;

    [Header("Simulation Controls")]
    public float interactRadius = 0.8f;
    public float interactStrength = 35f; // hold LMB attract, RMB repel
    public enum SimulationFrequency
    {
        Hz50,
        Hz100,
        Hz150
    }
    [SerializeField]
    private SimulationFrequency simulationFrequency;

    Vector2[] position;
    Vector2[] velocity;
    Vector2[] predictedPosition;
    float[] density;
    float[] pressure;
    Transform bounds;

    bool isRunning = false;

    int lastNumParticles;
    float lastParticleSpacing;

    private SPHFluid2D fluid;

    private SpatialHash2D grid;

    private Matrix4x4[] matrices;

    void Start() // runs once
    {
        SpawnBounds();
        lastNumParticles = numParticles;
        lastParticleSpacing = particleSpacing;
        RebuildFluid();
    }

    void FixedUpdate()
    {
        boundsSize.x = Mathf.Max(particleSize * 2f, boundsSize.x);
        boundsSize.y = Mathf.Max(particleSize * 2f, boundsSize.y);
        bounds.localScale = Vector3.one * boundsSize;

        if (!isRunning) return;


        int substeps = Mathf.CeilToInt(GetFrequency() * Time.fixedDeltaTime);
        float dt = Time.fixedDeltaTime / substeps;

        for (int i = 0; i < substeps; i++)
        {
            SimulationStep(dt);
        }
    }

    void Update()
    {
        if (!isRunning)
        {
            if (numParticles != lastNumParticles || particleSpacing != lastParticleSpacing)
            {
                lastNumParticles = numParticles;
                lastParticleSpacing = particleSpacing;
                RebuildFluid();
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                isRunning = true;
            }
        }
        for (int i = 0; i < numParticles; i++)
        {
            matrices[i] = Matrix4x4.TRS(
                position[i],
                Quaternion.identity,
                Vector3.one * (particleSize * 2f)
            );
        }
        DrawParticles();
        if (Input.GetKeyDown(KeyCode.D))
        {
            float density = fluid.DensityAt(Vector2.zero);
            Debug.Log($"Density: {density}");
        }

    }

    void LateUpdate()
    {
        Camera cam = Camera.main;

        cam.transform.position = new Vector3(
            boundsSize.x * 0.5f,
            boundsSize.y * 0.5f,
            -10f
        );

        float aspect = (float)Screen.width / Screen.height;

        float sizeY = boundsSize.y * 0.5f;
        float sizeX = (boundsSize.x * 0.5f) / aspect;

        cam.orthographicSize = Mathf.Max(sizeY, sizeX);
    }

    float GetFrequency()
    {
        return simulationFrequency switch
        {
            SimulationFrequency.Hz50 => 50f,
            SimulationFrequency.Hz100 => 100f,
            SimulationFrequency.Hz150 => 150f,
            _ => 100
        };
    }

    void SimulationStep(float deltaTime)
    {
        if (fluid == null || position == null || velocity == null || predictedPosition == null || density == null || pressure == null || velocity == null)
        {
            Debug.LogWarning("Reinitializing fluid (null state detected).");
            isRunning = false;
            RebuildFluid();
            return;
        }
        fluid.mass = mass;
        fluid.smoothingRadius = smoothingRadius;
        fluid.restDensity = restDensity;
        fluid.stiffness = stiffness;
        fluid.minDensity = minDensity;
        fluid.viscosityStrength = viscosityStrength;

        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += gravity * deltaTime;
            predictedPosition[i] = position[i] + velocity[i] * deltaTime;

            float minX = particleSize;
            float maxX = boundsSize.x - particleSize;
            float minY = particleSize;
            float maxY = boundsSize.y - particleSize;

            predictedPosition[i].x = Mathf.Clamp(predictedPosition[i].x, minX, maxX);
            predictedPosition[i].y = Mathf.Clamp(predictedPosition[i].y, minY, maxY);
        });

        grid.BuildGrid(predictedPosition);

        Parallel.For(0, numParticles, i =>
        {
            density[i] = fluid.DensityAt(predictedPosition[i]);
            pressure[i] = fluid.ConvertDensityToPressure(density[i]);
        });

        Parallel.For(0, numParticles, i =>
        {
            Vector2 PandVForce = fluid.PressureAndViscosityAt(i);
            Vector2 pressureAcceleration = PandVForce / mass;
            velocity[i] += pressureAcceleration * deltaTime;
        });

        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            CollideBounds(i);
        });
    }

    void DrawParticles()
    {
        if (particleMesh == null || particleMaterial == null || matrices == null)
            return;

        if (matrices.Length < numParticles)
            return;

        int batchSize = 1023;

        for (int b = 0; b < Mathf.CeilToInt((float)numParticles / batchSize); b++)
        {
            int start = b * batchSize;
            int count = Mathf.Min(batchSize, numParticles - start);

            Matrix4x4[] batch = new Matrix4x4[count];

            for (int i = 0; i < count; i++)
            {
                batch[i] = matrices[start + i];
            }

            Graphics.DrawMeshInstanced(
                particleMesh,
                0,
                particleMaterial,
                batch,
                count
            );
        }
    }


    void RebuildFluid()
{

    numParticles = Mathf.Max(1, numParticles);

    position = new Vector2[numParticles];
    velocity = new Vector2[numParticles];
    predictedPosition = new Vector2[numParticles];
    density = new float[numParticles];
    pressure = new float[numParticles];
    matrices = new Matrix4x4[numParticles];

    SpawnFluid();


    if (grid == null) grid = new SpatialHash2D(smoothingRadius, boundsSize, numParticles);
    else grid.Bind(smoothingRadius, boundsSize, numParticles);

    // update fluid's addresses bc we're changing the positions and possibly numParticles
    if (fluid == null) fluid = new SPHFluid2D(predictedPosition, density, pressure, velocity, grid);
    else fluid.Bind(predictedPosition, density, pressure, velocity, grid);
}

void SpawnBounds()
{
    GameObject boundsGO = Instantiate(boundsPrefab);
    bounds = boundsGO.transform;
    bounds.localScale = new Vector3(boundsSize.x, boundsSize.y, 1f);
    bounds.position = new Vector3(
        boundsSize.x * 0.5f,
        boundsSize.y * 0.5f,
        0f
    );
    boundsSize.x = Mathf.Max(particleSize * 2f, boundsSize.x);
    boundsSize.y = Mathf.Max(particleSize * 2f, boundsSize.y);

    bounds.localScale = new Vector3(boundsSize.x, boundsSize.y, 1f);
    bounds.position = new Vector3(boundsSize.x * 0.5f, boundsSize.y * 0.5f, 0f);

    var sr = boundsGO.GetComponent<SpriteRenderer>();
    sr.color = new Color(0f, 1f, 1f, 0.2f);
    sr.sortingOrder = -1;
}

void SpawnFluid()
{
    int particlesPerRow = (int)Mathf.Sqrt(numParticles);
    int particlesPerCol = (numParticles - 1) / particlesPerRow + 1;
    float spacing = particleSize * 2 + particleSpacing;

    float blockWidth = (particlesPerRow - 1) * spacing;
    float blockHeight = (particlesPerCol - 1) * spacing;

    // Center of bounds (since bounds is positioned at half size)
    Vector2 boundsCenter = new Vector2(boundsSize.x * 0.5f, boundsSize.y * 0.5f);

    // Bottom-left corner of centered block
    Vector2 origin = boundsCenter - new Vector2(blockWidth, blockHeight) * 0.5f;

    for (int i = 0; i < numParticles; i++)
    {
        float x = origin.x + i % particlesPerRow * spacing;
        float y = origin.y + i / particlesPerRow * spacing;
        position[i] = new Vector2(x, y);

        matrices[i] = Matrix4x4.TRS(
            position[i],
            Quaternion.identity,
            Vector3.one * (particleSize * 2f)
        );
    }
    DrawParticles();
}

void CollideBounds(int i)
{
    float minX = particleSize;
    float maxX = boundsSize.x - particleSize;

    float minY = particleSize;
    float maxY = boundsSize.y - particleSize;

    if (position[i].x > maxX)
    {
        position[i].x = maxX;
        if (velocity[i].x > 0)
            velocity[i].x *= -collisionDamping;
    }

    if (position[i].x < minX)
    {
        position[i].x = minX;
        if (velocity[i].x < 0)
            velocity[i].x *= -collisionDamping;
    }

    if (position[i].y > maxY)
    {
        position[i].y = maxY;
        if (velocity[i].y > 0)
            velocity[i].y *= -collisionDamping;
    }

    if (position[i].y < minY)
    {
        position[i].y = minY;
        if (velocity[i].y < 0)
            velocity[i].y *= -collisionDamping;
    }
}
}