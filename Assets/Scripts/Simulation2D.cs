using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.U2D.IK;

public class Simulation2D : MonoBehaviour
{
    [Header("Spawn")]
    public int numParticles = 900;
    public float particleSize = 0.06f;
    public Vector2 spawnCenter = new Vector2(0, 0);
    public float particleSpacing = 0.006f;
    public GameObject particlePrefab;

    [Header("Bounds")]
    public Vector2 boundsSize = new Vector2(8f, 5f);
    public float collisionDamping = 0.6f;
    public GameObject boundsPrefab;

    [Header("SPH")]
    public Vector2 gravity = new Vector2(0, -9.81f);
    public float smoothingRadius = 0.35f;
    public float restDensity = 22;
    public float stiffness = 22;
    public float mass = 1;
    public float minDensity = 1e-4f;

    [Header("Mouse Interaction")]
    public float interactRadius = 0.8f;
    public float interactStrength = 35f; // hold LMB attract, RMB repel

    Vector2[] position;
    Vector2[] velocity;
    Vector2[] predictedPosition;
    float[] density;
    Transform[] visual;
    Transform bounds;

    bool isRunning = false;

    int lastNumParticles;
    float lastParticleSpacing;

    private SPHSolver2D solver;

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

        SimulationStep(Time.fixedDeltaTime);
        for (int i = 0; i < numParticles; i++)
        {
            visual[i].localScale = Vector3.one * (particleSize * 2f);
            visual[i].position = new Vector3(position[i].x, position[i].y, 0);
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
        if (Input.GetKeyDown(KeyCode.D))
        {
            float density = solver.DensityAt(Vector2.zero);
            Debug.Log($"Density: {density}");
        }
    }

    void SimulationStep(float deltaTime)
    {
        solver.mass = mass;
        solver.smoothingRadius = smoothingRadius;
        solver.restDensity = restDensity;
        solver.stiffness = stiffness;
        solver.minDensity = minDensity;
        
        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += gravity * deltaTime;
            predictedPosition[i] = position[i] + velocity[i] * deltaTime;
        });

        Parallel.For(0, numParticles, i =>
        {
            density[i] = solver.DensityAt(predictedPosition[i]);
        });

        Parallel.For(0, numParticles, i =>
        {
            float di = Mathf.Max(density[i], minDensity);
            Vector2 pressureForce = solver.CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / di;
            velocity[i] += pressureAcceleration * deltaTime;
        });

        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            CollideBounds(i);
        });
    }

    void RebuildFluid()
    {
        if (visual != null)
        {
            for (int i = 0; i < visual.Length; i++)
                if (visual[i] != null) Destroy(visual[i].gameObject);
        }

        numParticles = Mathf.Max(1, numParticles);

        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        predictedPosition = new Vector2[numParticles];
        density = new float[numParticles];
        visual = new Transform[numParticles];

        SpawnFluid();

        // update solver's addresses bc we're changing the positions and possibly numParticles
        if (solver == null) solver = new SPHSolver2D(predictedPosition, density);
        else solver.Bind(predictedPosition, density);
    }

    void SpawnBounds()
    {
        GameObject boundsGO = Instantiate(boundsPrefab);
        bounds = boundsGO.transform;
        bounds.position = Vector3.zero;

        boundsSize.x = Mathf.Max(particleSize * 2f, boundsSize.x);
        boundsSize.y = Mathf.Max(particleSize * 2f, boundsSize.y);
        bounds.localScale = Vector3.one * boundsSize;

        var sr = boundsGO.GetComponent<SpriteRenderer>();
        sr.color = new Color(0f, 1f, 1f, 0.2f);
        sr.sortingOrder = -1;
    }

    void SpawnFluid()
    {
        int particlesPerRow = (int)Mathf.Sqrt(numParticles);
        int particlesPerCol = (numParticles - 1) / particlesPerRow + 1;
        float spacing = particleSize * 2 + particleSpacing;

        for (int i = 0; i < numParticles; i++)
        {
            float x = (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
            float y = (i / particlesPerRow - particlesPerCol / 2f + 0.5f) * spacing;
            position[i] = new Vector2(x, y);

            GameObject particleGO = Instantiate(particlePrefab, new Vector3(position[i].x, position[i].y, 0f), Quaternion.identity);
            visual[i] = particleGO.transform;
            visual[i].localScale = Vector3.one * (particleSize * 2f);
        }
    }

    void CollideBounds(int i)
    {
        Vector2 boundsHalfSize = boundsSize / 2 - Vector2.one * particleSize;

        if (position[i].x > boundsHalfSize.x)
        {
            position[i].x = boundsHalfSize.x;

            if (velocity[i].x > 0) 
                velocity[i].x = -velocity[i].x * collisionDamping;
        }
        if (position[i].x < -boundsHalfSize.x)
        {
            position[i].x = -boundsHalfSize.x;

            if (velocity[i].x < 0) 
                velocity[i].x = -velocity[i].x * collisionDamping;
        }
        if (position[i].y > boundsHalfSize.y)
        {
            position[i].y = boundsHalfSize.y;

            if (velocity[i].y > 0f)
                velocity[i].y = -velocity[i].y * collisionDamping;
        }

        if (position[i].y < -boundsHalfSize.y)
        {
            position[i].y = -boundsHalfSize.y;

            if (velocity[i].y < 0f)
                velocity[i].y = -velocity[i].y * collisionDamping;
        }
    }
}