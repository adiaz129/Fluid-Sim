using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.VisualScripting;
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
    public float nearStiffness = 22f;
    public float viscosityStrength = 0.5f;
    public float mass = 1;
    public float minDensity = 1e-4f;
    public float surfaceThreshold = 0.2f;
    public float surfTensMultiplier = 3f;


    [Header("Testing")]
    public Gradient testingGradient;
    public enum TestingMethod
    {
        none,
        velocity,
        pressure,
        density,
        neighbors,
        normals
    }
    [SerializeField]
    private TestingMethod testingMethod;
    public float upperSpeed = 200f;
    public float upperPressure = 8000f;
    public float upperDensity = 200f;
    public int upperNeighbors = 7;
    public float upperNormals = 2f;

    [Header("Simulation Controls")]
    public float mouseRadius = 1.0f;
    public float mouseForce = 50f;
    public enum SimulationFrequency
    {
        Hz50,
        Hz100,
        Hz150,
        Hz200,
        Hz500
    }
    [SerializeField]
    private SimulationFrequency simulationFrequency;

    Vector2[] position;  
    Vector2[] velocity;
    Vector2[] predictedPosition;
    float[] density;
    float[] nearDensity;
    float[] pressure;
    float[] nearPressure;
    int[] neighbors;
    float[] normals;
    Transform bounds;
    Vector2 _mouseWorldPos;
    bool _mouseDown;

    bool isRunning = false; 
    bool debug = false;

    int lastNumParticles;
    float lastParticleSpacing;
    Vector2 lastBoundsSize;
    float lastSmoothingRadius;

    private SPHFluid2D fluid;

    private SpatialHash2D grid;

    private Matrix4x4[] matrices;
    private Vector4[] colors;
    private MaterialPropertyBlock propertyBlock;
    private Matrix4x4[] batchMatrices;
    private Vector4[] batchColors;

    void Start() // runs once
    {
        SpawnBounds();
        lastNumParticles = numParticles;
        lastParticleSpacing = particleSpacing;
        lastBoundsSize = boundsSize;
        lastSmoothingRadius = smoothingRadius;
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
        _mouseDown = Input.GetMouseButton(0);
        _mouseWorldPos = GetMouseWorldPos();
        if (!isRunning && Input.GetKeyDown(KeyCode.RightArrow))
        {
            int substeps = Mathf.CeilToInt(GetFrequency() * Time.fixedDeltaTime);
            float dt = Time.fixedDeltaTime / substeps;
            SimulationStep(dt);
        }
        if (Input.GetKeyDown(KeyCode.Space))
            isRunning = !isRunning;
        if (Input.GetKeyDown(KeyCode.D))
            debug = !debug;
        if (numParticles != lastNumParticles || particleSpacing != lastParticleSpacing || boundsSize != lastBoundsSize)
        {
            lastNumParticles = numParticles;
            lastParticleSpacing = particleSpacing;
            lastBoundsSize = boundsSize;
            SpawnBounds();
            RebuildFluid();
        }
        Parallel.For(0, numParticles, i =>
        {
            if (float.IsNaN(position[i].x) || float.IsNaN(position[i].y))
        {
            Debug.LogError("NaN position detected at particle " + i);
            position[i] = Vector2.zero;
        }
            matrices[i] = Matrix4x4.TRS(
                position[i],
                Quaternion.identity,
                Vector3.one * (particleSize * 2f)
            );
        });
        for (int i = 0; i < numParticles; i++)
            TestingGradient(i);
        DrawParticles();
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
            SimulationFrequency.Hz200 => 200f,
            SimulationFrequency.Hz500 => 500f,
            _ => 100
        };
    }

    void SimulationStep(float deltaTime)
    {
        if (fluid == null || position == null || velocity == null || predictedPosition == null || density == null || nearDensity == null || pressure == null || nearPressure == null || velocity == null)
        {
            Debug.LogWarning("Reinitializing fluid (null state detected).");
            isRunning = false;
            RebuildFluid();
            return;
        }
        if (lastSmoothingRadius != smoothingRadius)
        {
            SPHMath2D.SetSmoothingRadius(smoothingRadius);
            lastSmoothingRadius = smoothingRadius; 
        }
        fluid.mass = mass;
        fluid.smoothingRadius = smoothingRadius;
        fluid.restDensity = restDensity;
        fluid.stiffness = stiffness;
        fluid.nearStiffness = nearStiffness;
        fluid.minDensity = minDensity;
        fluid.viscosityStrength = viscosityStrength;
        fluid.surfaceThreshold = surfaceThreshold;
        fluid.surfTensMultiplier = surfTensMultiplier;

        Parallel.For(0, numParticles, i =>
        {
            Vector2 mouseForce = MouseForceAt(i);
            velocity[i] += (gravity + mouseForce) * deltaTime;
            predictedPosition[i] = position[i] + velocity[i] * deltaTime;

            float minX = particleSize;
            float maxX = boundsSize.x - particleSize;
            float minY = particleSize;
            float maxY = boundsSize.y - particleSize;

            predictedPosition[i].x = Mathf.Clamp(predictedPosition[i].x, minX, maxX);
            predictedPosition[i].y = Mathf.Clamp(predictedPosition[i].y, minY, maxY);
        });

        grid.BuildGrid(predictedPosition);
        Array.Clear(neighbors, 0, neighbors.Length);

        Parallel.For(0, numParticles, i =>
        {
            (density[i], nearDensity[i]) = fluid.DensityAt(predictedPosition[i]);
            (pressure[i], nearPressure[i]) = fluid.ConvertDensityToPressure(density[i], nearDensity[i]);
        });

        Parallel.For(0, numParticles, i =>
        {
            Vector2 fluidForce = fluid.PressureAndViscosityAt(i);

            Vector2 totalForce = fluidForce;

            Vector2 acceleration = totalForce / mass;
            
            velocity[i] += acceleration * deltaTime;
            if (debug)
            {
                Debug.DrawLine(
                position[i],
                position[i] + totalForce * 0.01f,
                Color.red,
                0f,
                false
            );
            }
        });

        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            CollideBounds(i);
        });
    }

    void DrawParticles()
    {
        if (particleMesh == null ||
        particleMaterial == null ||
        matrices == null ||
        colors == null ||
        propertyBlock == null)
            return;

        if (matrices.Length < numParticles)
            return;

        int batchSize = 1023;

        for (int b = 0; b < Mathf.CeilToInt((float)numParticles / batchSize); b++)
        {
            int start = b * batchSize;
            int count = Mathf.Min(batchSize, numParticles - start);

            for (int i = 0; i < count; i++)
            {
                batchMatrices[i] = matrices[start + i];
                batchColors[i] = colors[start + i];
            }
            propertyBlock.Clear();
            propertyBlock.SetVectorArray("_Color", batchColors);

            Graphics.DrawMeshInstanced(
                particleMesh,
                0,
                particleMaterial,
                batchMatrices,
                count,
                propertyBlock
            );
        }
    }

    void TestingGradient(int pointIndex)
    {
        float normalizedValue;
        if (testingMethod == TestingMethod.velocity)
        {
            float sqrSpeed = velocity[pointIndex].sqrMagnitude;
            normalizedValue = Mathf.InverseLerp(0, upperSpeed, sqrSpeed);
        }
        else if (testingMethod == TestingMethod.pressure)
            normalizedValue = Mathf.InverseLerp(-100, upperPressure, pressure[pointIndex]);
        else if (testingMethod == TestingMethod.density)
            normalizedValue = Mathf.InverseLerp(0, upperDensity, density[pointIndex]);
        else if (testingMethod == TestingMethod.neighbors)
            normalizedValue = Mathf.InverseLerp(0, upperNeighbors, neighbors[pointIndex]);
        else if (testingMethod == TestingMethod.normals)
            normalizedValue = Mathf.InverseLerp(0, Math.Max(surfaceThreshold, 0), normals[pointIndex]);
        else
            normalizedValue = 0;

        colors[pointIndex] = testingGradient.Evaluate(normalizedValue);
    }


    void RebuildFluid()
    {

        numParticles = Mathf.Max(1, numParticles);

        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        predictedPosition = new Vector2[numParticles];
        density = new float[numParticles];
        nearDensity = new float[numParticles];
        pressure = new float[numParticles];
        nearPressure = new float[numParticles];
        matrices = new Matrix4x4[numParticles];
        colors = new Vector4[numParticles];
        propertyBlock = new MaterialPropertyBlock();
        neighbors = new int[numParticles];
        normals = new float[numParticles];
        batchMatrices = new Matrix4x4[1023];
        batchColors = new Vector4[1023];

        SPHMath2D.SetSmoothingRadius(smoothingRadius);
 
        SpawnFluid();


        if (grid == null) grid = new SpatialHash2D(smoothingRadius, boundsSize, numParticles);
        else grid.Bind(smoothingRadius, boundsSize, numParticles);

        // update fluid's addresses bc we're changing the positions and possibly numParticles
        if (fluid == null) fluid = new SPHFluid2D(predictedPosition, density, nearDensity, pressure, nearPressure, velocity, neighbors, normals, grid);
        else fluid.Bind(predictedPosition, density, nearDensity, pressure, nearPressure, velocity, neighbors, normals, grid);
    }

    void SpawnBounds()
    {
        if (bounds != null)
            Destroy(bounds.gameObject);
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


        bounds.position = new Vector3(boundsSize.x * 0.5f, boundsSize.y * 0.5f, 0f);

        var sr = boundsGO.GetComponent<SpriteRenderer>();
        sr.color = Color.black;
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
        }
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
    Vector2 GetMouseWorldPos()
    {
        Vector3 mouse = Input.mousePosition;
        mouse.z = -Camera.main.transform.position.z;
        return Camera.main.ScreenToWorldPoint(mouse);
    }
   Vector2 MouseForceAt(int pointIndex)
{
    if (!_mouseDown)
        return Vector2.zero;

    Vector2 pos = predictedPosition[pointIndex];

    Vector2 offset = _mouseWorldPos - pos;

    float sqrDst = offset.sqrMagnitude;
    float radiusSqr = mouseRadius * mouseRadius;

    if (sqrDst > radiusSqr || sqrDst < 1e-8f)
        return Vector2.zero;

    float dst = Mathf.Sqrt(sqrDst);

    float edgeT = dst / mouseRadius;
    float centreT = 1f - edgeT;

    Vector2 dirToCentre = offset / dst;

    // Mimic his saturate(x) behavior
    float strengthNorm = Mathf.Clamp(mouseForce / 10f, 0f, 1f);

    // Gravity weight modulation (like his gravityWeight)
    float gravityWeight = 1f - (centreT * strengthNorm);

    // Attraction field
    Vector2 gravityAccel =
        dirToCentre * centreT * mouseForce;

    // Velocity damping inside radius
    Vector2 damping =
        -velocity[pointIndex] * centreT;

    return gravityAccel * gravityWeight + damping;
}

}