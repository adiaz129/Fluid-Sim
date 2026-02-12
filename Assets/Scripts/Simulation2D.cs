using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulation2D : MonoBehaviour
{
    [Header("Spawn")]
    public int numParticles = 1;
    public float particleSize = 0.1f;
    public Vector2 spawnCenter = new Vector2(0, 0);
    public float particleSpacing = 0.01f;
    public GameObject particlePrefab;

    [Header("Bounds")]
    public Vector2 boundsSize = new Vector2(8f, 5f);
    public float collisionDamping = 0.6f;
    public GameObject boundsPrefab;

    [Header("SPH")]
    public Vector2 gravity = new Vector2(0, -9.81f);

    [Header("Mouse Interaction")]
    public float interactRadius = 0.8f;
    public float interactStrength = 35f; // hold LMB attract, RMB repel

    Vector2[] position;
    Vector2[] velocity;
    Transform[] visual;
    Transform bounds;

    bool isRunning = false;

    int lastNumParticles;


    void Start()
    {
        SpawnBounds();
        lastNumParticles = numParticles;
        RebuildFluid();

    }

    void FixedUpdate()
    {
        boundsSize.x = Mathf.Max(particleSize * 2f, boundsSize.x);
        boundsSize.y = Mathf.Max(particleSize * 2f, boundsSize.y);
        bounds.localScale = Vector3.one * boundsSize;

        if (!isRunning) return;

        for (int i = 0; i < numParticles; i++)
        {
            velocity[i] += gravity * Time.fixedDeltaTime;
            position[i] += velocity[i] * Time.fixedDeltaTime;
            CollideBounds(i);
        }
    }

    void Update()
    {
        if (!isRunning)
        {
            if (numParticles != lastNumParticles)
            {
                lastNumParticles = numParticles;
                RebuildFluid();
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                isRunning = true;
            }
        }
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
        visual = new Transform[numParticles];

        SpawnFluid();
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

        // X
        if (position[i].x > boundsHalfSize.x) { position[i].x = boundsHalfSize.x; velocity[i].x = -Mathf.Abs(velocity[i].x) * collisionDamping; }
        if (position[i].x < -boundsHalfSize.x) { position[i].x = -boundsHalfSize.x; velocity[i].x = Mathf.Abs(velocity[i].x) * collisionDamping; }

        // Y
        if (position[i].y > boundsHalfSize.y) { position[i].y = boundsHalfSize.y; velocity[i].y = -Mathf.Abs(velocity[i].y) * collisionDamping; }
        if (position[i].y < -boundsHalfSize.y) { position[i].y = -boundsHalfSize.y; velocity[i].y = Mathf.Abs(velocity[i].y) * collisionDamping; }

        visual[i].localScale = Vector3.one * (particleSize * 2f);
        visual[i].position = new Vector3(position[i].x, position[i].y, 0);
    }
}