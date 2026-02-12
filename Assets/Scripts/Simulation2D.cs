using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Simulation2D : MonoBehaviour
{
    [Header("Spawn")]
    public int numParticles = 1;
    public float particleSize = 0.1f;
    public Vector2 spawnCenter = new Vector2(0, 1.5f);
    public GameObject particlePrefab;

    [Header("Bounds")]
    public Vector2 boundsSize = new Vector2(8f, 5f);
    public float collisionDamping = 0.6f;
    public GameObject boundsPrefab;

    [Header("SPH")]
    public Vector2 gravity = new Vector2(0, 9.81f);

    [Header("Mouse Interaction")]
    public float interactRadius = 0.8f;
    public float interactStrength = 35f; // hold LMB attract, RMB repel

    Vector2[] position;
    Vector2[] velocity;
    Transform[] visual;
    Transform bounds;

    

    void Start()
    {
        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        visual = new Transform[numParticles];

        position[0] = spawnCenter;
        velocity[0] = Vector2.zero;
        SpawnBounds();
        GameObject particleGO = Instantiate(particlePrefab, new Vector3(spawnCenter.x, spawnCenter.y, 0), Quaternion.identity);
        visual[0] = particleGO.transform;
        visual[0].localScale = Vector3.one * (particleSize * 2f);

    }

    void FixedUpdate()
    {
        velocity[0] += gravity * Time.fixedDeltaTime;
        position[0] += velocity[0] * Time.fixedDeltaTime;

        CollideBounds();
        visual[0].localScale = Vector3.one * (particleSize * 2f);
        visual[0].position = new Vector3(position[0].x, position[0].y, 0);

        bounds.localScale = Vector3.one * boundsSize;
    }

    void SpawnBounds()
    {
        GameObject boundsGO = Instantiate(boundsPrefab);
        bounds = boundsGO.transform;
        bounds.position = Vector3.zero;

        bounds.localScale = Vector3.one * boundsSize;

        var sr = boundsGO.GetComponent<SpriteRenderer>();
        sr.color = new Color(0f, 1f, 1f, 0.2f);
        sr.sortingOrder = -1;
    }

    void CollideBounds()
    {
        Vector2 boundsHalfSize = boundsSize / 2 - Vector2.one * particleSize;

        if (Math.Abs(position[0].x) > boundsHalfSize.x)
        {
            position[0].x = boundsHalfSize.x * Math.Sign(position[0].x);
            velocity[0].x *= -1 * collisionDamping;
        }
        if (Math.Abs(position[0].y) > boundsHalfSize.y)
        {
            position[0].y = boundsHalfSize.y * Math.Sign(position[0].y);
            velocity[0].y *= -1 * collisionDamping;
        }
    }
}