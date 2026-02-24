using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class SpatialHash2D
{
    private float cellSize;
    private int gridSizeX;
    private int gridSizeY;
    private int numCells;

    public int[] cellStart;
    public int[] cellEnd;
    public CellPair[] spatialLookup;

    public struct CellPair
    {
        public int key;
        public int particleIndex;

        public CellPair(int key, int particleIndex)
        {
            this.key = key;
            this.particleIndex = particleIndex;
        }
    }

    public SpatialHash2D(float cellSize, Vector2 boundsSize, int numParticles)
    {
        Bind(cellSize, boundsSize, numParticles);
    }

    public void Bind(float cellSize, Vector2 boundsSize, int numParticles)
    {
        this.cellSize = cellSize;
        
        gridSizeX = Mathf.CeilToInt(boundsSize.x / cellSize);
        gridSizeY = Mathf.CeilToInt(boundsSize.y / cellSize);

        numCells = gridSizeX * gridSizeY;

        cellStart = new int[numCells];
        cellEnd = new int[numCells];
        spatialLookup = new CellPair[numParticles];
    }

    public bool IsCellInBounds(Vector2Int cell)
    {
        return cell.x >= 0 && cell.x < gridSizeX && cell.y >= 0 && cell.y < gridSizeY;
    }

    // make a function that finds what grid a position belongs in
    public Vector2Int CellCoord(Vector2 p)
    {
        return new Vector2Int(
            Mathf.FloorToInt(p.x / cellSize),
            Mathf.FloorToInt(p.y / cellSize)
        );
    }

    public int GetKey(Vector2Int cell)
    {
        return cell.x + cell.y * gridSizeX;
    }

    // make a function that builds the grid
    public void BuildGrid(Vector2[] pos)
    {
        Parallel.For(0, pos.Length, i =>
        {
            var cell = CellCoord(pos[i]);
            int key = GetKey(cell);
            spatialLookup[i] = new CellPair(key, i);
        });

        Array.Sort(spatialLookup, (a, b) => a.key.CompareTo(b.key));

        Parallel.For(0, spatialLookup.Length, i =>
        {
            int key = spatialLookup[i].key;
            if (i == spatialLookup.Length - 1 || key != spatialLookup[i + 1].key)
                cellEnd[key] = i + 1;
        });


        Parallel.For(0, spatialLookup.Length, i =>
        {
            int key = spatialLookup[i].key;
            if (i == 0 || key != spatialLookup[i - 1].key)
                cellStart[key] = i;
        });
    }
}