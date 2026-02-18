using System.Collections.Generic;
using UnityEngine;

public class SpatialHash2D
{
    // make a function that finds what grid a position belongs in
    public static Vector2Int CellCoord(Vector2 p, float cellSize)
    {
        return new Vector2Int(
            Mathf.FloorToInt(p.x / cellSize),
            Mathf.FloorToInt(p.y / cellSize)
        );
    }

    // make a function that builds the grid
    public static void BuildGrid(Dictionary<Vector2Int, List<int>> grid, Vector2[] pos, float cellSize)
    {
        grid.Clear();
        for (int i = 0; i < pos.Length; i++)
        {
            var cell = CellCoord(pos[i], cellSize);
            if (!grid.TryGetValue(cell, out var list))
            {
                list = new List<int>(16);
                grid[cell] = list;
            }
            list.Add(i);
        }
    }
}