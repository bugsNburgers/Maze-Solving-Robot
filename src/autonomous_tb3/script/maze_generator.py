#! /usr/bin/env python3

"""Generate a random solvable maze with dynamic start/goal endpoints.

The generated artifact is written to /tmp/autonomous_tb3/maze_runtime.json so
other runtime nodes can consume it in later phases.
"""

import json
import os
import random
from collections import deque


MAZE_HEIGHT = 21
MAZE_WIDTH = 21


def _neighbors_two_cells(row, col, height, width):
    steps = [(-2, 0), (2, 0), (0, -2), (0, 2)]
    for dr, dc in steps:
        nr = row + dr
        nc = col + dc
        if 1 <= nr < height - 1 and 1 <= nc < width - 1:
            yield nr, nc, dr, dc


def generate_maze(height=MAZE_HEIGHT, width=MAZE_WIDTH, seed=None):
    if height % 2 == 0 or width % 2 == 0:
        raise ValueError("Maze dimensions must be odd values")

    rng = random.Random(seed)
    maze = [[1 for _ in range(width)] for _ in range(height)]

    # Recursive backtracker using an explicit stack.
    start = (1, 1)
    stack = [start]
    visited = {start}
    maze[start[0]][start[1]] = 0

    while stack:
        row, col = stack[-1]
        options = []
        for nr, nc, dr, dc in _neighbors_two_cells(row, col, height, width):
            if (nr, nc) not in visited:
                options.append((nr, nc, dr, dc))

        if not options:
            stack.pop()
            continue

        nr, nc, dr, dc = rng.choice(options)
        # Remove wall between current cell and chosen neighbor.
        maze[row + dr // 2][col + dc // 2] = 0
        maze[nr][nc] = 0
        visited.add((nr, nc))
        stack.append((nr, nc))

    return maze


def _bfs_farthest_open_cell(maze, start):
    height = len(maze)
    width = len(maze[0])
    queue = deque([start])
    visited = {start}
    farthest = start

    while queue:
        row, col = queue.popleft()
        farthest = (row, col)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr = row + dr
            nc = col + dc
            if 0 <= nr < height and 0 <= nc < width and maze[nr][nc] == 0:
                nxt = (nr, nc)
                if nxt not in visited:
                    visited.add(nxt)
                    queue.append(nxt)

    return farthest


def _to_world_cell(cell, resolution=0.5, origin=(-5.0, -5.0)):
    row, col = cell
    ox, oy = origin
    x = ox + (col + 0.5) * resolution
    y = oy + (row + 0.5) * resolution
    return {"x": round(x, 3), "y": round(y, 3), "yaw": 0.0}


def _save_artifact(payload, output_path):
    out_dir = os.path.dirname(output_path)
    os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2)


def main():
    seed = random.randrange(1, 10_000_000)
    maze = generate_maze(height=MAZE_HEIGHT, width=MAZE_WIDTH, seed=seed)

    start_cell = (1, 1)
    goal_cell = _bfs_farthest_open_cell(maze, start_cell)

    payload = {
        "seed": seed,
        "height": len(maze),
        "width": len(maze[0]),
        "resolution": 0.5,
        "origin": {"x": -5.0, "y": -5.0},
        "start_cell": {"row": start_cell[0], "col": start_cell[1]},
        "goal_cell": {"row": goal_cell[0], "col": goal_cell[1]},
        "start_pose": _to_world_cell(start_cell),
        "goal_pose": _to_world_cell(goal_cell),
        "grid": maze,
    }

    output_path = "/tmp/autonomous_tb3/maze_runtime.json"
    _save_artifact(payload, output_path)

    print("Generated new procedural maze")
    print(f"Seed: {seed}")
    print(f"Fixed grid size: {MAZE_HEIGHT}x{MAZE_WIDTH}")
    print(f"Start cell: {start_cell} -> world {payload['start_pose']}")
    print(f"Goal  cell: {goal_cell} -> world {payload['goal_pose']}")
    print(f"Saved artifact: {output_path}")


if __name__ == "__main__":
    main()
