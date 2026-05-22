# Maze Solver using A* Search

# Author: Archi (Syntecxhub Internship Project)

import heapq
import matplotlib.pyplot as plt


maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]

start = (0, 0)  # top-left corner
goal = (4, 4)   # bottom-right corner


# ---------- Step 2: Define the Heuristic ----------
def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ---------- Step 3: Get Valid Neighbors ----------
def get_neighbors(maze, node):
    neighbors = []
    directions = [(0,1), (1,0), (0,-1), (-1,0)]  # right, down, left, up
    for dx, dy in directions:
        x, y = node[0] + dx, node[1] + dy
        if 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] == 0:
            neighbors.append((x, y))
    return neighbors


# ---------- Step 4: Reconstruct Path ----------
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------- Step 5: Implement A* Search ----------
def astar(maze, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(maze, current):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, neighbor))
    return None


# ---------- Step 6: Run the Algorithm ----------
path = astar(maze, start, goal)
if path:
    print("Shortest path found:", path)
else:
    print("No path found!")


# ---------- Step 7: Visualize ----------
def visualize(maze, path):
    maze_copy = [row[:] for row in maze]
    for (x, y) in path:
        maze_copy[x][y] = 2  # mark path
    plt.imshow(maze_copy, cmap='cool')
    plt.title("Maze Solver using A* Search")
    plt.show()

if path:
    visualize(maze, path)
