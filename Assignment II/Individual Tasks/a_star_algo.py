import numpy as np
import matplotlib.pyplot as plt
import heapq

# Load the grid from the CSV file
grid_file = "merged_grid30.csv"
binary_grid = np.genfromtxt(grid_file, delimiter=',', dtype=int)

# Define the heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A-Star algorithm implementation
def a_star_search(grid, start, goal):
    rows, cols = grid.shape
    open_set = []  # Priority queue for open nodes
    heapq.heappush(open_set, (0, start))  # (priority, position)
    came_from = {}  # To reconstruct the path
    g_score = {start: 0}  # Actual cost from start to each position
    f_score = {start: heuristic(start, goal)}  # Estimated total cost
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        # If the goal is reached
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path
        
        x, y = current
        # Explore neighbors (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (x + dx, y + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:  # Check bounds
                if grid[neighbor] == 1:  # Skip blocked cells
                    continue
                
                # Tentative g_score
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found

# Example usage
start_point = (9, 10)  # Replace with your desired start point
goal_point = (16, 10)  # Replace with your desired goal point

# Ensure start and goal points are passable
binary_grid[start_point] = 0
binary_grid[goal_point] = 0

# Run the A-star algorithm
path = a_star_search(binary_grid, start_point, goal_point)

# Mark the path on the grid
if path:
    for x, y in path:
        binary_grid[x, y] = 2  # Mark path cells with a unique value

# Visualization of the grid and path
plt.figure(figsize=(8, 8))
# Use a colormap to distinguish walls, passable cells, and the path
cmap = plt.cm.colors.ListedColormap(["white", "black", "red"])  # 0=white, 1=black, 2=red
plt.imshow(binary_grid.T, origin='lower', cmap=cmap)

# Add grid lines and labels
plt.xticks(ticks=np.arange(31), labels=np.arange(31))
plt.yticks(ticks=np.arange(31), labels=np.arange(31))
plt.grid(color='gray', linestyle='-', linewidth=0.5)
plt.title("A-Star Pathfinding with Full Cell Path")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")

# Show the plot
plt.show()
