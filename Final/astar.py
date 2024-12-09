import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.optimize import leastsq
from heapq import heappop, heappush
import os


# Load all three CSV files containing the LiDAR scan points
df1 = pd.read_csv('lidar11.csv')  # Map recorded at (1, 1)
df2 = pd.read_csv('lidar31.csv')  # Map recorded at (3, 1)
df3 = pd.read_csv('lidar13.csv')  # Map recorded at (1, 3)

# Extract x, y coordinates from each file
x1, y1 = df1['x'].values, df1['y'].values
x2, y2 = df2['x'].values, df2['y'].values
x3, y3 = df3['x'].values, df3['y'].values

# Define translation vectors and rotation angles
translation_vector_1 = np.array([0, 0])  # Map 1 is the reference frame
translation_vector_2 = np.array([2, 0])  # Translate by (2, 0) for Map 2
translation_vector_3 = np.array([0, 2])  # Translate by (0, 2) for Map 3

rotation_angle_1 = 0  # No rotation for Map 1
rotation_angle_2 = 0  # No rotation for Map 2
rotation_angle_3 = 0  # No rotation for Map 3

# Create rotation matrices
def get_rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

rotation_matrix_1 = get_rotation_matrix(rotation_angle_1)
rotation_matrix_2 = get_rotation_matrix(rotation_angle_2)
rotation_matrix_3 = get_rotation_matrix(rotation_angle_3)

# Apply transformations to align slices to the same frame
transformed_1 = np.dot(rotation_matrix_1, np.vstack((x1, y1))) + translation_vector_1.reshape(2, 1)
transformed_2 = np.dot(rotation_matrix_2, np.vstack((x2, y2))) + translation_vector_2.reshape(2, 1)
transformed_3 = np.dot(rotation_matrix_3, np.vstack((x3, y3))) + translation_vector_3.reshape(2, 1)

# Combine all points into a single dataset
all_x = np.concatenate((transformed_1[0], transformed_2[0], transformed_3[0]))
all_y = np.concatenate((transformed_1[1], transformed_2[1], transformed_3[1]))

# Combine x and y into a 2D array
all_points = np.vstack((all_x, all_y)).T

# Step 1: Detect clusters using DBSCAN
db = DBSCAN(eps=0.2, min_samples=10).fit(all_points)
labels = db.labels_

# Extract unique clusters
unique_labels = set(labels)
clusters = [all_points[labels == k] for k in range(len(unique_labels)) if k != -1]

# Step 2: Circle fitting function
def fit_circle(points):
    def calc_radius(params, x, y):
        xc, yc, r = params
        return np.sqrt((x - xc)**2 + (y - yc)**2) - r

    x, y = points[:, 0], points[:, 1]
    x_m, y_m = np.mean(x), np.mean(y)  # Approximate center
    initial_guess = [x_m, y_m, 0.5]  # Initial guess: center and radius
    params, _ = leastsq(calc_radius, initial_guess, args=(x, y))
    return params  # Returns xc, yc, r

# Fit circles to the clusters
fitted_circles = [fit_circle(cluster) for cluster in clusters]

# Step 3: Inflate the radius and draw circles
min_x, min_y = np.min(all_x), np.min(all_y)
max_x, max_y = np.max(all_x), np.max(all_y)

# Inflate boundary by -0.3
inflation_radius = -0.3
inflated_x_min = min_x - inflation_radius
inflated_y_min = min_y - inflation_radius
inflated_x_max = max_x + inflation_radius
inflated_y_max = max_y + inflation_radius


# Define finer grid resolution
cell_size = 0.02  # Reduce cell size for higher resolution

# Define grid bounds to include the full range of the data
x_min, x_max = np.min(all_x) - 0.5, np.max(all_x) + 0.5
y_min, y_max = np.min(all_y) - 0.5, np.max(all_y) + 0.5

# Include inflated boundaries
x_min = min(x_min, inflated_x_min)
x_max = max(x_max, inflated_x_max)
y_min = min(y_min, inflated_y_min)
y_max = max(y_max, inflated_y_max)

# Increase the radius of all circles
radius_increment = 0.1  # Increase each circle radius by this value
fitted_circles = [(xc, yc, r + radius_increment) for xc, yc, r in fitted_circles]

# Create the grid
x_range = np.arange(x_min, x_max, cell_size)
y_range = np.arange(y_min, y_max, cell_size)
grid = np.zeros((len(y_range), len(x_range)))

# Populate the grid with refined conditions
for i, y in enumerate(y_range):
    for j, x in enumerate(x_range):
        cell_center = np.array([x + cell_size / 2, y + cell_size / 2])
        
        # Condition 1: Check if the cell is inside any circle with radius < 1
        inside_circle = any(
            np.sqrt((cell_center[0] - xc)**2 + (cell_center[1] - yc)**2) <= r
            for xc, yc, r in fitted_circles if r < 1
        )
        
        # Condition 2: Check if the cell is between the original and inflated boundaries
        inside_boundary = (
            (min_x <= cell_center[0] <= max_x and min_y <= cell_center[1] <= max_y) and
            not (inflated_x_min <= cell_center[0] <= inflated_x_max and inflated_y_min <= cell_center[1] <= inflated_y_max)
        )
        
        # Mark the cell as occupied if it meets either condition
        if inside_circle or inside_boundary:
            grid[i, j] = 1

# Plot the refined grid
plt.figure(figsize=(10, 10))
plt.imshow(
    grid,
    extent=[x_range[0], x_range[-1], y_range[0], y_range[-1]],
    origin='lower',
    cmap='Greys',
    alpha=0.5
)

# Define functions to map between world coordinates and grid indices
def world_to_grid(x, y, x_min, y_min, cell_size):
    """Convert world coordinates to grid indices."""
    grid_x = int((x - x_min) / cell_size)
    grid_y = int((y - y_min) / cell_size)
    return grid_x, grid_y

def grid_to_world(grid_x, grid_y, x_min, y_min, cell_size):
    """Convert grid indices to world coordinates."""
    x = grid_x * cell_size + x_min
    y = grid_y * cell_size + y_min
    return x, y

def a_star(grid, start, goal):
    rows, cols = grid.shape
    directions = [
        (0, 1), (0, -1), (1, 0), (-1, 0),  # Cardinal directions
        (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal directions
    ]
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_cost = {start: 0}
    f_cost = {start: heuristic(start, goal)}

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and grid[neighbor[1], neighbor[0]] == 0:
                # Calculate tentative g_cost
                movement_cost = np.sqrt(dx**2 + dy**2)  # 1 for cardinal, √2 for diagonal
                tentative_g_cost = g_cost[current] + movement_cost
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                    heappush(open_set, (f_cost[neighbor], neighbor))
    return None  # No path found

def heuristic(a, b):
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    """Reconstruct the path from the A* result."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    path.reverse()
    return path

# Define the grid, bounds, and finer grid resolution (from your code)
cell_size = 0.02
x_min, x_max = np.min(all_x) - 0.5, np.max(all_x) + 0.5
y_min, y_max = np.min(all_y) - 0.5, np.max(all_y) + 0.5
x_min = min(x_min, inflated_x_min)
x_max = max(x_max, inflated_x_max)
y_min = min(y_min, inflated_y_min)
y_max = max(y_max, inflated_y_max)

x_range = np.arange(x_min, x_max, cell_size)
y_range = np.arange(y_min, y_max, cell_size)
grid = np.zeros((len(y_range), len(x_range)))

# Populate the grid with conditions
radius_increment = 0.2
fitted_circles = [(xc, yc, r + radius_increment) for xc, yc, r in fitted_circles]

for i, y in enumerate(y_range):
    for j, x in enumerate(x_range):
        cell_center = np.array([x + cell_size / 2, y + cell_size / 2])
        inside_circle = any(
            np.sqrt((cell_center[0] - xc) ** 2 + (cell_center[1] - yc) ** 2) <= r
            for xc, yc, r in fitted_circles if r < 1
        )
        inside_boundary = (
            (min_x <= cell_center[0] <= max_x and min_y <= cell_center[1] <= max_y) and
            not (inflated_x_min <= cell_center[0] <= inflated_x_max and inflated_y_min <= cell_center[1] <= inflated_y_max)
        )
        if inside_circle or inside_boundary:
            grid[i, j] = 1

# Define start and goal points in world coordinates
start_world = (0.3, 0.3)  # Example start point in meters
goal_world = (2.0, 2.0)   # Example goal point in meters

# Convert to grid indices
start_grid = world_to_grid(start_world[0], start_world[1], x_min, y_min, cell_size)
goal_grid = world_to_grid(goal_world[0], goal_world[1], x_min, y_min, cell_size)

# Run A* algorithm
path = a_star(grid, start_grid, goal_grid)

# Visualize the grid and the path
plt.figure(figsize=(10, 10))
plt.imshow(
    grid,
    extent=[x_range[0], x_range[-1], y_range[0], y_range[-1]],
    origin='lower',
    cmap='Greys',
    alpha=0.5
)

# # Plot all points and fitted circles
# plt.scatter(all_x, all_y, s=1, c='blue')
# for xc, yc, r in fitted_circles:
#     if r < 1:
#         circle = plt.Circle((xc, yc), r, color='orange', fill=False)
#         plt.gca().add_patch(circle)

# Plot the start and goal points
plt.scatter([start_world[0]], [start_world[1]], color='green', label='Start')
plt.scatter([goal_world[0]], [goal_world[1]], color='blue', label='Goal')

# Plot the path if found
if path:
    world_path = [grid_to_world(px, py, x_min, y_min, cell_size) for px, py in path]
    path_x, path_y = zip(*world_path)
    plt.plot(path_x, path_y, 'r-', label='Path')
else:
    print("No path found.")

# Directory to save the files
output_dir = 'output_files'
os.makedirs(output_dir, exist_ok=True)

# Save the grid map
grid_file = os.path.join(output_dir, 'grid_map.npy')
np.save(grid_file, grid)
print(f"Grid map saved to {grid_file}")

# Save the path as an array of world coordinates (if a path exists)
if path:
    world_path = np.array([grid_to_world(px, py, x_min, y_min, cell_size) for px, py in path])
    path_file = os.path.join(output_dir, 'path.npy')
    np.save(path_file, path)
    print(f"A* path saved to {path_file}")
else:
    print("No path found, so the path was not saved.")



# Add boundaries and legend
plt.plot([min_x, min_x], [min_y, max_y], color='black', linewidth=2)
plt.plot([max_x, max_x], [min_y, max_y], color='black', linewidth=2)
plt.plot([min_x, max_x], [min_y, min_y], color='black', linewidth=2)
plt.plot([min_x, max_x], [max_y, max_y], color='black', linewidth=2)
plt.legend()
plt.title("Grid Representation with A* Path")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.axis('equal')
plt.grid(True)
plt.show()