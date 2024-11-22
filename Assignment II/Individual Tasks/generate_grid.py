import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV file
file_path = 'p2.csv'  # Replace with your file path
lidar_data = pd.read_csv(file_path)

# Function to rotate coordinates by a given angle (degrees)
def apply_rotation(x_coords, y_coords, rotation_angle):
    angle_rad = np.radians(rotation_angle)
    x_rotated = x_coords * np.cos(angle_rad) - y_coords * np.sin(angle_rad)
    y_rotated = x_coords * np.sin(angle_rad) + y_coords * np.cos(angle_rad)
    return x_rotated, y_rotated

# Validate that the required columns exist
if 'x' in lidar_data.columns and 'y' in lidar_data.columns:
    # Define rotation angle (degrees)
    rotation_angle = 0
    
    # Apply rotation to lidar points
    lidar_data['x_rotated'], lidar_data['y_rotated'] = apply_rotation(
        lidar_data['x'], lidar_data['y'], rotation_angle
    )

    # Create a grid map for rotated points
    resolution = 0.05 # Grid cell size
    x_min, y_min = lidar_data[['x_rotated', 'y_rotated']].min()
    x_max, y_max = lidar_data[['x_rotated', 'y_rotated']].max()

    # Calculate grid dimensions
    grid_width = int((x_max - x_min) / resolution) + 1
    grid_height = int((y_max - y_min) / resolution) + 1

    # Initialize the grid map
    occupancy_grid = np.zeros((grid_width, grid_height), dtype=int)

    # Map lidar points to the grid
    for x, y in zip(lidar_data['x_rotated'], lidar_data['y_rotated']):
        grid_col = int((x - x_min) / resolution)
        grid_row = int((y - y_min) / resolution)
        occupancy_grid[grid_col, grid_row] = 1  # Mark as occupied

    # Visualization: Grid Map
    plt.figure(figsize=(8, 8))
    plt.imshow(
        occupancy_grid.T, origin='lower',
        extent=[x_min, x_max, y_min, y_max],
        cmap='Greys', alpha=0.7
    )
    plt.title("Grid Map from Point 1")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid(True)

    # Show the plot
    plt.tight_layout()
    plt.show()


    # output_csv_path = 'grid3.csv'  # Specify the file name
    # np.savetxt(output_csv_path, occupancy_grid, delimiter=',', fmt='%d')
