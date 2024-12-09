import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import zoom


# Load grid maps from CSV files
grid1_path = 'grid1.csv'  
grid2_path = 'grid2.csv'
grid3_path = 'grid3.csv'

grid1 = np.genfromtxt(grid1_path, delimiter=',', dtype=int)
grid2 = np.genfromtxt(grid2_path, delimiter=',', dtype=int)
grid3 = np.genfromtxt(grid3_path, delimiter=',', dtype=int)

# Function to pad a grid for alignment
def pad_grid(grid, target_shape, offset):
    padded_grid = np.zeros(target_shape, dtype=int)
    x_offset, y_offset = offset
    # Ensure that the grid fits within the target shape with the given offsets
    grid_height, grid_width = grid.shape
    padded_grid[x_offset:x_offset + grid_height, y_offset:y_offset + grid_width] = grid
    return padded_grid

# Calculate target shape for merged grid
# The final grid should be large enough to accommodate all three grids
max_width = max(grid1.shape[1], grid2.shape[1], grid3.shape[1])
max_height = max(grid1.shape[0], grid2.shape[0], grid3.shape[0])

# Set padding and offsets based on the size of your grids
offset1 = (0, 0)  
offset2 = (0, 0)  
offset3 = (0, 0)  

# Calculate the target shape of the merged grid
merged_shape = (max(grid1.shape[0] + offset3[0], grid2.shape[0] + offset2[0], grid3.shape[0] + offset3[0]),
                max(grid1.shape[1] + offset1[1], grid2.shape[1] + offset2[1], grid3.shape[1] + offset3[1]))

# Pad grids to align them
grid1_padded = pad_grid(grid1, merged_shape, offset1)
grid2_padded = pad_grid(grid2, merged_shape, offset2)
grid3_padded = pad_grid(grid3, merged_shape, offset3)

# Merge grids by taking the maximum value at each position
merged_grid = np.maximum.reduce([grid1_padded, grid2_padded, grid3_padded])

# # Save the merged grid to a CSV file
# output_path = 'merged_grid.csv'
# np.savetxt(output_path, merged_grid, delimiter=',', fmt='%d')
# print(f"Merged grid map saved to {output_path}")

# # Visualization
# import matplotlib.pyplot as plt

# plt.figure(figsize=(10, 10))
# plt.imshow(merged_grid.T, origin='lower', cmap='Greys', alpha=0.7)
# plt.title("Merged Grid Map")
# plt.xlabel("X Coordinate")
# plt.ylabel("Y Coordinate")
# plt.grid(True)

# # Save the plot as an image
# # plt.savefig("merged_grid_plot.png", dpi=300)
# plt.show()
# Scale the merged grid to 30x30
scaled_grid = zoom(merged_grid, (30 / merged_grid.shape[0], 30 / merged_grid.shape[1]), order=0)

# Visualization with a 30x30 grid overlay
plt.figure(figsize=(8, 8))
plt.imshow(scaled_grid.T, origin='lower', cmap='Greys', alpha=0.7)

# Add gray grid lines at integer intervals
plt.xticks(ticks=np.arange(31), labels=np.arange(31))
plt.yticks(ticks=np.arange(31), labels=np.arange(31))
plt.grid(color='gray', linestyle='-', linewidth=0.5)

# Add labels and title
plt.title("Scaled and Merged Grid Map (30x30)")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")

# Show the plot
plt.show()

output_path = 'merged_grid30.csv'
np.savetxt(output_path, scaled_grid, delimiter=',', fmt='%d')
