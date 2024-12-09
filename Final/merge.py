import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

# Plot the combined map
plt.figure(figsize=(10, 10))
plt.scatter(all_x, all_y, s=1, c='blue')
plt.title("Combined Grid Map")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
