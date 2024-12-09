import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.optimize import leastsq

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
plt.figure(figsize=(10, 10))
plt.scatter(all_x, all_y, s=1, c='blue')

# Plot boundary points as black, but don't treat them as obstacles
min_x, min_y = np.min(all_x), np.min(all_y)
max_x, max_y = np.max(all_x), np.max(all_y)

# Plot the boundary (rectangular box)
plt.plot([min_x, min_x], [min_y, max_y], color='black', linewidth=2)  # Left
plt.plot([max_x, max_x], [min_y, max_y], color='black', linewidth=2)  # Right
plt.plot([min_x, max_x], [min_y, min_y], color='black', linewidth=2)  # Bottom
plt.plot([min_x, max_x], [max_y, max_y], color='black', linewidth=2)  # Top

# Step 4: Inflate and draw the circles around landmarks (exclude boundary)
for i, (xc, yc, r) in enumerate(fitted_circles):
    inflated_r = r + 0.1  # Inflate the radius of the circles

    # Only create the circle if the radius is less than 1
    if inflated_r < 1:
        circle = plt.Circle((xc, yc), inflated_r, color='orange', alpha=0.5)
        plt.gca().add_patch(circle)
        plt.plot(xc, yc, 'ro')

# Inflating the boundary (not affected by circle inflation)
inflation_radius = -0.4
inflated_x_min = min_x - inflation_radius
inflated_y_min = min_y - inflation_radius
inflated_x_max = max_x + inflation_radius
inflated_y_max = max_y + inflation_radius


# Create the coordinates for the original and inward-inflated rectangles
outer_x = [min_x, max_x, max_x, min_x, min_x]
outer_y = [min_y, min_y, max_y, max_y, min_y]
inner_x = [inflated_x_min, inflated_x_max, inflated_x_max , inflated_x_min , inflated_x_min]
inner_y = [inflated_y_min, inflated_y_min, inflated_y_max, inflated_y_max, inflated_y_min]

# Fill the area between bounds (shade cells between the rectangles)
plt.fill(np.concatenate([outer_x, inner_x[::-1]]), 
         np.concatenate([outer_y, inner_y[::-1]]), 
         color='orange', alpha=0.5)




# Plot the inflated boundary
plt.plot([inflated_x_min, inflated_x_min], [inflated_y_min, inflated_y_max], color='orange', linewidth=2)  # Left
plt.plot([inflated_x_max, inflated_x_max], [inflated_y_min, inflated_y_max], color='orange', linewidth=2)  # Right
plt.plot([inflated_x_min, inflated_x_max], [inflated_y_min, inflated_y_min], color='orange', linewidth=2)  # Bottom
plt.plot([inflated_x_min, inflated_x_max], [inflated_y_max, inflated_y_max], color='orange', linewidth=2)  # Top

plt.title("just let me die")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
