import numpy as np
import matplotlib.pyplot as plt
import csv

# Input points for ground truth trajectory (open path)

ground_truth_points = np.array([
    [1.54, -1.02],
    [1.38, -1.74],
    [0.61, -1.56],
    [0.76, -0.76],
    [1.34, -0.81]
])

# Generate additional points for smooth ground truth plotting
def interpolate_points(points, num=100):
    smooth_points = []
    for i in range(len(points) - 1):
        x = np.linspace(points[i, 0], points[i+1, 0], num)
        y = np.linspace(points[i, 1], points[i+1, 1], num)
        smooth_points.append(np.column_stack((x, y)))
    return np.vstack(smooth_points)

ground_truth_smooth = interpolate_points(ground_truth_points)

# Read data from CSV files
def read_csv(filepath):
    data = []
    with open(filepath, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # print(row)
            data.append([float(row[0]), float(row[1])])
    return np.array(data)


landmark_file = "data2.csv"
dead_reckoning_file = "data1.csv"

landmark_data = read_csv(landmark_file)
dead_reckoning_data = read_csv(dead_reckoning_file)

# Function to apply a rotation matrix with an arbitrary angle
def rotate_data(data, degrees):
    radians = np.radians(degrees)  # Convert degrees to radians
    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians), np.cos(radians)]
    ])
    return np.dot(data, rotation_matrix.T)

dead_reckoning_data = rotate_data(dead_reckoning_data, -95)

# Plot all data
plt.figure(figsize=(10, 6))
plt.plot(ground_truth_smooth[:, 0], ground_truth_smooth[:, 1], label='Ground Truth Values', color='blue', linewidth=2)
plt.plot(landmark_data[:, 0], landmark_data[:, 1], label='Landmark-Based Method', color='red', linestyle=':')

# plt.scatter(landmark_data[:, 0], landmark_data[:, 1])
plt.plot(dead_reckoning_data[:, 0]+1.55, dead_reckoning_data[:, 1]-1,label='Dead-Reckoning Method', color='green', linestyle=':')

# Graph formatting
plt.xlabel("X Coordinate (meters)")
plt.ylabel("Y Coordinate (meters)")
plt.title("Robot Trajectories")
plt.legend(loc="best")
plt.grid(True)
plt.axis("equal")
plt.show()
