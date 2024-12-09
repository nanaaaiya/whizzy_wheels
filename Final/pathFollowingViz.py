import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, TransformStamped
import tf2_ros

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Parameters
        self.path = np.load('path.npy')  # Load saved path
        self.grid_map = np.load('grid_map.npy')  # Load grid map
        self.grid_size = 0.02  # Grid resolution in meters per cell
        self.pose = np.array([self.path[0][0] * self.grid_size, self.path[0][1] * self.grid_size, 0])  # Start pose based on the first path point
        self.prev_scan = None  # Store the previous scan
        self.path_index = 0
        self.goal_tolerance = 0.4  # Tolerance for reaching a path point
        self.ahead_distance = 0.5  # Distance to look ahead on the path

        # TF broadcaster for map -> odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers and Publishers
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.grid_map_publisher = self.create_publisher(OccupancyGrid, '/grid_map', 10)
        self.path_publisher = self.create_publisher(Marker, '/path_marker', 10)
        self.robot_marker_publisher = self.create_publisher(Marker, '/robot_marker', 10)
        self.ahead_point_publisher = self.create_publisher(Marker, '/robot_ahead_point_marker', 10)

        # Timer to periodically publish the map and path
        self.create_timer(1.0, self.publish_grid_map)
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_robot_marker)
        self.create_timer(1.0, self.publish_ahead_point_marker)
        self.create_timer(0.1, self.publish_transform)  # Publish TF at 10 Hz

        # Add the publisher for the closest point marker
        self.closest_point_publisher = self.create_publisher(Marker, '/closest_point_marker', 10)
        self.create_timer(1.0, self.publish_closest_point_marker)

        self.get_logger().info('Path Follower Node has started.')

    def publish_grid_map(self):
        """
        Publish the grid map as an OccupancyGrid message.
        """
        occupancy_grid = OccupancyGrid()

        # Header
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()

        # Map metadata
        occupancy_grid.info.resolution = self.grid_size  # Map resolution in meters per cell
        occupancy_grid.info.width = self.grid_map.shape[1]
        occupancy_grid.info.height = self.grid_map.shape[0]
        occupancy_grid.info.origin = Pose()
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0

        # Flatten and normalize the grid map to match OccupancyGrid format
        grid_data = (self.grid_map.flatten() * 100).astype(np.int8)  # Scale [0, 1] to [0, 100]
        grid_data = np.clip(grid_data, 0, 100).astype(np.int8)  # Clamp to [0, 100]
        occupancy_grid.data = [int(value) if 0 <= value <= 100 else -1 for value in grid_data]  # Use -1 for unknown cells

        self.grid_map_publisher.publish(occupancy_grid)

    def publish_path(self):
        """
        Publish the path as a Marker message.
        """
        path_marker = Marker()

        # Header
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()

        # Marker properties
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02  # Line width in meters

        # Path color (red)
        path_marker.color.r = 1.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        # Convert path points to geometry_msgs/Point
        for point in self.path:
            path_point = Point()
            path_point.x = point[0] * self.grid_size  # Scale grid to world coordinates
            path_point.y = point[1] * self.grid_size
            path_point.z = 0.0
            path_marker.points.append(path_point)

        self.path_publisher.publish(path_marker)

    def publish_robot_marker(self):
        # Create a Marker message for the robot position
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'robot'
        robot_marker.id = 1
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.color.a = 1.0  # Fully opaque
        robot_marker.color.g = 1.0  # Green
        robot_marker.scale.x = 0.3  # Length of the arrow
        robot_marker.scale.y = 0.05  # Width of the arrow
        robot_marker.scale.z = 0.05  # Height of the arrow
        robot_marker.pose.position.x = self.pose[0]
        robot_marker.pose.position.y = self.pose[1]
        robot_marker.pose.position.z = 0.1  # Slightly above the ground

        # Set the orientation of the arrow to match the robot's theta
        quaternion = R.from_euler('z', self.pose[2]).as_quat()
        robot_marker.pose.orientation.x = quaternion[0]
        robot_marker.pose.orientation.y = quaternion[1]
        robot_marker.pose.orientation.z = quaternion[2]
        robot_marker.pose.orientation.w = quaternion[3]



        # Publish the robot marker
        self.robot_marker_publisher.publish(robot_marker)

    def publish_ahead_point_marker(self):
        # Get the ahead point
        ahead_point = self.get_ahead_point(self.pose)

        # Create a Marker message for the ahead point
        ahead_point_marker = Marker()
        ahead_point_marker.header.frame_id = 'map'
        ahead_point_marker.header.stamp = self.get_clock().now().to_msg()
        ahead_point_marker.ns = 'ahead_point'
        ahead_point_marker.id = 2
        ahead_point_marker.type = Marker.SPHERE
        ahead_point_marker.action = Marker.ADD
        ahead_point_marker.color.a = 1.0  # Fully opaque
        ahead_point_marker.color.b = 1.0  # Blue
        ahead_point_marker.scale.x = 0.1  # Diameter of the sphere
        ahead_point_marker.scale.y = 0.1
        ahead_point_marker.scale.z = 0.1
        ahead_point_marker.pose.position.x = ahead_point[0]
        ahead_point_marker.pose.position.y = ahead_point[1]
        ahead_point_marker.pose.position.z = 0.1  # Slightly above the ground

        # Publish the ahead point marker
        self.ahead_point_publisher.publish(ahead_point_marker)

    def publish_closest_point_marker(self):
        # Get the ahead point
        ahead_point = self.get_ahead_point(self.pose)

        # Find the closest path point to the ahead point
        closest_point, _ = self.find_closest_path_point(ahead_point)

        # Create a Marker message for the closest point
        closest_point_marker = Marker()
        closest_point_marker.header.frame_id = 'map'
        closest_point_marker.header.stamp = self.get_clock().now().to_msg()
        closest_point_marker.ns = 'closest_point'
        closest_point_marker.id = 3
        closest_point_marker.type = Marker.SPHERE
        closest_point_marker.action = Marker.ADD
        closest_point_marker.color.a = 1.0  # Fully opaque
        closest_point_marker.color.r = 1.0  # Red
        closest_point_marker.scale.x = 0.1  # Diameter of the sphere
        closest_point_marker.scale.y = 0.1
        closest_point_marker.scale.z = 0.1
        closest_point_marker.pose.position.x = closest_point[0]
        closest_point_marker.pose.position.y = closest_point[1]
        closest_point_marker.pose.position.z = 0.1  # Slightly above the ground

        # Publish the closest point marker
        self.closest_point_publisher.publish(closest_point_marker)

    def publish_transform(self):
        """
        Publish the map -> odom transformation.
        """
        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        # Set the transformation based on the robot's pose
        transform.transform.translation.x = 1.0 # self.pose[0]
        transform.transform.translation.y = 1.0 # self.pose[1]
        transform.transform.translation.z = 0.0

        # Convert yaw to quaternion
        quaternion = R.from_euler('z', self.pose[2]).as_quat()
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform)

    def lidar_callback(self, msg):
        # Convert LaserScan data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        points = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ]).T

        # Remove invalid points (e.g., range = 0)
        points = points[np.linalg.norm(points, axis=1) > 0]

        if self.prev_scan is not None:
            # Apply ICP to calculate the relative transformation
            delta_pose = self.icp(self.prev_scan, points)
            self.pose = self.update_pose(self.pose, delta_pose)
            self.get_logger().info(f'Updated Pose: {self.pose}')

            # Publish the robot marker to visualize the position
            self.publish_robot_marker()

            # Control the robot to follow the path
        #    self.control_robot()

        self.prev_scan = points

    def icp(self, source, target, max_iterations=200, tolerance=1e-15):
        """
        Perform ICP between source and target point clouds.
        """
        source_copy = source.copy()
        transformation = np.eye(3)

        for i in range(max_iterations):
            # Find the nearest neighbors between the source and target
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target)
            distances, indices = nbrs.kneighbors(source_copy)

            # Compute the centroids of the matched points
            target_matched = target[indices[:, 0]]
            source_centroid = np.mean(source_copy, axis=0)
            target_centroid = np.mean(target_matched, axis=0)

            # Subtract centroids to align the points
            source_centered = source_copy - source_centroid
            target_centered = target_matched - target_centroid

            # Compute the optimal rotation using SVD
            H = np.dot(source_centered.T, target_centered)
            U, _, Vt = np.linalg.svd(H)
            R_opt = np.dot(Vt.T, U.T)

            # Ensure R_opt is a proper rotation matrix
            if np.linalg.det(R_opt) < 0:
                Vt[1, :] *= -1
                R_opt = np.dot(Vt.T, U.T)

            # Compute the translation
            t_opt = target_centroid - np.dot(source_centroid, R_opt)

            # Update the transformation matrix
            current_transform = np.eye(3)
            current_transform[:2, :2] = R_opt
            current_transform[:2, 2] = t_opt
            transformation = np.dot(current_transform, transformation)

            # Apply the transformation to the source points
            source_copy = (np.dot(R_opt, source_copy.T).T + t_opt)

            # Check for convergence
            mean_error = np.mean(distances)
            if mean_error < tolerance:
                break

        # Extract translation and rotation (angle) from the final transformation
        dx = transformation[0, 2]
        dy = transformation[1, 2]
        dtheta = np.arctan2(transformation[1, 0], transformation[0, 0])

        return np.array([-dx, dy, -dtheta])

    def update_pose(self, pose, delta_pose):
        """
        Update the robot's pose based on the delta pose.
        """
        dx, dy, dtheta = delta_pose
        theta = pose[2]

        # Update the pose with respect to the robot's current orientation
        x_new = pose[0] + dx * np.cos(theta) - dy * np.sin(theta)
        y_new = pose[1] + dx * np.sin(theta) + dy * np.cos(theta)
        theta_new = (pose[2] + dtheta + np.pi) % (2 * np.pi) - np.pi  # update and limit angle to [-pi, pi]

        return np.array([x_new, y_new, theta_new])

    def get_ahead_point(self,pose):
        # NEED TO FIND THE AHEAD POINT
        # x_ap = 1.2
        # y_ap = 0.8

        x, y, theta = pose

        # change self.ahead_distance above 
        x_ap = x + self.ahead_distance * np.cos(theta)
        y_ap = y + self.ahead_distance * np.sin(theta)

        return np.array([x_ap, y_ap])

    # def find_closest_path_point(self, ahead_point):
    #     """
    #     WRITE YOUR CODE HERE
    #     Find the point on the path closest to the ahead point.
    #     """ 
    #     # closest_point = np.array([1.2, 1.0])
    #     # return closest_point, 0

    def find_closest_path_point(self, ahead_point):
        # """
        # Find the point on the path closest to the ahead point.
        # """
        # Initialize the minimum distance to a large value
        min_distance = float('inf')
        closest_point = None
        
        # Iterate over all path points
        for path_point in self.path:
            # Scale the path point according to grid size (if necessary)
            path_point_scaled = np.array([path_point[0] * self.grid_size, path_point[1] * self.grid_size])

            # print(path_point_scaled)
            # Calculate the Euclidean distance between the ahead point and the path point
            distance = np.linalg.norm(ahead_point - path_point_scaled)
            # print(distance)
            # If this distance is the smallest so far, update the closest point
            if distance < min_distance:
                min_distance = distance
                closest_point = path_point_scaled

            print(closest_point)

        return closest_point, 0

    def calculate_distance_to_line(self, robot_pose, ahead_point, closest_point):
        """
        Calculate the perpendicular distance from the closest_point to the line
        connecting robot_pose and ahead_point.

        Args:
            robot_pose: numpy array [x, y] of the robot's current position.
            ahead_point: numpy array [x, y] of the ahead point.
            closest_point: numpy array [x, y] of the closest point.

        Returns:
            distance: The perpendicular distance from closest_point to the line.
        """
        # Line vector from robot_pose to ahead_point
        line_vector = ahead_point - robot_pose

        # Vector from robot_pose to closest_point
        point_vector = closest_point - robot_pose

        # Compute the cross product magnitude (area of the parallelogram)
        cross_product = np.abs(line_vector[0] * point_vector[1] - line_vector[1] * point_vector[0])

        # Compute the line vector magnitude (length of the base)
        line_length = np.linalg.norm(line_vector)

        # Calculate perpendicular distance (area divided by base length)
        if line_length == 0:  # Avoid division by zero
            return np.linalg.norm(point_vector)
        
        distance = cross_product / line_length
        return distance

    

    def control_robot(self):
        """
        WRITE YOUR CODE HERE
        Control the robot to follow the path using a simple proportional controller.
        """

        

    def stop_robot(self):
        """
        Stop the robot by publishing a zero Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
