#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import yaml
import os


def interpolate_3d(points, velocity, frequency):
    """Linearly interpolate between 3D points."""
    interpolated = []
    for i in range(len(points) - 1):
        p1 = np.array(points[i])
        p2 = np.array(points[i + 1])
        num_points_per_segment = int(np.linalg.norm(p2-p1)/(velocity)*frequency)
        for t in np.linspace(0, 1, num_points_per_segment, endpoint=False):
            interpolated.append(p1 + t * (p2 - p1))
    interpolated.append(points[-1])
    return np.array(interpolated)


class InterpolatorNode(Node):
    def __init__(self):
        super().__init__('interpolator_node')

        # Publishers
        self.point_pub = self.create_publisher(Point, 'pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'pose_marker', 10)
        self.path_pub = self.create_publisher(Path, 'path', 1)

        # Parameters
        self.declare_parameter('velocity', 1.0)       # meters per second
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('yaml_path', '')

        # Get parameter values
        self.velocity = self.get_parameter('velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        yaml_path = self.get_parameter('yaml_path').value

        #Load path from YAML 
        if yaml_path and os.path.exists(yaml_path):
            self.points = self.load_points_from_yaml(yaml_path)
            self.get_logger().info(f"Loaded {len(self.points)} points from {yaml_path}")
        else:
            self.get_logger().warn("No valid YAML path provided. Using default path.")
            self.points = [
                (0, 0, 0.5),
                (5, 0, 0.5),
                (10, 5, 0.5),
                (15, 10, 0.5),
                (10, 15, 0.5),
                (5, 10, 0.5),
                (0, 0, 0.5)
            ]
            
        self.path = interpolate_3d(self.points, self.velocity, self.publish_rate)

        # Compute distances and total path length
        self.segment_distances = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.total_distance = np.sum(self.segment_distances)

        # Derived timing
        self.dt = 1.0 / self.publish_rate
        self.dist_per_step = self.velocity * self.dt

        # Index and position trackers
        self.current_index = 0
        self.current_position = np.array(self.path[0])

        # Publish path once for visualization
        self.publish_path()

        # Start fixed-rate timer
        self.timer = self.create_timer(self.dt, self.publish_callback)
        self.get_logger().info(f"Started interpolator at {self.publish_rate} Hz, velocity = {self.velocity} m/s")
    
    def load_points_from_yaml(self, yaml_path):
        """Loads 3D waypoints from a YAML file."""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        # Expected format examples:
        # points:
        #   - [x, y, z]
        #   - [x, y, z]
        # OR:
        # points:
        #   - {x: 0.0, y: 0.0, z: 0.0}
        points_raw = data.get('points', [])
        points = []
        for p in points_raw:
            if isinstance(p, (list, tuple)) and len(p) == 3:
                points.append(tuple(p))
            elif isinstance(p, dict) and all(k in p for k in ['x', 'y', 'z']):
                points.append((p['x'], p['y'], p['z']))
            else:
                self.get_logger().warn(f"Invalid point format: {p}")
        return points
    
    def publish_path(self):
        """Publish the full path for visualization."""
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        for p in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(self.path)} points.")

    def publish_callback(self):
        """Move along the path and publish current position at fixed frequency."""
        # Calculate motion
        remaining_distance = self.dist_per_step
        while remaining_distance > 0:
            next_point = np.array(self.path[self.current_index + 1])
            segment = next_point - self.current_position
            seg_len = np.linalg.norm(segment)

            if remaining_distance < seg_len:
                # Move within this segment
                direction = segment / seg_len
                self.current_position += direction * remaining_distance
                remaining_distance = 0
            else:
                # Jump to next point
                self.current_position = next_point
                remaining_distance -= seg_len
                self.current_index += 1
                if self.current_index >= len(self.path) - 1:
                    self.current_index = 0  # loop back to start

        # --- Publish geometry_msgs/Point ---
        msg = Point()
        msg.x, msg.y, msg.z = self.current_position
        self.point_pub.publish(msg)

        # --- Publish visualization marker ---
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = msg.z
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 1.0, 0.2, 0.2
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = InterpolatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
