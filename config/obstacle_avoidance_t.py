#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your LiDAR topic name
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',  # Replace with your robot's velocity command topic name
            10)
        self.min_distance = 0.5  # Minimum distance to maintain from obstacles

    def lidar_callback(self, msg):
        # Process LiDAR data to detect obstacles and calculate steering command
        ranges = np.array(msg.ranges)
        closest_obstacle_index = np.argmin(ranges)
        closest_obstacle_distance = ranges[closest_obstacle_index]
        steering_angle = msg.angle_min + closest_obstacle_index * msg.angle_increment

        # Calculate linear velocity based on the distance to the closest obstacle
        linear_velocity = 0.2 if closest_obstacle_distance > self.min_distance else 0.0

        # Publish steering command
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = -0.5 * steering_angle  # Proportional control for steering
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

