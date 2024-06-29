#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # topic name
            self.scan_callback,  # callback function
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Process the laser scan message
        ranges = msg.ranges
        # Do something with the laser scan data
        # For example, print the first range value
        if ranges:
            print("Distance at the first angle:", ranges[0])

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

