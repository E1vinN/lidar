#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class OmniRobot(Node):
	def __init__(self):
		super().__init__('wall_centering_obstacle_avoidance')
		self.subscription = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			10)
		self.publisher = self.create_publisher(
			Twist,
			'/cmd_vel',
			10)

		#cb variables:
		self.valid_ranges = []
		self.min_angle = 0
		self.angle_increment = 0
		self.front_range = 0
		self.back_range = 0
		self.right_range = 0
		self.left_range= 0
		self.orientation = 0
		self.current_range = 0.0
		self.updated_cb = False

		#center variables:
		# self.close_wall_1_range = 0
		# self.close_wall_2_range = 0
		# self.far_wall_1_range = 0
		# self.far_wall_2_range= 0

	def stop_omni_robot(self):
		twist = Twist()

		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0

		self.publisher.publish(twist)

	def rotate_omni_robot(self, rotation_angle, angular_speed, angle_range):
		twist = Twist()
		# Set angular speed for rotation; the direction is based on the sign of rotation_angle
		if rotation_angle < np.pi:
			twist.angular.z = -1 * angular_speed
		else:
			twist.angular.z = angular_speed
        
		while not np.isclose(self.current_range, angle_range, rtol=0.001):
			print(self.current_range)
			print(angle_range)
			print()
			self.publisher.publish(twist)
			rclpy.spin_once(self, timeout_sec=0.5)
		
		self.stop_omni_robot()
		self.orientation = (self.orientation + rotation_angle) % (2*np.pi)
		if self.orientation < self.min_angle:
			self.orientation += 2*np.pi

	def move_omni_robot(self, speed):
		twist = Twist()
		twist.linear.x = speed
		self.publisher.publish(twist)

	def lidar_callback(self, msg):
		self.min_angle = msg.angle_min
		self.angle_increment = msg.angle_increment
		ranges = np.array(msg.ranges)
		self.valid_ranges = np.where(np.isinf(ranges) | np.isnan(ranges), np.inf, ranges)
		self.current_range = self.valid_ranges[0]

		# Finding the front wall:
		front_angle = self.orientation % (2*np.pi)
		if front_angle < self.min_angle:
			front_angle += 2*np.pi

		front_index = int((front_angle - self.min_angle)/self.angle_increment) % len(ranges)
		self.front_range = self.valid_ranges[front_index]

		# min_index = np.argmin(self.valid_ranges)
		# self.wall_1_range = self.valid_ranges[min_index]
		
		# Find the wall opposite to it
		opposite_angle = (np.pi + self.orientation) % (2*np.pi)
		if opposite_angle < self.min_angle:
			opposite_angle += 2 * np.pi

		opposite_index = int((opposite_angle - self.min_angle)/self.angle_increment) % len(ranges)
		self.back_range = self.valid_ranges[opposite_index]
		
		# Find the wall to the right
		right_angle = (np.pi/2 + self.orientation) % (2*np.pi)
		if right_angle < self.min_angle:
			right_angle += 2 * np.pi
	
		right_index = int((right_angle - self.min_angle)/self.angle_increment) % len(ranges)
		self.right_range = self.valid_ranges[right_index]
		
		# Find the wall to the left
		left_angle = (1.5 * np.pi + self.orientation) % (2*np.pi)
		if left_angle < self.min_angle:
			left_angle += 2 * np.pi

		left_index = int((left_angle - self.min_angle)/self.angle_increment) % len(ranges)
		self.left_range = self.valid_ranges[left_index]
		self.updated_cb = True
		
		# # Find the closest pair of walls:
		# print("Left: ", self.far_wall_2_range)
		# print("Right: ", self.far_wall_1_range)
		# print("Front: ",self.close_wall_1_range)
		# print("Opposite: ",self.close_wall_2_range)
		
		# pair_one = min_range + opposite_range
		# pair_two = left_range + right_range
		
		# wall_1_range = None
		# wall_2_range = None
		
		# if pair_one >= pair_two:
		# 	wall_1_range = left_range
		# 	wall_2_range = right_range
		# else:
		# 	wall_1_range = min_range
		# 	wall_2_range = opposite_range
		
		# # Move to the middle
		# mean_range = (wall_1_range + wall_2_range)/2
		# target_range = np.abs(mean_range - wall_1_range)

	# def define_walls(self):
	# 	pair_1 = self.wall_1_range + self.wall_2_range
	# 	pair_2 = self.wall_3_range + self.wall_4_range

	# 	if pair_1 > pair_2:
	# 		self.far_wall_1_range = self.wall_1_range
	# 		self.far_wall_2_range = self.wall_2_range
	# 		self.close_wall_1_range = self.wall_3_range
	# 		self.close_wall_2_range = self.wall_4_range

	# 	else:
	# 		self.far_wall_1_range = self.wall_3_range
	# 		self.far_wall_2_range = self.wall_4_range
	# 		self.close_wall_1_range = self.wall_1_range
	# 		self.close_wall_2_range = self.wall_2_range

	def center_robot(self):
		while not self.updated_cb:
			rclpy.spin_once(self,timeout_sec=0.5)
		self.updated_cb = False

		turn_range = self.right_range
		turn_angle = np.pi/2
		if self.left_range < self.right_range:
			turn_range = self.left_range
			turn_angle = 1.5 * np.pi

		print("Front: ", self.front_range, "Back: ", self.back_range, "Right :", self.right_range, "Left :", self.left_range)
		print("Turning for centering")
		self.rotate_omni_robot(turn_angle, 0.1, turn_range)
		rclpy.spin_once(self,timeout_sec=1)
		while np.abs(self.left_range - self.right_range) > 0.1:
			if not self.updated_cb:
				self.stop_omni_robot()
				rclpy.spin_once(self,timeout_sec=0.5)
	
			else:
				self.move_omni_robot(0.2)
				rclpy.spin_once(self,timeout_sec = 0.2)
				self.updated_cb = False
		self.stop_omni_robot()
		print("Turning for moving")
		print("Front: ", self.front_range, "Back: ", self.back_range, "Right :", self.right_range, "Left :", self.left_range)
		self.rotate_omni_robot(np.pi/2, 0.2, self.front_range)
		rclpy.spin_once(self, timeout_sec=1)
		print("Orientation: ",self.orientation)
		print("Front: ", self.front_range, "Back: ", self.back_range, "Right :", self.right_range, "Left :", self.left_range)
	
if __name__ == '__main__':
	# main()
	rclpy.init()
	omni = OmniRobot()
	omni.center_robot()
	# while True:
	# 	rclpy.spin_once(omni,timeout_sec=0.5)
	# 	if np.abs(omni.left_range - omni.right_range) > 0.2:
	# 		omni.stop_omni_robot()
	# 		omni.center_robot()
	# 	else:
	# 		omni.move_omni_robot(0.4)