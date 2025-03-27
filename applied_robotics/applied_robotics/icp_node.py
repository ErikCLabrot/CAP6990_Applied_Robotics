"""ICP Node for CAP6990 Applied Robotics Assignment_6

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/27/2025

This module defines a ROS node that wraps an implementation of the ICP algorithm, and uses the 
output to integrate a positional estimate over time and publish an Odometry message containing current 
position and velocity.
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from sensor_msgs.msg import LaserScan
from applied_robotics.icp import ICP

class ICPNode(Node):
	'''
	ROS2 Node that wraps the ICP class functionality in to the ros system.
	Subscribes to laser scans from a robot system, and publishes lidar based
	odometry to the ROS network.

	Attributes:
		node_name: The name of the node in the ROS system
		lidar_topic: The name of the ROS topic for LaserScans
		lidar_sub: The ROS Subscriber object for LaserScans
		lidar_odometry_topic: The name of the ROS topic for LiDAR Odometry
		lidar_odometry_pub: The ROS Publisher object for Odometry
		icp: The ICP class that computes ICP laser scan registration
		last_scan: The last scan received for frame-to-frame comparisons
		last_scan_time: The ROS time the last scan was received at
		current_pose: Current robot position and orientation
		current_twist: Current robot linear and angular velocities
		current_transform: Current homogeneous transform from robots starting position
		last_pose: Pose of the robot's previous update frame
	'''
	def __init__(self):
		'''
		Initializes ICPNode with given parameters

		args:
			lidar_topic: The topic the lidar lasercans are published to
			odometry_topic: the topic the odometry should be published to
		'''

		self.node_name = 'lidar_odometry_node'
		super().__init__(self.node_name)

		#Create Lidar Subscription
		self.lidar_topic = self.declare_parameter('lidar_topic', '/lidar/scan').value
		self.lidar_sub = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 1)

		#Create Odometry Publisher
		self.lidar_odometry_topic = self.declare_parameter('lidar_odometry_topic', '/lidar/odometry').value
		self.lidar_odometry_pub = self.create_publisher(Odometry, self.lidar_odometry_topic, 1)

		#Create iterative closest point class
		self.icp = ICP()

		#scan management
		self.last_scan = None 
		self.last_scan_time = None

		#Odometry management
		self.current_pose = Pose(position = Point(x=-8.0, y=-8.0, z=0.0), orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
		self.current_twist = Twist(linear = Vector3(x=0.0, y=0.0, z=0.0), angular = Vector3(x=0.0, y=0.0, z=0.0))

		yaw = self.yaw_from_quaternion(self.current_pose.orientation)

		initial_rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                                    [np.sin(yaw), np.cos(yaw)]])

		initial_translation = np.array([[self.current_pose.position.x], [self.current_pose.position.y]])

		self.current_transform = np.eye(3)
		self.current_transform[:2, :2] = initial_rotation
		self.current_transform[:2, 2] = initial_translation.T
		self.last_pose = None

	def yaw_from_quaternion(self, quat):
		'''
		Function to convert quaternion to euler yaw (roll/pitch ignored, 2D case)

		args:
			quat: Input quaternion
		returns:
			yaw retrieved from quaternion
		'''
		a = 2.0 * (quat.w * quat.z + quat.x * quat.y)
		b = 1.0 - 2.0 * (quat.y**2 + quat.z**2)
		return np.arctan2(a, b)

	def quaternion_from_yaw(self, yaw):
		'''
		Function to convert euler yaw to a quaternion (roll/pitch ignored, 2D case)

		args:
			yaw: Input euler angle about Z axis
		returns:
			quaternion with only yaw angle considered
		'''
		half_angle = yaw/2.0
		z = np.sin(half_angle)
		w = np.cos(half_angle)
		quat = Quaternion(x=0.0, y=0.0, z=z, w=w)
		return quat

	def update_pose(self, transform):
		'''
		Function to update current pose with homogeneous transform received from ICP

		args:
			transform: homoegeneous transform to update current pose with
		'''
	
		self.last_pose = self.current_pose

		self.current_transform = np.dot(self.current_transform, transform)

		#Update current position
		updated_x, updated_y = self.current_transform[:2, 2]
		updated_position = Point(x = updated_x, y = updated_y, z = 0.0)
		self.current_pose.position = updated_position

		#Update current rotation
		updated_rotation = self.current_transform[:2, :2]
		updated_theta = np.arctan2(updated_rotation[1,0], updated_rotation[0,0])
		updated_orientation = self.quaternion_from_yaw(yaw = updated_theta)

		self.current_pose.orientation = updated_orientation

	def calculate_velocity(self, scan_time):
		'''
		Calculates velocity between frames, updates internal Twist

		args:
			scan_time: timestamp for the latest scan
		'''
		if self.last_scan is None:
			return

		if self.last_pose is None:
			return

		last_scan_time = self.last_scan.header.stamp

		dt = (scan_time.sec - last_scan_time.sec) + ((scan_time.nanosec - last_scan_time.nanosec) * 1e-9)
		
		if dt == 0:
			return

		#Calculate linear velocity
		dx = self.current_pose.position.x - self.last_pose.position.x
		dy = self.current_pose.position.y - self.last_pose.position.y
		linear_velocity = math.sqrt(dx**2 + dy**2) / dt

		#Calculate angular velocity
		last_yaw = self.yaw_from_quaternion(self.last_pose.orientation)
		current_yaw = self.yaw_from_quaternion(self.current_pose.orientation)
		angular_velocity = (current_yaw - last_yaw) / dt

		#Update Twist
		self.current_twist.linear.x = linear_velocity
		self.current_twist.angular.z = angular_velocity

		self.last_scan_time = scan_time

	def publish_odometry(self):
		'''
		Publish odometry message containing latest pose and twist
		'''
		odometry_message = Odometry()
		odometry_message.pose.pose = self.current_pose
		odometry_message.twist.twist = self.current_twist

		self.lidar_odometry_pub.publish(odometry_message)

	def lidar_callback(self, scan):
		'''
		Receive latest laser scan, update pose and twist using ICP, and publish as odometry

		args:
			scan: laserscan from lidar received through ROS subscription
		'''

		#Needs investigation: First laserscan from Gazebo seems to be buggy. Ignore it, and ICP works.
		scan_time = scan.header.stamp

		if self.last_scan is not None:
			Transform = self.icp.compute_registration(source_scan = scan, target_scan = self.last_scan)
			self.update_pose(transform = Transform)
			self.calculate_velocity(scan_time)
			self.publish_odometry()

		self.last_scan = scan

def main(args=None):
	rclpy.init(args=args)
	lidar_topic = '/lidar/laserscan'
	odometry_topic = '/lidar/odometry'

	lidar_odometry_node = ICPNode()

	rclpy.spin(lidar_odometry_node)
	lidar_odometry_node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()