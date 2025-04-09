"""lidar_variance_measurement for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 4/8/2025

This module defines a simple script that listens to a number of samples from lidar odometry, then 
writes the mean and variance of the data to a file. This is used to calculate the 'white noise' 
in the lidar odometry for R in the EKF.
"""


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class LidarAnalysisNode(Node):
    '''
    ROS2 Node for listening to the lidar odometry, and calculating the mean and variance of it over n samples

    Params:
        lidar_odometry_topic: Topic that lidar odometry is published on
        lidar_odometry_sub: ROS2 subscriber object for lidar odometry
        num_samples: Number of samples to listen to (lidar is at 10hz, so num_seconds * 10 = num_samples)
        linear_velocities: List to store linear velocities in
        angular_velocities: List to store angular velocities in 
    '''
    def __init__(self):
        '''
        Initializes the lidar analysis node
        '''
        super().__init__('LidarOdometryWatcherNode')

        self.lidar_odometry_topic = self.declare_parameter('lidar_odometry_topic', '/lidar/odometry').value

        self.lidar_odometry_sub = self.create_subscription(Odometry, self.lidar_odometry_topic, self.lidar_odometry_callback, 1)

        self.num_samples = 300
        self.linear_velocities = []
        self.angular_velocities = []

    def lidar_odometry_callback(self, odometry):
        '''
        Callback function for the lidar odometry. Stores readings as they come in.
        When n samples are reached, we write to file and shut down.
        
        args:
            odometry: lidar odometry message from ros
        '''
        v = odometry.twist.twist.linear.x
        w = odometry.twist.twist.angular.z

        self.linear_velocities.append(v)
        self.angular_velocities.append(w)

        self.get_logger().info(f'{len(self.linear_velocities)}')

        if len(self.linear_velocities) >= self.num_samples:
            self.compute_variance()
            rclpy.shutdown()

    def compute_variance(self):
        '''
        Function to write mean and variance of data to file
        '''
        linear_array = np.array(self.linear_velocities)
        angular_array = np.array(self.angular_velocities)

        linear_mean = linear_array.mean()
        linear_variance = linear_array.var()

        angular_mean = angular_array.mean()
        angular_variance = angular_array.var()

        output = (
            f"LiDAR Odometry Velocity Statistics (over {self.num_samples} samples):\n"
            f"Linear Velocity (v): Mean = {linear_mean:.6f}, Variance = {linear_variance:.6f}\n"
            f"Angular Velocity (w): Mean = {angular_mean:.6f}, Variance = {angular_variance:.6f}\n"
        )

        with open('lidar_variance.txt', 'w') as f:
            f.write(output)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()