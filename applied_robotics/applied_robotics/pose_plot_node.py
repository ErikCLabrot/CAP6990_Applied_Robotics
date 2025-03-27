"""Pose Plot Node for CAP6990 Applied Robotics Assignment_6

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/27/2025

This module defines a ROS node that wraps plotting function for ground truth and 
lidar based odometry, using Matplotlib animation functions to provide the impression
of real time data plotting.
"""
import rclpy
import threading
import time
import math
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Quaternion, TransformStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style


class PosePlotNode(Node):
    '''
    Class that wraps a ROS node, as well as plotting functionality for both
    ground truth and additional odometrys. Additionally, it plots the difference
    between the two in terms of the magnitude of the error.

    Parameters:
        node_name: Name for the ROS Node
        ground_truth_sub: ROS Subscriber object for Gazebo ground truth
        lidar_odometry_sub: ROS Subscriber object for lidar based odometry
        ground_truth_x: List of ground truth x points
        ground_truth_y; List of ground truth y points
        lidar_odometry_x: List of lidar odometry x points
        lidar_odometry_y: List of lidar odometry y points
        error_magnitudes: List of magnitudinal difference between ground_truth and lidar odometry
        pose_fig: Matplotlib figure object for robot position
        pose_ax: Matplotlib axis object for robot position
        ground_truth_line: Line plot for ground truth
        lidar_odometry_line: Line plot for lidar odometry
        difference_fig: Matplotlib figure object for difference magnitude
        difference_ax: Matplotlib axis object for difference axis
        difference_line: Line plot for difference magnitude
        pose_anim: Matplotlib animation object for real time data plotting of robot poses
        difference_anim: Matplotlib animation object for real time data plotting of magnitude
    '''

    def __init__(self):
        '''
        Initializes class, starts ROS subscribers
        '''

        self.node_name = "pose_plot_node"
        super().__init__(self.node_name)

        self.ground_truth_topic = self.declare_parameter('ground_truth_topic', '/robot_pose').value
        self.lidar_odometry_topic = self.declare_parameter('lidar_odometry_topic', '/lidar/odometry').value

        self.ground_truth_sub = self.create_subscription(TFMessage, self.ground_truth_topic, self.ground_truth_callback, 1)
        self.lidar_odometry_sub = self.create_subscription(Odometry, self.lidar_odometry_topic, self.lidar_odometry_callback, 1)

        self.ground_truth_x = []
        self.ground_truth_y = []

        self.lidar_odometry_x = []
        self.lidar_odometry_y = []

        self.error_magnitudes = [0.0,0.0]

        self.pose_fig, self.pose_ax = plt.subplots()
        self.pose_ax.set_xlim(-10, 10) 
        self.pose_ax.set_ylim(-10, 10)         
        self.pose_ax.set_title("Pose plotted over iterations")
        self.ground_truth_line, = self.pose_ax.plot([], [], label='Ground Truth', color='blue', marker='o')
        self.lidar_odometry_line, = self.pose_ax.plot([], [], label='LiDAR Odometry', color='red', marker='o')
        self.pose_ax.legend()


        self.difference_fig, self.difference_ax = plt.subplots()
        self.difference_ax.set_ylim(0, 10) 
        self.difference_ax.set_xlim(0,100)
        self.difference_line, = self.difference_ax.plot([], [], label='Difference Magnitude', color='green')
        self.difference_ax.set_title("Difference between LiDAR Odometry and Ground Truth")
        self.difference_ax.set_xlabel("Iterations")
        self.difference_ax.set_ylabel("Magnitude")
        self.difference_ax.legend()

    def ground_truth_callback(self, msg):
        '''
        Callback function for ground truth ROS subscriber

        args:
            msg: Data in from ROS, ground truth as TFMessage to parse
        '''
        frame = "vehicle_blue"
        for tf in msg.transforms:
            if tf.child_frame_id == frame:
                self.ground_truth_x.append(tf.transform.translation.x)
                self.ground_truth_y.append(tf.transform.translation.y)

    def lidar_odometry_callback(self, odometry):
        '''
        Callback function for lidar odometry ROS subscriber

        args:
            odometry: Lidar Odometry data in from ros as nav_msgs/Odometry
        '''
        self.lidar_odometry_x.append(odometry.pose.pose.position.x)
        self.lidar_odometry_y.append(odometry.pose.pose.position.y)

        if len(self.ground_truth_x) > 0:
            dx = self.ground_truth_x[-1] - self.lidar_odometry_x[-1]
            dy = self.ground_truth_y[-1] - self.lidar_odometry_y[-1]
            diff = math.sqrt(dx**2 + dy**2)
            self.error_magnitudes.append(diff)

        if len(self.error_magnitudes) > 100:
                self.error_magnitudes.pop(0)

    def update_pose_plot(self, frame):
        '''
        Matplotlib animation update function for pose plot
        '''
        self.ground_truth_line.set_data(self.ground_truth_x, self.ground_truth_y)
        self.lidar_odometry_line.set_data(self.lidar_odometry_x, self.lidar_odometry_y)
        return self.ground_truth_line, self.lidar_odometry_line

    def update_difference_plot(self, frame):
        '''
        Matplotlib animation update function for magnitudinal difference plot
        '''
        self.difference_line.set_data(list(range(len(self.error_magnitudes))), self.error_magnitudes)
        return (self.difference_line,)

    def plot(self):
        '''
        Main plot function, creates animation functions and 'runs' them
        '''
        self.pose_anim = animation.FuncAnimation(self.pose_fig, self.update_pose_plot, interval=500, blit=True)
        self.difference_anim = animation.FuncAnimation(self.difference_fig, self.update_difference_plot, interval=500, blit=True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    ground_topic = '/robot_pose'
    lidar_odom_topic = '/lidar/odometry'
    node = PosePlotNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target = executor.spin, daemon=True)
    thread.start()
    node.plot()
    

if __name__ == '__main__':
    main()