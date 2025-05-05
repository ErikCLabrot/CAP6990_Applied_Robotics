'''
PoseGraphLogger Node for CAP6990 Assignment 7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This node logs pose graph data for offline SLAM optimization
It subscribes to odometry, commands, laser scans, landmark observations, and TF ground truth
It writes VERTEX_SE2, EDGE_SE2, CMD, and EDGE_SE2_XY entries to a log file
It also writes raw ground truth poses and de-skewed 2D laser scans for later analysis
'''

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from applied_robotics_utilities.msg import LandmarkArray

class PoseGraphLogger(Node):
    def __init__(self):
        '''initialize logger node, declare parameters, set up subscriptions and file handles'''
        super().__init__('graph_logger')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/lidar/scan').value
        self.ekf_state_topic = self.declare_parameter('ekf_state_topic', '/filtered_odometry').value
        self.landmark_topic = self.declare_parameter('landmark_observation_topic', '/landmarks').value
        self.command_topic = self.declare_parameter('command_topic', '/cmd_vel').value
        self.lidar_sample_sigma = self.declare_parameter('lidar_noise_sigma', 0.01).value
        self.ground_truth_topic = self.declare_parameter('ground_truth_topic', '/robot_pose').value
        self.robot_frame = self.declare_parameter('robot_frame', 'vehicle_blue').value

        self.file_name = self.declare_parameter('graph_log_file', 'pose_graph_log.txt').value
        self.ground_truth_file_name = 'ground_truth_poses.txt'
        self.laserscan_filename = 'laserscans.txt'

        self.odometry_subscriber = self.create_subscription(Odometry, self.ekf_state_topic, self.odometry_callback, 1)
        self.command_subscriber = self.create_subscription(Twist, self.command_topic, self.command_callback, 1)
        self.observation_subscriber = self.create_subscription(LandmarkArray, self.landmark_topic, self.observation_callback, 1)
        self.ground_truth_subscriber = self.create_subscription(TFMessage, self.ground_truth_topic, self.ground_truth_callback,1)
        self.laserscan_subsriber = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 1)

        self.log_file = open(self.file_name, 'w')
        self.ground_truth_file = open(self.ground_truth_file_name, 'w')
        self.laserscan_file = open(self.laserscan_filename, 'w')

        self.current_odometry = None
        self.last_odometry = None
        self.pose_id = 0

        self.observation_flag = False
        self.command_flag = False

        self.linear_command = 0.0
        self.angular_command = 0.0

    def odometry_callback(self, odometry):
        '''process incoming EKF odometry, log pose, edge, and command, trigger landmark and scan logging'''
        if self.last_odometry is None:
            self.current_odometry = odometry
            current_x = odometry.pose.pose.position.x
            current_y = odometry.pose.pose.position.y
            current_theta = 2 * math.atan2(odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
            self.log_file.write(f"VERTEX_SE2 {self.pose_id} {current_x} {current_y} {current_theta}\n")
            self.ground_truth_file.write(f"{self.ground_truth_pos.x} {self.ground_truth_pos.y} {self.ground_truth_orientation}\n")
            self.last_odometry = odometry
            self.pose_id += 1
            return

        self.get_logger().info(f'{odometry}')
        self.current_odometry = odometry

        current_x = odometry.pose.pose.position.x
        current_y = odometry.pose.pose.position.y
        current_theta = 2 * math.atan2(odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)

        last_x = self.last_odometry.pose.pose.position.x
        last_y = self.last_odometry.pose.pose.position.y
        last_theta = 2 * math.atan2(self.last_odometry.pose.pose.orientation.z, self.last_odometry.pose.pose.orientation.w)

        covariance_matrix = np.array(odometry.pose.covariance).reshape((6,6))
        pose_covariance = covariance_matrix[:3, :3]
        info_matrix = np.linalg.pinv(pose_covariance)

        dx = current_x - last_x
        dy = current_y - last_y
        dtheta = current_theta - last_theta

        x_transform = math.cos(dtheta) * dx - math.sin(dtheta) * dy
        y_transform = math.sin(dtheta) * dx + math.cos(dtheta) * dy

        self.log_file.write(f"VERTEX_SE2 {self.pose_id} {current_x} {current_y} {current_theta}\n")
        self.log_file.write(f"EDGE_SE2 {self.pose_id - 1} {self.pose_id} {x_transform} {y_transform} {dtheta} {info_matrix[0,0]} {info_matrix[0,1]} {info_matrix[0,2]} {info_matrix[1,1]} {info_matrix[1,2]} {info_matrix[2,2]}\n")
        self.log_file.write(f"CMD {self.pose_id -1} {self.pose_id} {self.linear_command} {self.angular_command}\n")

        self.ground_truth_file.write(f"{self.ground_truth_pos.x} {self.ground_truth_pos.y} {self.ground_truth_orientation}\n")
        self.laserscan_to_file(scan_msg=self.latest_scan)

        self.last_odometry = odometry
        self.pose_id += 1
        self.observation_flag = True

    def laserscan_to_file(self, scan_msg):
        '''deskew and flatten a LaserScan to file line

        args:
            scan_msg: incoming LaserScan message
        '''
        scan_ranges = np.array(scan_msg.ranges)
        scan_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_ranges))
        valid_mask = np.isfinite(scan_ranges)
        scan_ranges = scan_ranges[valid_mask]
        scan_angles = scan_angles[valid_mask]

        points = np.vstack((scan_ranges * np.cos(scan_angles), scan_ranges * np.sin(scan_angles))).T
        lidar_rate = 10  # Hz
        time_step = 1 / (lidar_rate * len(points))

        corrected_points = []
        for i, point in enumerate(points):
            time = i * time_step
            angle = self.angular_command * time
            rotation_matrix = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle),  np.cos(angle)]
            ])
            translation = np.array([self.linear_command * time, 0.0])
            corrected_point = rotation_matrix @ point - translation
            corrected_points.append(corrected_point)

        flat = np.array(corrected_points).flatten()
        line = ' '.join(f"{val:.4f}" for val in flat)
        self.laserscan_file.write(line + '\n')

    def ground_truth_callback(self, message):
        '''store latest TF transform as ground truth pose

        args:
            message: TFMessage containing robot pose
        '''
        for tf in message.transforms:
            if tf.child_frame_id == self.robot_frame:
                self.ground_truth_pos = Point(
                    x=tf.transform.translation.x,
                    y=tf.transform.translation.y,
                    z=0.0
                )
                r = tf.transform.rotation
                self.ground_truth_orientation = 2 * math.atan2(r.z, r.w)

    def observation_callback(self, observation_array):
        '''log landmark observations if allowed by observation_flag

        args:
            observation_array: LandmarkArray message
        '''
        if self.current_odometry is None or not self.observation_flag:
            return

        info_val = 1.0
        for obs in observation_array.observations:
            landmark_id = obs.id
            x = obs.position.x
            y = obs.position.y
            self.log_file.write(f"EDGE_SE2_XY {self.pose_id-1} {landmark_id} {x} {y} {info_val} 0.0 {info_val}\n")

        self.observation_flag = False

    def command_callback(self, command):
        '''store latest control input for logging

        args:
            command: Twist message
        '''
        self.command_flag = True
        self.linear_command = command.linear.x
        self.angular_command = command.angular.z

    def lidar_callback(self, scan_msg):
        '''cache latest scan for deskewing on odometry update

        args:
            scan_msg: LaserScan message
        '''
        self.latest_scan = scan_msg


def main(args=None):
    '''entry point for ROS2 node'''
    rclpy.init(args=args)
    log_node = PoseGraphLogger()
    rclpy.spin(log_node)
    log_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()