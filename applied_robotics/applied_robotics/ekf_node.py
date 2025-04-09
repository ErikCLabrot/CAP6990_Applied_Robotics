"""EKF_Node for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 4/8/2025

This module defines a ROS node that wraps an implementation of an extended kalman filter. 
The node reads in lidar odometry from the icp node, as well as imu from a simulated robot in
gazebo, and handles predicting based on imu input, then correcting and updating based on 
lidar odometry.
"""


import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu
from applied_robotics.extended_kalman_filter import ekf


class EKF_Node(Node):
    '''
    ROS2 Node that wraps an extended kalman filter implementation.
    The node handles receiving lidar odometry and IMU data, and manages the EKF
    implementation accordingly. Whenever IMU data is received, the EKF predicts its 
    current state based off this information. Whenever Lidar Odometry data is received,
    the EKF updates its states using the velocity information in this odometry.

    Params:
        lidar_odometry_topic: Topic for the lidar odometry subscriber
        ekf_state_topic: Topic for the EKF state publisher
        imu_sensor_topic: Topic for the imu sensor subscriber
        imu_sensor_update_rate: imu sensor update rate in Hz
        start_position: Starting position for the robot in world coordinates
        start_orientation: Starting orientation for the robot in world coordinates
        lidar_odometry_sub: ROS2 Subscriber object for lidar odometry
        imu_sensor_sub: ROS2 Subscriber object for imu sensor information
        ekf_state_pub: ROS2 Publisher object for ekf state output
        ekf_state_estimator: extended kalman filter class object
        state_vector: 5 element state vector in the following form [x, y, theta, v, w]
    '''
    def __init__(self):
        '''
        Initializes EKF node, and configures EKF with ros params.
        '''

        self.node_name = "ekf_node"
        super().__init__(self.node_name)

        #Init Params
        self.lidar_odometry_topic = self.declare_parameter('lidar_odometry_topic', '/lidar/odometry').value
        self.ekf_state_topic = self.declare_parameter('ekf_state_topic', '/filtered_odometry').value
        self.imu_sensor_topic = self.declare_parameter('imu_sensor_topic', '/robot/imu').value
        self.imu_sensor_update_rate = self.declare_parameter('imu_sensor_update_rate', 1000).value
        self.start_position = self.declare_parameter('world_start_position', [0.0, 0.0, 0.0]).value
        self.start_orientation = self.declare_parameter('world_start_orientation', 0.0).value

        #Init Subs
        self.lidar_odometry_sub = self.create_subscription(Odometry, self.lidar_odometry_topic, self.lidar_odometry_callback, 1)
        self.imu_sensor_sub = self.create_subscription(Imu, self.imu_sensor_topic, self.imu_sensor_callback, 1)

        #Init Pubs
        self.ekf_state_pub = self.create_publisher(Pose, self.ekf_state_topic, 1)

        #Init EKF class
        start_x = self.start_position[0]
        start_y = self.start_position[1]
        start_z = self.start_position[2]

        start_linear_acceleration = 0.0
        start_angular_velocity = 0.0

        self.ekf_state_estimator = ekf(imu_update_rate=self.imu_sensor_update_rate)

        initial_state = np.array([[start_x], [start_y], [self.start_orientation], [start_linear_acceleration], [start_angular_velocity]])
        self.ekf_state_estimator.set_state(initial_state)

        self.state_vector = np.zeros((5,1))
        self.filtered_odometry = Odometry()

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

    def lidar_odometry_callback(self, msg):
        '''
        Lidar Odometry callback, EKF Update step. Updates EKF with lidar odometry whenever triggered.

        args:
            msg: ROS Odometry message containing lidar odometry
        '''
        lidar_linear_velocity = msg.twist.twist.linear.x
        lidar_angular_velocity = msg.twist.twist.angular.z

        lidar_measurement_vector = [lidar_linear_velocity, lidar_angular_velocity]

        self.ekf_state_estimator.update(measurement_vector = lidar_measurement_vector)

        self.state_vector = self.ekf_state_estimator.get_state()
        self.get_logger().info(f'State post update: {self.state_vector}')
        self.publish_state()

    def imu_sensor_callback(self, msg):
        '''
        IMU sensor callback, EKF predict step. Predict based on latest IMU reading whenever triggered.

        args:
            msg: IMU data from ROS
        '''
        imu_angular_velocity = msg.angular_velocity.z
        imu_linear_acceleration = msg.linear_acceleration.x
        
        self.ekf_state_estimator.predict(imu_angular_rate = imu_angular_velocity, imu_linear_acceleration = imu_linear_acceleration)
        
        self.state_vector = self.ekf_state_estimator.get_state()
        self.get_logger().info(f'State post predict: {self.state_vector}')

        self.publish_state()

    def publish_state(self):
        '''
        Publish relevant information from current state as a ROS Pose message (x, y, theta)
        '''
        current_position = Point()

        current_position.x = self.state_vector[0,0]
        current_position.y = self.state_vector[1,0]
        current_position.z = 0.0

        current_yaw = self.state_vector[2,0]
        current_orientation = self.quaternion_from_yaw(yaw=current_yaw)

        current_pose = Pose(position = current_position, orientation = current_orientation)

        self.ekf_state_pub.publish(current_pose)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKF_Node()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()