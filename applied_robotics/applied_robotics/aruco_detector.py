"""aruco_detector for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines a visual fiduciary marker detector. This module detects markers from a 
camera stream vis ROS, calculates the pose for the marker, then provides the transform for
the marker as if it's attached to a 1m cube, publishing a list of all detected cubes over ROS.
"""
import rclpy
import math
import cv2 as cv
import numpy as np

from cv2 import aruco
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
from applied_robotics_utilities.msg import LandmarkArray, LandmarkObservation

class ArucoDetector(Node):
    """
    Node for detecting Aruco markers and publishing their positions in the robot's coordinate space

    Methods:
        image_cb: Processes image to detect Aruco markers
        caminfo_cb: Retrieves camera intrinsic parameters
        pose_cb: Updates robot position and orientation from TF
        _yaw_from_quat: Retrieves yaw from orientation quaternion
        find_tf: Transforms marker coordinates to robot's coordinate space
        publish_obs: Publishes all detected cubes as a PoseArray
        get_ids: Returns detected cubes' IDs

    Attributes:
        aruco_dict: Aruco dictionary for marker detection
        aruco_params: Aruco detector parameters
        marker_size: Physical size of markers in meters
        bridge: CV Bridge for converting ROS Image messages to OpenCV images
        detected_cubes: Dictionary of detected cube IDs
        detected_cube_pos: Dictionary of cube positions in C-space
        all_detected_cubes: List of all detected cubes
        cam_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients of the camera
        detected_marker: Flag for marker detection
        info_ready: Flag for camera info readiness
        pose_ready: Flag for robot pose readiness
        robot_pos: Robot's position as a Point
        robot_yaw: Robot's orientation in radians
        frame: TF frame name of the robot
        image_sub: Subscription to the image topic
        info_sub: Subscription to the camera info topic
        pose_sub: Subscription to the robot pose topic
        obs_pub: Publisher for detected obstacles as PoseArray
    """

    def __init__(self, aruco_dict=cv.aruco.DICT_6X6_250):
        super().__init__("aruco_node")

        self.image_topic = self.declare_parameter('camera_topic', '/camera/image').value
        self.image_info_topic = self.declare_parameter('camera_info_topic', '/camera/info').value
        self.odometry_topic = self.declare_parameter('odometry_topic', '/robot/odometry').value
        self.ekf_state_topic = self.declare_parameter('ekf_state_topic', '/filtered_odometry').value
        self.landmark_topic = self.declare_parameter('landmark_observation_topic', '/landmarks').value

        self.start_position = self.declare_parameter('world_start_position', [0.0,0.0,0.0]).value
        self.start_orientation = self.declare_parameter('world_start_orientation', 0.0).value

        self.marker_size = self.declare_parameter('aruco_marker_size', 0.4).value

        # aruco detector
        self.aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict)
        self.aruco_params = cv.aruco.DetectorParameters.create()
        self.bridge = CvBridge()

        self.detected_cubes = {}
        self.detected_cube_pos = {}
        self.all_detected_cubes = []  # Store all detected cubes

        self.cam_matrix = None
        self.dist_coeffs = None

        self.detected_marker = False
        self.info_ready = False
        self.pose_ready = False

        self.robot_pos = Point(x=self.start_position[0], y=self.start_position[1], z=0.0)
        self.robot_yaw = self.start_orientation  # rads

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.info_sub = self.create_subscription(CameraInfo, self.image_info_topic, self.caminfo_callback, 1)
        self.odometry_sub = self.create_subscription(Odometry, self.ekf_state_topic, self.odometry_callback, 1)
        self.obstacle_pub = self.create_publisher(LandmarkArray, self.landmark_topic, 1)

    def image_callback(self, img_msg):
        """
        Processes image to detect Aruco markers

        Args:
            img_msg (Image): Image message
        """
        if self.info_ready is not True or self.pose_ready is not True:
            return

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        cubes = []

        if ids is not None:
            for i, marker_id in enumerate(ids):
                cube_id = marker_id[0] // 6
                cube_pos = self.find_tf(corners=corners[i], cube_id=cube_id)

                if cube_pos is not None:
                    cubes.append([cube_id, cube_pos])

            if cubes:
                self.publish_obstacles(cubes)

    def caminfo_callback(self, info_msg):
        """
        Retrieves camera intrinsic parameters

        Args:
            info_msg (CameraInfo): CameraInfo message
        """

        self.camera_matrix = np.array(info_msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(info_msg.d)
        self.info_ready = True

    def odometry_callback(self, odometry):
        self.pose_ready = True

        self.robot_pos = odometry.pose.pose.position
        orientation = odometry.pose.pose.orientation
        self.robot_yaw = self._yaw_from_quat(quat=orientation)

    def _yaw_from_quat(self, quat):
        '''
        Retrieve yaw from quat (rot about z axis)

        Args:
            quat (Quaternion): Orientation quat

        Returns:
            float: Yaw angle
        '''
        a = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        b = 1.0 - 2.0 * (quat.y**2 + quat.z**2)
        yaw = math.atan2(a, b)

        return yaw

    def find_tf(self, corners, cube_id):
        """
        Transforms marker coordinates to robot's coordinate space

        Args:
            corners (list): Detected marker corners
            cube_id (int): ID of the detected cube
        """

        t_cx = 1.0  # t_vec between rob origin and cam
        t_cy = 0.0

        cube_size = 1.0

        marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                  [self.marker_size / 2, self.marker_size / 2, 0],
                                  [self.marker_size / 2, -self.marker_size / 2, 0],
                                  [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)

        success, rotation_vec, translation_vec = cv.solvePnP(marker_points, corners, self.camera_matrix, self.dist_coeffs, False, cv.SOLVEPNP_IPPE_SQUARE)
        if success:
            rot_camera_to_marker, _ = cv.Rodrigues(rotation_vec)

            marker_to_cube_center = np.array([[0.0], [0.0], [-cube_size / 2]])
            cube_pos_in_camera_frame = translation_vec + rot_camera_to_marker @ marker_to_cube_center

            t_cx = 1.0  # in meters
            t_cy = 0.0

            # Cube position relative to robot frame:
            cam_x = cube_pos_in_camera_frame[0][0]
            cam_y = cube_pos_in_camera_frame[1][0]
            cam_z = cube_pos_in_camera_frame[2][0]

            # In robot frame:
            cube_rel_x = t_cx + cam_z  # forward
            cube_rel_y = t_cy - cam_x  # left (note: -cam_x since cam x = right)

            relative_tf = Point(x=cube_rel_x, y=cube_rel_y, z=0.0)

            return relative_tf

        return None

    def publish_obstacles(self, cubes):
        '''
        Publish all detected cubes as a PoseArray
        '''
        cube_array = LandmarkArray()

        for cube in cubes: 
            self.get_logger().info(f'{cube}')
            observation = LandmarkObservation()
            observation.id = int(cube[0])
            observation.position = cube[1]
            cube_array.observations.append(observation)

        self.obstacle_pub.publish(cube_array)

    def get_ids(self):
        """
        Returns detected cubes' IDs

        Args: None

        Returns:
            dict: Dictionary of detected cube IDs
        """

        return self.detected_cubes


def main(args=None):
    rclpy.init(args=args)
    image_topic = '/camera/image'
    info_topic = '/camera/info'
    pose_topic = '/robot_pose'
    marker_size = 0.4 #m
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
