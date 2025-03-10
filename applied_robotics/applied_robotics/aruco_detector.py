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
        markerSize: Physical size of markers in meters
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

    def __init__(self, img_topic, info_topic, pose_topic, markerSize=0.4, aruco_dict=cv.aruco.DICT_6X6_250):
        super().__init__("aruco_node")
        # aruco detector
        self.aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict)
        self.aruco_params = cv.aruco.DetectorParameters.create()
        self.markerSize = markerSize  # meters
        self.bridge = CvBridge()

        self.detected_cubes = {}
        self.detected_cube_pos = {}
        self.all_detected_cubes = []  # Store all detected cubes

        self.cam_matrix = None
        self.dist_coeffs = None

        self.detected_marker = False
        self.info_ready = False
        self.pose_ready = False

        self.robot_pos = Point(x=0.0, y=0.0, z=0.0)
        self.robot_yaw = 0.0  # rads
        self.frame = 'vehicle_blue'

        self.image_sub = self.create_subscription(Image, img_topic, self.image_cb, 1)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.caminfo_cb, 1)
        self.pose_sub = self.create_subscription(TFMessage, pose_topic, self.pose_cb, 1)
        self.obs_pub = self.create_publisher(PoseArray, '/obstacles', 1)

        # Timer to periodically publish all detected cubes as an array
        self.create_timer(0.1, self.publish_obs)

    def image_cb(self, img_msg):
        """
        Processes image to detect Aruco markers

        Args:
            img_msg (Image): Image message
        """
        if self.info_ready is not True:
            return

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
        crnrs, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for i, marker_id in enumerate(ids):
                cube_id = marker_id[0] // 6
                if cube_id not in self.detected_cubes and self.pose_ready and self.info_ready:
                    self.detected_cubes[cube_id] = True
                    self.find_tf(crnrs[i], cube_id)

    def caminfo_cb(self, info_msg):
        """
        Retrieves camera intrinsic parameters

        Args:
            info_msg (CameraInfo): CameraInfo message
        """

        self.camera_matrix = np.array(info_msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(info_msg.d)
        self.info_ready = True

    def pose_cb(self, pose_msg):
        for tf in pose_msg.transforms:
            if tf.child_frame_id == self.frame:
                self.robot_pos = Point(
                    x=tf.transform.translation.x,
                    y=tf.transform.translation.y,
                    z=tf.transform.translation.z
                )
                r = tf.transform.rotation
                self.robot_yaw = self._yaw_from_quat(r)
                self.pose_ready = True

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

        marker_points = np.array([[-self.markerSize / 2, self.markerSize / 2, 0],
                                  [self.markerSize / 2, self.markerSize / 2, 0],
                                  [self.markerSize / 2, -self.markerSize / 2, 0],
                                  [-self.markerSize / 2, -self.markerSize / 2, 0]], dtype=np.float32)

        s, r, t = cv.solvePnP(marker_points, corners, self.camera_matrix, self.dist_coeffs, False, cv.SOLVEPNP_IPPE_SQUARE)
        if s:
            r_mat, _ = cv.Rodrigues(r)
            v_cube_center = np.array([[0.0], [0.0], [-cube_size / 2]])  # vec aruco to cube center

            v_cam_cc = t + (r_mat @ v_cube_center)  # tf cube to cam

            rob_x = v_cam_cc[2][0]  # cam_z = rob_x
            rob_y = -v_cam_cc[0][0]  # -cam_x = rob_y

            c_x = (t_cx * np.cos(self.robot_yaw) - t_cy * np.sin(self.robot_yaw))  # Cam to cspace tf
            c_y = (t_cx * np.sin(self.robot_yaw) + t_cy * np.cos(self.robot_yaw))

            # cube coords in cspace
            cube_cspace_x = self.robot_pos.x + c_x + (rob_x * np.cos(self.robot_yaw) - rob_y * np.sin(self.robot_yaw))
            cube_cspace_y = self.robot_pos.y + c_y + (rob_x * np.sin(self.robot_yaw) + rob_y * np.cos(self.robot_yaw))

            # pub new cube
            point = Point()
            point.x = cube_cspace_x
            point.y = cube_cspace_y
            point.z = 0.0
            self.get_logger().info(f'Detected Cube! ID = {cube_id}: Cspace Coords: {cube_cspace_x}, {cube_cspace_y}')

            self.detected_cube_pos[cube_id] = point

    def publish_obs(self):
        '''
        Publish all detected cubes as a PoseArray
        '''
        pose_array = PoseArray()

        for cube_pos in self.detected_cube_pos:
            pose = Pose()
            pose.position = self.detected_cube_pos[cube_pos]
            pose.orientation.w = 1.0  
            pose_array.poses.append(pose)

        self.obs_pub.publish(pose_array)

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
    node = ArucoDetector(img_topic=image_topic,info_topic=info_topic,pose_topic=pose_topic, markerSize=marker_size)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
