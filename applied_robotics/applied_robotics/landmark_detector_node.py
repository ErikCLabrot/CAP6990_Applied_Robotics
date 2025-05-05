'''
Landmark Detector Node for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This module defines a laserscan based feature detection implementation based on the paper 
Extracting general-purpose features from LIDAR data by Yangming Li and Edwin B. Olson

This module is intended for use in the pose graph optimization pipeline, currently incomplete
Limitations arise due to indistinct cube corner features and odometry drift, reducing reliability
This implementation may be more effective in environments with richer, more unique features
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree
from applied_robotics_utilities.msg import LandmarkObservation, LandmarkArray
import numpy as np
import cv2

class LandmarkDetectorNode(Node):
    def __init__(self):
        '''initialize node, declare parameters, create subscriptions and publishers'''
        super().__init__('landmark_detector')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/lidar/scan').value
        self.lidar_odometry_topic = self.declare_parameter('lidar_odometry_topic', '/lidar/odometry').value
        self.lidar_sample_count = self.declare_parameter('lidar_sample_count', 7200).value
        self.lidar_sample_rate = self.declare_parameter('lidar_sample_rate', 10).value
        self.lidar_sample_sigma = self.declare_parameter('lidar_noise_sigma', 0.01).value
        self.ekf_state_topic = self.declare_parameter('ekf_state_topic', '/filtered_odometry').value
        self.landmark_topic = self.declare_parameter('landmark_observation_topic', '/landmarks').value
        self.start_position = self.declare_parameter('world_start_position', [0.0, 0.0, 0.0]).value
        self.start_orientation = self.declare_parameter('world_start_orientation', 0.0).value

        self.subscription = self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 1)
        self.odom_subscription = self.create_subscription(Odometry, self.ekf_state_topic, self.odometry_callback, 1)
        self.landmark_publisher = self.create_publisher(LandmarkArray, '/landmarks', 10)

        self.current_twist = None
        self.current_pose = None
        self.scan_time = None

        self.image_size = (1000, 1000)
        self.image_resolution = 0.02

        self.landmark_positions = []
        self.kd_tree = None

    def yaw_from_quaternion(self, quat):
        '''convert quaternion to 2D yaw angle

        args:
            quat: geometry_msgs quaternion

        returns:
            yaw: float
        '''
        a = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        b = 1.0 - 2.0 * (quat.y**2 + quat.z**2)
        return np.arctan2(a, b)

    def undistort_and_convert_laser_scan(self, scan, linear_velocity, angular_velocity, lidar_rate):
        '''deskew lidar scan using constant velocity model

        args:
            scan: LaserScan message
            linear_velocity: robot linear velocity
            angular_velocity: robot angular velocity
            lidar_rate: lidar scan frequency

        returns:
            corrected_points: Nx2 numpy array of deskewed points
        '''
        scan_ranges = np.array(scan.ranges)
        scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(scan_ranges))
        scan_array = np.vstack((scan_ranges, scan_angles)).T
        time_step = 1 / (lidar_rate * len(scan_ranges))

        corrected_points = []
        time = 0.0

        for scan in scan_array:
            if np.isfinite(scan[0]):
                angle = angular_velocity * time
                rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                            [np.sin(angle), np.cos(angle)]])
                translation = linear_velocity * time
                point_x = scan[0] * np.cos(scan[1])
                point_y = scan[0] * np.sin(scan[1])
                point = np.array([point_x, point_y])
                corrected_point = rotation_matrix @ point - translation
                corrected_points.append(corrected_point)

            time += time_step

        return np.array(corrected_points)

    def transform_to_global(self, points, current_pose):
        '''transform local scan points to global frame using current robot pose

        args:
            points: Nx2 list or array of local points
            current_pose: robot pose message

        returns:
            transformed_points: Nx2 list of global coordinates
        '''
        x = current_pose.position.x
        y = current_pose.position.y
        yaw = self.yaw_from_quaternion(quat=current_pose.orientation)

        transformed_points = []
        for p in points:
            x_tf = (np.cos(yaw) * p[0] - np.sin(yaw) * p[1]) + x
            y_tf = (np.sin(yaw) * p[0] + np.cos(yaw) * p[1]) + y
            transformed_points.append([x_tf, y_tf])

        return transformed_points

    def blur_pixel(self, point, lidar_sigma, lidar_resolution):
        '''generate gaussian blur kernel based on point location and lidar noise

        args:
            point: (x, y) coordinates
            lidar_sigma: sensor noise standard deviation
            lidar_resolution: number of rays per scan

        returns:
            gaussian_kernel: 2D numpy array
        '''
        distance = np.linalg.norm(point)
        d_theta = 2 * np.pi / lidar_resolution
        lidar_sigma_pixels = lidar_sigma / self.image_resolution
        distance_pixels = distance / self.image_resolution
        kernel_sigma = np.sqrt(lidar_sigma_pixels**2 + (distance_pixels * np.sin(d_theta))**2)
        kernel_sigma *= 5
        kernel_radius = int(np.ceil(3 * kernel_sigma))
        kernel_size = (2 * kernel_radius + 1)
        gaussian_kernel_1d = cv2.getGaussianKernel(kernel_size, kernel_sigma)
        gaussian_kernel_2d = gaussian_kernel_1d @ gaussian_kernel_1d.T
        gaussian_kernel = gaussian_kernel_2d / np.sum(gaussian_kernel_2d)
        return gaussian_kernel

    def rasterize_points(self, points, resolution=0.05, image_size=(512, 512)):
        '''rasterize point cloud into a grayscale image using gaussian blurs

        args:
            points: list of (x, y) lidar points
            resolution: pixel resolution in meters
            image_size: size of raster image in pixels

        returns:
            normalized_image: 2D numpy array image
        '''
        img = np.zeros(image_size, dtype=np.float64)
        origin = np.array(image_size) // 2

        for point in points:
            gaussian_patch = self.blur_pixel(point, self.lidar_sample_sigma, self.lidar_sample_count)
            half_patch = gaussian_patch.shape[0] // 2

            px = int(point[0] / resolution + origin[0])
            py = int(point[1] / resolution + origin[1])

            x_min = max(px - half_patch, 0)
            x_max = min(px + half_patch + 1, image_size[0])
            y_min = max(py - half_patch, 0)
            y_max = min(py + half_patch + 1, image_size[1])

            patch_x_min = half_patch - (px - x_min)
            patch_x_max = half_patch + (x_max - px)
            patch_y_min = half_patch - (py - y_min)
            patch_y_max = half_patch + (y_max - py)

            img[y_min:y_max, x_min:x_max] += gaussian_patch[patch_y_min:patch_y_max, patch_x_min:patch_x_max]

        normalized_image = img / np.max(img)
        return normalized_image.astype(np.float32)

    def derasterize_pixels(self, pixels, resolution, image_size):
        '''convert image pixel positions back to (x, y) world coordinates

        args:
            pixels: list of (x, y) image coordinates
            resolution: meters per pixel
            image_size: dimensions of raster image

        returns:
            landmarks: list of (x, y) world coordinates
        '''
        origin = np.array(image_size) // 2
        landmarks = []
        for px, py in pixels:
            x = (px - origin[0]) * resolution
            y = (py - origin[1]) * resolution
            landmarks.append((x, y))
        return landmarks

    def detect_features(self, img):
        '''detect corner-like features using Harris corner detector

        args:
            img: rasterized lidar scan image

        returns:
            corners: Nx2 array of pixel positions
        '''
        corners = cv2.goodFeaturesToTrack(img, maxCorners=50, qualityLevel=0.05, minDistance=10, blockSize=5, useHarrisDetector=True, k=0.04)
        if corners is not None:
            return corners.reshape(-1, 2)
        return []

    def identify_landmarks(self, features):
        '''match or register features as landmarks using KDTree search

        args:
            features: list of (x, y) global coordinates

        returns:
            ids: list of integer landmark ids
        '''
        if features:
            self.get_logger().info(f'We transformed the points')
            features = self.transform_to_global(points=features, current_pose=self.current_pose)
        else:
            return []

        if self.kd_tree is None:
            self.landmark_positions = features
            self.kd_tree = cKDTree(np.array(self.landmark_positions, dtype=np.float64))
            return list(range(len(self.landmark_positions)))

        x_covariance = self.current_pose_covariance[0]
        y_covariance = self.current_pose_covariance[6]
        position_sigma = np.sqrt(x_covariance + y_covariance)
        search_radius = 0.5

        ids = []
        new_landmarks = False
        for feature in features:
            identity = self.kd_tree.query_ball_point(feature, search_radius, return_sorted=True)
            if identity:
                ids.append(identity[0])
            else:
                self.landmark_positions.append(feature)
                ids.append(len(self.landmark_positions) - 1)
                new_landmarks = True

        if new_landmarks:
            self.kd_tree = cKDTree(np.array(self.landmark_positions, dtype=np.float64))

        return ids

    def find_landmarks(self, scan):
        '''process scan to detect and publish landmark observations

        args:
            scan: LaserScan message
        '''
        if self.current_twist is None:
            return

        points = self.undistort_and_convert_laser_scan(scan=scan, linear_velocity=self.current_twist.linear.x, angular_velocity=self.current_twist.angular.z, lidar_rate=self.lidar_sample_rate)
        img = self.rasterize_points(points=points, resolution=self.image_resolution, image_size=self.image_size)
        feature_pixels = self.detect_features(img=img)
        feature_coordinates = self.derasterize_pixels(pixels=feature_pixels, resolution=self.image_resolution, image_size=self.image_size)
        landmark_ids = self.identify_landmarks(features=feature_coordinates)
        self.publish_landmarks(observations=feature_coordinates, landmark_ids=landmark_ids)

    def publish_landmarks(self, observations, landmark_ids):
        '''publish landmark observations as LandmarkArray message

        args:
            observations: list of (x, y) coordinates
            landmark_ids: corresponding landmark ids
        '''
        msg = LandmarkArray()
        for observation, identity in zip(observations, landmark_ids):
            element = LandmarkObservation()
            element.id = identity
            element.position.x = observation[0]
            element.position.y = observation[1]
            element.position.z = 0.0
            msg.observations.append(element)
        self.landmark_publisher.publish(msg)

    def lidar_callback(self, scan):
        '''callback for incoming lidar scan

        args:
            scan: LaserScan message
        '''
        self.scan_time = scan.header.stamp
        self.find_landmarks(scan)

    def odometry_callback(self, odometry):
        '''callback for incoming filtered odometry

        args:
            odometry: Odometry message
        '''
        self.current_twist = odometry.twist.twist
        self.current_twist_covariance = odometry.twist.covariance
        self.current_pose = odometry.pose.pose
        self.current_pose_covariance = odometry.pose.covariance

def main(args=None):
    '''entry point for ROS2 node'''
    rclpy.init(args=args)
    landmark_detector_node = LandmarkDetectorNode()
    rclpy.spin(landmark_detector_node)
    landmark_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
