"""ICP Class for CAP6990 Applied Robotics Assignment_6

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/27/2025

This module defines an implementation of the Iterative Closest Point algorithm. The class takes in 
a ROS LaserScan message, converts it to cartesian coordinates, and performs the ICP algorithm. 
"""

import numpy as np
from sensor_msgs.msg import LaserScan
from applied_robotics.kd_tree import KDTree
from scipy.spatial import cKDTree


class ICP():
    '''
    Class that implements the ICP algorithm. Correspondences between two sets of points are found, the covariance of this
    subset is determined, and the rigid body transform between both is computed until a number of iterations is reached or 
    a satisfactory convergence tolerance is passed. Additionally, this ICP algorithm makes an initial guess at the transform
    between the sets, and filters points that correspond poorly.

    Attributes:
        num_iterations: The maximum number of iterations to run the ICP algorithm
        error_tolerance: The tolerance for error by which convergence is reached
        max_corresponence_dist: The maximum allowed distance for a point to have a neighbor before being considered an outlier
        voxel_size: The current size for the adaptive voxel downsampling system
        min_voxel: The minimum voxel size
        target_point_count: the number of points to attempt to scale a cloud to using voxels
    '''
    def __init__(self,node, update_rate, num_iterations=100, error_tolerance=1e-2, max_correspondence_dist=1.0):
        '''
        Function to initalize ICP parameters

        args:
            num_iterations: Number of iterations to run the ICP algorithm
            error_tolerance: Value by which convergence is considered to be reached
            max_correspondence_dist: Maximum neighbor distance for outliers
        '''
        self.update_rate = update_rate
        self.num_iterations = num_iterations
        self.error_tolerance = error_tolerance
        self.max_correspondence_dist = max_correspondence_dist
        self.voxel_size = 0.01
        self.min_voxel = 0.01
        self.target_point_count = 250

    def filter_laserscan(self, scan):
        '''
        Function to filter erroneous values from laserscan, extracts ranges and angles from ros datatype

        args:
            scan: laserscan to be filtered
            
        returns:
            filtered ranges and angles 
        '''
        scan_ranges = np.array(scan.ranges)
        scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(scan_ranges))

        valid_points = np.isfinite(scan_ranges)

        valid_ranges = scan_ranges[valid_points]
        valid_angles = scan_angles[valid_points]

        return valid_ranges, valid_angles

    def laserscan_to_coordinates(self, ranges, angles):
        '''
        Function to convert polar LaserScan coordinates to cartesian coordinates

        args:
            ranges: Range from LaserScan
            angles: Angle for each range reading
        returns:
            points: vertically stacked x,y pairs as np array
        '''
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points = np.vstack((x,y)).T

        return points

    def undistort_laser_scan(self, scan, linear_velocity, angular_velocity, lidar_rate=10):
        '''
        Function to undistort and convert a laser scan. Converts a laserscan to euclidean points, and
        deskews the points based on the linear and angular velocity associated with the scan.

        args:
            scan: ROS LaserScan to be converted
            linear_velocity: The current estimated linear velocity of the laser scanner
            angular_velocity: The current estimated angular velocity of the laser scanner
            lidar_rate: The rate at which the lidar scanner produces complete scans
        returns:
            corrected_points: A set of points converted from a LaserScan and deskewed based on estimated velocity
        '''

        scan_ranges = np.array(scan.ranges)
        scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(scan_ranges))

        scan_array = np.vstack((scan_ranges, scan_angles)).T

        time_step = 1/(lidar_rate * len(scan_ranges))

        corrected_points = []
        time = 0.0

        for scan in scan_array:
            if np.isfinite(scan[0]):
                #Estimate Rotation due to Angular Velocity
                angle = angular_velocity * time
                rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                            [np.sin(angle), np.cos(angle)]])
                
                #Estimate Translation due to Linear Velocity
                translation = linear_velocity * time

                point_x = scan[0] * np.cos(scan[1])
                point_y = scan[0] * np.sin(scan[1])

                point = np.array([point_x, point_y])

                #Apply Correction
                corrected_point = rotation_matrix @ point - translation
                corrected_points.append(corrected_point)

            time += time_step

        return np.array(corrected_points)

    def homogenize_coordinates(self, points):
        '''
        Function to homogenize a set of cartesian coordinates

        args:
            Points: points to be homogenized
        returns:
            points_homogeneous: Array of homogeneous points
        '''
        dimensions = points.shape[1]
        points_homogeneous = np.ones((dimensions + 1, points.shape[0]))
        points_homogeneous[:dimensions, :] = np.copy(points.T)
        return points_homogeneous

    def unhomogenize_coordinates(self, points):
        '''
        Function to unhomogenize a set of homogeneous coordinates

        args:
            points: points to be unhomogenized
        returns:
            points_cartesian: vertically stacked x,y cartesian coordinates
        '''
        w = points[-1, :]
        points_cartesian = points[:-1, :] / w[np.newaxis, :]
        return points_cartesian.T

    def filter_outliers(self, source_points, target_points):
        '''
        Function to filter outliers based on a maximum allowable euclidean distance. This is to help stabilize
        the ICP algorithm against sudden changes to provided scans, and large differences between source and 
        target scans

        args:
            source_points: source points to be filtered 
            target_points: target points to be filtered against
        returns:
            array of filtered points
        '''
        valid_points = []
        for p in source_points:
            distances = np.linalg.norm(target_points - p, axis = 1)
            if np.any(distances <= self.max_correspondence_dist):
                valid_points.append(p)

        return np.array(valid_points)

    def calculate_voxel_size(self, points, target_point_count):
        '''
        Function to calculate the voxel size based on point cloud size.

        args:
            points: The pointcloud to scale based off of
            target_point_count: The number of points desired in the downsampled cloud
        '''
        voxel_scale = np.sqrt(points.size / target_point_count)
        self.voxel_size = self.min_voxel * voxel_scale


    def downsample_pointcloud(self, points):
        '''
        Function to downsample pointcloud to ensure computational consistency.

        args:
            points: Pointcloud to be downsampled
        returns:
            downsampled_points: Pointcloud that's been downsampled to a desired size
        '''
        voxelized_points = np.floor(points/self.voxel_size)

        _, unique_indices = np.unique(voxelized_points, axis=0, return_index=True)

        downsampled_points = points[unique_indices]

        return downsampled_points


    def predictive_filter_outliers(self, source_points, target_points, linear_velocity, angular_velocity):
        '''
        Function to filter outliers based on a maximum allowable euclidean distance. This function compares a
        predicted point cloud by projecting target_points into the future, and comparing this projection to 
        source points. Points that don't align very well with this projection are rejected.

        args:
            source_points: The source points from the current scan
            target_points: The target points from the last scan to transform from
            linear_velocity: The linear velocity corresponding to the target scan
            angular_velocity: The angular velocity corresponding to the target scan
        returns:
            filtered_points: The set of source points with outliers removed
        '''

        #Create predicted set of points
        dt = 1/self.update_rate
        angle = angular_velocity * dt
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]])
        translation = linear_velocity * dt

        predicted_source = (rotation_matrix @ target_points.T).T + translation

        #Compare and filter out points between prediction and source 
        distances = []
        for p in source_points:
            dists = np.linalg.norm(predicted_source - p, axis = 1)
            nearest_neighbor = np.min(dists)
            distances.append(nearest_neighbor)
        distances = np.array(distances)

        #Return filtered source points
        mean = np.mean(distances)
        std_dev = np.std(distances)

        filter_mask = distances < self.max_correspondence_dist
        filtered_points = source_points[filter_mask]

        return filtered_points

    def find_correspondence(self, source_points, target_points):
        '''
        Function to find the nearest neighbor for each source point inside of target points, based on 
        euclidean distance.

        args:
            source_points: the source points to find the neighbor of 
            target_points: the target points to find the neighbors in
        return:
            corresponding_points: array of corresponding points referenced to source points in target points
            error: an list containing the euclidean distance between each source point and its closest point in target points
        '''
        correspondences= []
        error = []

        for p in source_points:
            distances = np.linalg.norm(target_points - p, axis = 1)
            nearest_point = np.argmin(distances)
            correspondences.append(nearest_point)
            error.append(np.amin(distances))

        corresponding_points = target_points[correspondences]

        return corresponding_points, error

    def find_correspondence_optimized(self, source_points, target_tree):
        '''
        Function that finds the nearest neighbor for each source point using a KD Tree. 
        In theory, is is more optimal than the O(n^2) euclidean distance search. In pracitce,
        this actually runs slow in its current form. 

        args:
            source_points: Source points to find the neighbor of
            target_tree: a KD-Tree of points to find the neighbor in
        returns:
            correspondences: array of corresponding points referenced to source points in target_points
            error: a list containing the euclidean distance between each source point and its closest point in target points
        ''' 
        N, M = source_points.shape

        correspondences = np.empty((N,M))
        distances = np.empty((N,M))

        for i in range(N):
            nearest_point, dist = target_tree.nearest_neighbor(source_points[i])
            correspondences[i] = nearest_point
            distances[i] = dist

        return correspondences, distances

    def find_correspondence_scipy(self, source_points, target_points, target_tree):
        '''
        Function that uses the scipy KDTree implementation. This runs faster than brute force and personal kdtree implementation.

        args:
            source_points: The source points to correspond from
            target_points: The target points to correspond to
            target_tree: The KDTree that contains the target_points, created at ICP start
        returns: 
            corresponding_points: THe points in target points that correspond to a given point in source points
            distances: The distances between the corresponding points
        '''
        distances, indices = target_tree.query(source_points, k=1)
        corresponding_points = target_points[indices]
        return corresponding_points, distances

    def compute_centroid(self, points):
        '''
        Function to compute the centroid of a given set of points
        args:

            points: points to find the centroid of

        returns:
            centroid; the centroid of the points
        '''
        centroid = np.mean(points, axis = 0)
        return centroid

    def build_covariance(self, source_points, target_points, source_centroid, target_centroid):
        '''
        Function to build a covariance matrix between a set of points

        args:
            source_points: the set of points to reference the covariance from
            target_points: the set of points to reference the covariance towards
            source_centroid: centroid of the source points
            target_centroid: centroid of the target points

        returns:
            covariance: covariance matrix encoding the variance between the two sets
        '''
        source_prime = source_points - source_centroid
        target_prime = target_points - target_centroid

        covariance = np.dot(source_prime.T, target_prime)

        return covariance

    def calculate_transform(self, covariance, source_centroid, target_centroid):
        '''
        Function to calculate the rigid body transform between two sets of points using SVD to 
        decompose a covariance matrix

        args:
            covariance: the covariance matrix to decompose
            source_centroid: the source centroid that is being rotated and translated
            target_centroid: the target centroid being rotated and translated towards

        returns:
            transform: homogeneous transform between the two sets
        '''
        U, S, Vt = np.linalg.svd(covariance)
        rotation = np.dot(Vt, U)

        #Reflection case
        if np.linalg.det(rotation) < 0:
            Vt[1, :] *= -1
            rotation = np.dot(Vt, U)

        translation = target_centroid.T - np.dot(rotation, source_centroid.T)

        transform = np.eye(3)
        transform[:2, :2] = rotation
        transform[:2, 2] = translation

        return transform

    def guess_initial_transform(self, source_points, target_points, source_heading, target_heading):
        '''
        Function that takes two sets of points, and makes a naieve guess at aligning them. This is accomplished
        by treating the points as if they have a common origin, then rotating and translating the source points ontop of the 
        target point. This helps the ICP algorithm converge to a good fit transform by mitigating problematic geometry can cause
        (e.g. straight lines causing local minima, and improperly calculating rotations as translations)

        args:
            source_points: source points to be transformed
            target_points: reference points to be transformed towards

        returns:
            guessed_alignment: set of points that have been transformed using the guessed transform
            transform: the transform with which the source points are transformed with
        '''

        _, start_error = self.find_correspondence(source_points = source_points, target_points = target_points)

        source_centroid = self.compute_centroid(points = source_points)
        target_centroid = self.compute_centroid(points = target_points)

        if source_heading is None or target_heading is None:
            temp_term = np.dot(source_centroid, target_centroid) / (np.linalg.norm(source_centroid) * np.linalg.norm(target_centroid))
            angle = np.arccos(np.clip(temp_term,-1.0,1.0))

            rotation_direction = np.sign(np.cross(source_centroid, target_centroid))
            angle *= rotation_direction
        else:
            angle = target_heading - source_heading

        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])

        rotated_source = (rotation_matrix @ source_points.T).T

        rotated_centroid = self.compute_centroid(points = rotated_source)
        translation = target_centroid - rotated_centroid

        guessed_alignment = rotated_source + translation

        _, guessed_error = self.find_correspondence(source_points = guessed_alignment, target_points = target_points)

        #If guess is worse than what we started with, discard it
        if np.mean(guessed_error) > np.mean(start_error):
            return source_points, np.eye(3)

        transform = np.eye(3)
        transform[:2,:2] = rotation_matrix
        transform[:2, 2] = translation

        return guessed_alignment, transform

    def compute_registration(self, source_scan, target_scan, linear_velocity=None, angular_velocity=None, last_velocity = None, last_angular = None, source_orientation=None, target_orientation=None):
        '''
        Function that performs the main ICP algorithm. This function will filter and convert 
        ros LaserScan messages into cartesian coordinates, then perform the ICP algorithm to compute
        a transform between the two LaserScans.

        args:
            source_scan: The source scan for the ICP algorithm (in terms of scans, n)
            target_scan: The target scan for the ICP algorithm (in terms of scans, n-1)
            linear_velocity: The n-1 to n linear velocity
            angular_velocity; The n-1 to n angular velocity
            last_velocity: the n-2 to n-1 linear velocity
            last_angular: tne n-2 to n-1 angular velocity
            source_orientation: The n orientation for the origin of the pointcloud
            target_orientation: the n-1 orientation for the origin of the pointcloud

        returns:
            final_transform: The rigid body transform computed after n iterations of the ICP algorithm, or until a reasonable 
                             convergence is reached
        '''
        initial_transform = np.eye(3)

        source_points = self.undistort_laser_scan(scan = source_scan, linear_velocity = linear_velocity, angular_velocity = angular_velocity)
        target_points = self.undistort_laser_scan(scan = target_scan, linear_velocity = last_velocity, angular_velocity = last_angular)

        self.calculate_voxel_size(source_points, self.target_point_count)
        source_points = self.downsample_pointcloud(source_points)
        target_points = self.downsample_pointcloud(target_points)

        source_points = self.predictive_filter_outliers(source_points=source_points, 
                                                        target_points=target_points, 
                                                        linear_velocity=linear_velocity, 
                                                        angular_velocity=angular_velocity)

        if source_points.size == 0:
            return initial_transform

        source_points, initial_transform = self.guess_initial_transform(source_points = source_points,
                                                                        target_points = target_points, 
                                                                        source_heading = source_orientation, 
                                                                        target_heading = target_orientation)
        
        if source_points.size == 0 or target_points.size == 0:
            return initial_transform

        target_tree = cKDTree(target_points)

        prev_error = 0

        final_transform = initial_transform

        for i in range(self.num_iterations):
            target_correspondences, error = self.find_correspondence_scipy(source_points = source_points, target_points = target_points, target_tree = target_tree)

            source_centroid = self.compute_centroid(points = source_points)
            target_centroid = self.compute_centroid(points = target_correspondences)
            covariance = self.build_covariance(source_points = source_points, 
                                               target_points = target_correspondences, 
                                               source_centroid = source_centroid, 
                                               target_centroid = target_centroid)

            transform = self.calculate_transform(covariance = covariance, source_centroid = source_centroid, target_centroid = target_centroid)
           
            final_transform = np.dot(final_transform, transform)
            source_points = self.homogenize_coordinates(points = source_points)
            source_homogeneous = np.dot(transform, source_points)
            source_points = self.unhomogenize_coordinates(points = source_homogeneous)

            mean_error = np.mean(error)

            #If we aren't meaningfully decreasing our error, we should break because we're, at best, stuck in a minima
            if np.abs(mean_error) < self.error_tolerance or np.abs(prev_error - mean_error) < 1e-6:
                break

            prev_error = mean_error
            
        return final_transform