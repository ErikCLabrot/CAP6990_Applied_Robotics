"""ICP Class for CAP6990 Applied Robotics Assignment_6

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/27/2025

This module defines an implementation of the Iterative Closest Point algorithm. The class takes in 
a ROS LaserScan message, converts it to cartesian coordinates, and performs the ICP algorithm. 
"""

import numpy as np
from sensor_msgs.msg import LaserScan

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
    '''
    def __init__(self, num_iterations=100, error_tolerance=1e-9, max_correspondence_dist=0.15):
        '''
        Function to initalize ICP parameters

        args:
            num_iterations: Number of iterations to run the ICP algorithm
            error_tolerance: Value by which convergence is considered to be reached
            max_correspondence_dist: Maximum neighbor distance for outliers
        '''
        self.num_iterations = num_iterations
        self.error_tolerance = error_tolerance
        self.max_correspondence_dist = max_correspondence_dist

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

    def guess_initial_transform(self, source_points, target_points):
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

        temp_term = np.dot(source_centroid, target_centroid) / (np.linalg.norm(source_centroid) * np.linalg.norm(target_centroid))
        angle = np.arccos(np.clip(temp_term,-1.0,1.0))

        rotation_direction = np.sign(np.cross(source_centroid, target_centroid))
        angle *= rotation_direction
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

    def compute_registration(self, source_scan, target_scan):
        '''
        Function that performs the main ICP algorithm. This function will filter and convert 
        ros LaserScan messages into cartesian coordinates, then perform the ICP algorithm to compute
        a transform between the two LaserScans.

        args:
            source_scan: The source scan for the ICP algorithm (in terms of scans, n+1)
            target_scan: The target scan for the ICP algorithm (in terms of scans, n)

        returns:
            final_transform: The rigid body transform computed after n iterations of the ICP algorithm, or until a reasonable 
                             convergence is reached
        '''
        initial_transform = np.eye(3)

        source_ranges, source_angles = self.filter_laserscan(scan = source_scan)
        target_ranges, target_angles = self.filter_laserscan(scan = target_scan)

        if source_ranges.size == 0 or target_ranges.size == 0:
            return initial_transform

        source_points = self.laserscan_to_coordinates(ranges = source_ranges, angles = source_angles)
        target_points = self.laserscan_to_coordinates(ranges = target_ranges, angles = target_angles)

        source_points, initial_transform = self.guess_initial_transform(source_points = source_points, target_points = target_points)
        source_points = self.filter_outliers(source_points = source_points, target_points = target_points)

        if source_points.size == 0 or target_points.size == 0:
            return initial_transform

        prev_error = 0

        final_transform = initial_transform

        for i in range(self.num_iterations):
            target_correspondences, error = self.find_correspondence(source_points = source_points, target_points = target_points)

            source_centroid = self.compute_centroid(points = source_points)
            target_centroid = self.compute_centroid(points = target_correspondences)
            covariance = self.build_covariance(source_points = source_points, target_points = target_correspondences, source_centroid=source_centroid, target_centroid=target_centroid)
            transform = self.calculate_transform(covariance = covariance, source_centroid = source_centroid, target_centroid = target_centroid)
           
            final_transform = np.dot(final_transform, transform)
            source_points = self.homogenize_coordinates(points = source_points)
            source_homogeneous = np.dot(transform, source_points)
            source_points = self.unhomogenize_coordinates(points = source_points)

            mean_error = np.mean(error)
            if np.abs(prev_error - mean_error) < self.error_tolerance:
                break
            prev_error = mean_error

        return final_transform