"""KDTree for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 4/8/2025

This module defines the implementation for a KDTree added onto the lidar odometry node.
In theory, this speeds the nearest neighbor search to O(n log n) from O(n^2) per wiki. 
In practice, python prevents this from working expediently due to the reliance on recursive calls.
"""


import numpy as np
import sys

class KDNode():
    '''
    Node for the KD tree. Contains its point and its axis, as well as two children nodes
    '''
    def __init__(self, point, axis, left=None, right=None):
        self.point = point
        self.axis = axis
        self.left = left
        self.right = right

class KDTree():
    '''
    Class for KDTree implementation. 
    
    Params:
        root: The root node of the tree
    '''
    def __init__(self, points):
        '''
        Initialize and build the tree, storing the root node.

        args:
            points: The point set to build the tree off of
        '''
        self.root = self.build_tree(points)
        sys.setrecursionlimit(10000)

    def build_tree(self, points, depth=0):
        '''
        Function to recursively build the KD tree

        args:
            points: Pointset to build the tree from
            depth: The current depth of the tree, used to determine which axis to base slice on
        '''
        if points.shape[0] == 0:
            return None

        N, M = points.shape
        axis = depth % M

        #Split based on median of axis
        median = N // 2
        sorted_points = np.argpartition(points[:, axis], median)
        median_point = points[sorted_points[median]]

        left_points = points[sorted_points[:median]]
        right_points = points[sorted_points[median+1:]]

        #Recursively Build Tree
        return KDNode(point = median_point,
                      axis = axis,
                      left = self.build_tree(left_points, depth + 1),
                      right = self.build_tree(right_points, depth + 1)
                      )

    def nearest_neighbor(self, query_point):
        '''
        Entry point for recusive tree search
        '''
        return self.tree_recursive_search(query_point=query_point, node=self.root)

    def tree_recursive_search(self, query_point, node=None, best_fit=None, best_distance=float('inf')):
        '''
        Recursively search tree for nearest neighbor of a given point based on euclidean distance

        args:
            query_point: The point to find nearest neighbor for
            node: The current node in the tree
            best_fit: The current best fit point for nearest neighbor
            best_distance: The current best fit distance from the point to its neighbor
        '''
        if node is None:
            return best_fit, best_distance

        point = node.point
        axis = node.axis
        distance = np.linalg.norm(query_point - point)
        if distance < best_distance:
            best_fit = point
            best_distance = distance

        direction = query_point[axis] - point[axis]

        if direction < 0:
            close = node.left
            away = node.right
        else:
            close = node.right
            away = node.left

        # Search close branch
        best_fit, best_distance = self.tree_recursive_search(query_point, close, best_fit, best_distance)

        # Maybe search away branch
        if abs(direction) < best_distance:
            best_fit, best_distance = self.tree_recursive_search(query_point, away, best_fit, best_distance)

        best_distance = best_distance ** 0.5

        return best_fit, best_distance