"""CES Service for CAP6990 Applied Robotics Assignment_5

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/10/2025

This module defines a ROS Service that wraps the CES Optimization algorithm. A path is provided
via request, and the optimized path is returned via response
"""


import rclpy 
import numpy as np
from rclpy.node import Node 
from geometry_msgs.msg import Point, Polygon
from shapely.geometry import Polygon
from applied_robotics.ces_optimizer import CESOptimizer
from applied_robotics_utilities.srv import CESPathOptimize
from applied_robotics_utilities.srv import CESOptimizeConfigure


class CESOptimizerService(Node):
    '''
    ROS2 service node for CES-based path optimization
    Methods:
        optimize_callback: Handles path optimization requests
        configuration_callback: Configures optimization parameters
    Attributes:
        optimize_srv: ROS2 service for path optimization
        configure_srv: ROS2 service for setting parameters
        is_configured: Tracks if the optimizer is set up
        alpha_k: Weight for smoothness
        rmin: Minimum turning radius
        avg_velocity: Average robot velocity
        lower_radius: Minimum bubble radius
        upper_radius: Maximum bubble radius
        obstacle_list: Obstacles as Shapely polygons
    '''
    def __init__(self):
        '''
        Initialize CES optimizer service
        '''
        super().__init__('ces_optimizer_service')
        self.optimize_srv = self.create_service(CESPathOptimize, 'ces_path_optimize', self.optimize_callback)
        self.configure_srv = self.create_service(CESOptimizeConfigure, 'ces_optimize_configure', self.configuration_callback)

        self.is_configured = False

        self.alpha_k = None
        self.rmin = None
        self.avg_velocity = None
        self.lower_radius = None
        self.upper_radius = None
        self.obstacle_list = []

    def optimize_callback(self, request, response):
        '''
        Handles path optimization requests
        args: request containing path
        return: response with optimized path
        '''
        # Must be configured before running CES
        if self.is_configured is not True:
            response.optimized_path = []
            return response

        response.optimized_path = []

        raw_path = [(point.x, point.y) for point in request.path]

        obstacles = Polygon()

        ces_optimizer = CESOptimizer(path = raw_path,
                                     obs = self.obstacle_list,
                                     alpha_k = self.alpha_k,
                                     rmin = self.rmin,
                                     velocity = self.avg_velocity,
                                     rl = self.lower_radius,
                                     ru = self.upper_radius)
        ces_optimizer.optimize()

        optimized_path = ces_optimizer.get_path()
        self.get_logger().info(f"{optimized_path}")
        response.optimized_path = [Point(x = x, y = y, z = 0.0) for x,y in optimized_path]

        return response

    def configuration_callback(self, request, response):
        '''
        Configures optimization parameters
        args: request with optimization parameters
        return: response indicating success
        '''
        self.alpha_k = request.alpha_k
        self.rmin = request.rmin
        self.avg_velocity = request.avg_velocity
        self.lower_radius = request.lower_radius
        self.upper_radius = request.upper_radius
        
        # Convert from geometry_msgs/Polygon to Shapely for CES
        self.obstacle_list = []
        for obstacle in request.obstacle_list:
            coords = [(point.x, point.y ) for point in obstacle.points]
            self.obstacle_list.append(Polygon(coords))

        self.is_configured = True
        response.success = True
        return response


def main():
    '''
    Initializes and runs the ROS2 CES optimizer service
    '''
    rclpy.init()
    node = CESOptimizerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
