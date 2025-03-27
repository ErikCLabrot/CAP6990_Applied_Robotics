""" plan_state for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines a state for the FSM that wraps a motion planner, in this case RRT,
executes a planning action, and then informs the FSM that the planning is finished and which 
state to transition to next
"""
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Twist, Quaternion, TransformStamped, Point32
from geometry_msgs.msg import Polygon as GeometryPolygon
from nav_msgs.msg import Odometry
from shapely.geometry import Polygon as ShapelyPolygon
from applied_robotics.fsm_state import FSMState
from applied_robotics.rrt_planner import RRTPlanner
from applied_robotics_utilities.srv import CESPathOptimize, CESOptimizeConfigure
import matplotlib.pyplot as plt

class PlanState(FSMState):
    """
    Handles planning actions using RRTPlanner, communicates planning status to FSM

    Methods:
        execute_state: Executes the planning action
        get_status: Retrieves current planning status
        exit_state: Handles state exit, sets FSM waypoints if planning succeeded
        get_raw_path: Generates path using RRT, updates status
        pose_cb: Updates start pose using ground truth from TFMessage
        _yaw_from_quat: Extracts yaw from quaternion

    Attributes:
        fsm: FSM reference for state transitions
        start: Start pose for planning
        sim_time: Simulation time per iteration
        max_it: Maximum iterations for RRT
        rrt_planner: Instance of RRTPlanner for path generation
        pose_topic: ROS topic for robot pose
        pose_sub: ROS subscriber for robot pose
        frame: Reference frame for ground truth
        status: Current state status
        raw_path: Generated path from RRT
        """
    def __init__(self, data,  state_name, goal_pos, map_size, obs, robot_bb, max_v, max_w):
        super().__init__(state_name)
        self.start = None

        self.obs = data["obstacles"]    

        self.sim_time = 0.75 #sec/it
        self.max_it = 10000
        self.rrt_planner = RRTPlanner(start=self.start, goal=goal_pos, map_size=map_size, obs=self.obs, 
                              robot_bb=robot_bb, vx=max_v, vw=max_w, dt=self.sim_time, max_it=self.max_it)

        self.pose_topic = '/robot_pose'
        self.frame = 'vehicle_blue'

        use_pose = False

        if use_pose:
            self.pose_topic = '/robot_pose'
            self.frame = 'vehicle_blue'
            self.pose_sub = self.create_subscription(TFMessage, self.pose_topic, self.pose_cb, 1)
        else:
            self.odom_sub = self.create_subscription(Odometry, 'lidar/odometry', self.odometry_callback, 1)

        self.optimization_config_client = self.create_client(CESOptimizeConfigure, 'ces_optimize_configure')
        self.path_optimization_client = self.create_client(CESPathOptimize, 'ces_path_optimize')

        self.status = "STARTING"

        self.raw_path = []
        self.optimized_path = []

    def execute_state(self):
        """
        Executes the planning action

        Args: None
        """

        self.get_raw_path()
        self.get_optimized_path()
        self.status = "FINISHED"

    def get_status(self):
        """
        Retrieves current planning status

        Args: None

        Returns:
            str: Current status
        """
        return self.status

    def exit_state(self):
        """
        Handles state exit, sets FSM waypoints if planning succeeded

        Args: None

        Returns:
            str: Transition code for FSM
        """
        self.get_logger().info(f'{len(self.optimized_path)}')
        if len(self.optimized_path) > 0:
            self.optimized_path.pop(0)
            return "PLAN_SUCCESS", {"path" : self.optimized_path}
        else:
            return "PLAN_FAIL", None

    #Rewrite this to call CES Service
    #Maybe rewrite RRT to service as well?
    #Decouple plan state from logic
    def get_raw_path(self):
        """
        Generates path using RRT, updates status

        Args: None
        """
        self.status = "PLANNING"
        self.get_logger().info("PLANNING")
        while self.start is None:
            pass
        while not self.rrt_planner.get_path():
            self.rrt_planner.grow_tree(self.start)
            if len(self.rrt_planner.get_path()) == 0:
                self.get_logger().info("RRT Planning failed! Replanning until we find a valid path...")
        self.get_logger().info("RRT Planned Succesfully!")
        self.raw_path = self.rrt_planner.get_path()

        #Convert raw_path to Point for compatibility
        path = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in self.raw_path]
        self.raw_path = path

        #self.plot_path(self.raw_path)
        #self.status = "FINISHED"


    '''
    Maybe rewrite in the future to be more generic, to allow for more path planners/optimization schemes
    '''
    def get_optimized_path(self):
        #Call Config service
        self.configure_optimize_service()
        #Call Optimize service
        self.optimized_path = self.call_optimize_service()
        #self.plot_path(self.optimized_path)
    def configure_optimize_service(self):
        request = CESOptimizeConfigure.Request()

        request.alpha_k = 10.0
        request.rmin = 0.1
        request.avg_velocity = 1.0
        request.lower_radius = 0.05
        request.upper_radius = 2.5

        #Convert obs to geometry_msg/Polygon (Rewrite to be slimmer)
        poly = []
        for obstacle in self.obs:
            coords = [Point32(x = x, y = y, z = 0.0) for x, y in obstacle.exterior.coords]
            temp_poly = GeometryPolygon()
            temp_poly.points = coords
            poly.append(temp_poly)

        request.obstacle_list = poly

        future = self.optimization_config_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Optimize Configure Pass")

        return future.result()


    def call_optimize_service(self):
        request = CESPathOptimize.Request()
        request.path = self.raw_path
        self.get_logger().info("Optimizer Pass")

        future = self.path_optimization_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Optimizer Pass")
        return future.result().optimized_path


    def pose_cb(self, msg):
        '''
        Pose CB

        Args:
            msg (TFMessage): TFMessage to pull ground truth from
        '''

        for tf in msg.transforms:
            if tf.child_frame_id == self.frame:
                self.pos = Point(
                    x= tf.transform.translation.x,
                    y= tf.transform.translation.y,
                    z= tf.transform.translation.z
                    )
                r = tf.transform.rotation

        self.theta = self._yaw_from_quat(r)

        if self.start is None:
            self.start = (self.pos.x, self.pos.y, self.theta)

    def odometry_callback(self, msg):
        self.pos = Point(
            x = msg.pose.pose.position.x,
            y = msg.pose.pose.position.y,
            z = 0.0)

        orientation = msg.pose.pose.orientation
        self.theta = self._yaw_from_quat(orientation)

        if self.start is None:
            self.start = (self.pos.x, self.pos.y, self.theta)

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

    def plot_path(self, path):
        fig, ax = plt.subplots()

        for obs in self.obs:
            x,y = obs.exterior.xy
            ax.fill(x,y,color='red')

        path_x = [p.x for p in path]
        path_y = [p.y for p in path]

        ax.plot(path_x, path_y, color='orange', linewidth = 2)

        plt.show()


def plot_path(rrt_planner):
    """
    Plots the path generated by RRT

    Args:
        rrt_planner: RRTPlanner instance with generated path
    """
    fig, ax = plt.subplots()
    
    for obs in rrt_planner.obs:
        x, y = obs.exterior.xy
        ax.fill(x, y, color='red')

    ax.scatter(rrt_planner.start[0], rrt_planner.start[1], color='green', s=100)
    ax.scatter(rrt_planner.goal[0], rrt_planner.goal[1], color='blue', s=100)

    path = rrt_planner.get_path()
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, color='orange', linewidth=2)

    ax.set_xlim(-rrt_planner.map_size, rrt_planner.map_size)
    ax.set_ylim(-rrt_planner.map_size, rrt_planner.map_size)
    ax.set_aspect('equal', adjustable='box')
    
    # Show the plot
    plt.show()

