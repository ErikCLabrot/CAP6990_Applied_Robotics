""" Move Server for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

ROS Action Server for handling waypoint path planning. Wraps a position based velocity controller
to navigate a robot between its current position and a target position.
"""


import rclpy
import math
import threading

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Twist, Quaternion
from nav_msgs.msg import Odometry
from applied_robotics_utilities.action import MoveToGoal
from applied_robotics.position_controller import PositionController


class MoveServer(Node):
    '''
    Action Server for Movement States

    Attributes:
        action_server (ActionServer): ROS2 Action Server for MoveToGoal
        pos_controller (PositionController): Position based controller
    '''

    def __init__(self, pose_topic, vel_topic):
        '''
        Initialize MoveToGoal server

        Args:
            pose_topic (str): Pose subscriber topic
            vel_topic (str): Velocity publisher topic
        '''
        super().__init__('move_action_server')
        self.action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            execute_callback=self.execute_goal_cb,
            cancel_callback=self.cancel_callback
        )

        self.pose_topic = self.declare_parameter('pose_topic', '/robot/pose').value
        self.odometry_topic = self.declare_parameter('odometry_topic', '/robot/odometry').value
        self.command_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value

        self.pos_controller = PositionController(node=self, pose_topic=self.pose_topic, vel_topic=self.command_topic, odom_topic=self.odometry_topic ,use_pose=False )

    def execute_goal_cb(self, goal_handle):
        '''
        Goal execution callback

        Args:
            goal_handle (ServerGoalHandle): Action Goal Handle

        Returns:
            MoveToGoal.Result: Result of Action
        '''
        goal_point = goal_handle.request.target_point
        goal_orientation = 0  # goal_handle.request.target_pose.orientation.z

        result = MoveToGoal.Result()
        feedback = MoveToGoal.Feedback()

        self.get_logger().info(f'Received Goal Position: x= {goal_point.x}, y= {goal_point.y}')
        self.get_logger().info(f'Received Goal Orientation theta= {goal_orientation}')

        # Check input message validity
        if goal_point.x is not None and goal_point.y is not None:
            self.pos_controller.start(goal_point, goal_orientation)

        # Run position control until goal is reached or canceled
        while not self.pos_controller.final_reached:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel request received. Stopping motion.")
                self.pos_controller.stop()
                goal_handle.canceled()
                result.success = False
                return result  # Exit execution immediately

            self.pos_controller.run()
            feedback.current_pos = self.pos_controller.get_curr_pos()
            goal_handle.publish_feedback(feedback)

        # Success
        if self.pos_controller.is_pos_reached():
            self.get_logger().info("Move Server is Returning a Success!")
            self.pos_controller.stop()
            result.success = True
            goal_handle.succeed()
        else:
            self.get_logger().info("Move Server is Returning a Failure!")
            self.pos_controller.stop()
            result.success = False
            goal_handle.abort()

        return result

    def cancel_callback(self, goal_handle):
        '''
        Goal cancel callback

        Args:
            goal_handle (ServerGoalHandle): Action goal handle

        Returns:
            CancelResponse: Accept or reject cancel request
        '''
        self.get_logger().info('Cancel request received. Stopping motion.')
        self.pos_controller.stop()  # Ensure robot stops when canceling
        return CancelResponse.ACCEPT

    def _yaw_from_quat(self, quat):
        '''
        Retrieve yaw from a quaternion (i.e., orientation)

        Args:
            quat (Quaternion): Orientation quat

        Returns:
            float: Yaw (radians)
        '''
        a = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        b = 1.0 - 2.0 * (quat.y**2 + quat.z**2)
        return math.atan2(a, b)


def main(args=None):
    rclpy.init(args=args)
    pose_topic = '/robot_pose'
    vel_topic = '/cmd_vel'
    move_server = MoveServer(pose_topic=pose_topic, vel_topic=vel_topic)
    mtexec = MultiThreadedExecutor()
    mtexec.add_node(move_server)
    mtexec.spin()

    move_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
