""" position_controller for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/11/2025

This module defines a position based velocity controller. Positions are constantly read and 
used to adjust robot velocity to facilitate navigation between two points. Process is 
'state machine like' in that the main run loop is context aware of the current state of the 
movement process. The movement process is rotate-move-rotate-finish.'
"""
import rclpy
import math
import time

from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from applied_robotics_utilities.action import MoveToGoal
from applied_robotics.pid_controller import PIDController

class PositionController():
    '''
    Class for position based velocity control of a robot. Uses PID controllers
    and operates on a rotate, translate, rotate, stop scheme

        Attributes:
        node (Node): ROS node
        vel_pub (Publisher): Publisher for velocity commands
        pose_sub (Subscriber): Subscriber for position updates
        frame (str): Frame of reference for position data
        linear_controller (PIDController): PID controller for linear velocity
        angular_controller (PIDController): PID controller for angular velocity
        start_pos (Point): Robot starting pos
        pos (Point): Current pos
        theta (float): Current yaw angle
        goal (Point): Target pos
        goal_theta (float): Target yaw
        goal_reached (bool): True if goal pos reached
        theta_reached (bool): True if target yaw reached
        final_reached (bool): True if both pos and yaw reached
        angle_reached (bool): True if target angle reached
        running (bool): True if controller is active
        accept_radius (float): Distance threshold for goal pos
        accept_angle (float): Angular threshold for target yaw
        STOP (int): State for stopping
        ROTATE_TO_GOAL (int): State for rotating to goal
        TRANSLATE_TO_GOAL (int): State for translating to goal
        ROTATE_TO_FINAL (int): State for rotating to final yaw
        state (int): Current state

    Methods:
        __init__(node, pose_topic, vel_topic):  Init ros node and pos control vars
        start(goal, angle):  Starts position control
        stop(): Emergency Stop Robot
        reset(): Reset controller
        run(): Run position controller as state machine. Rotate to goal point, translate to goal point, rotate to final, stop
        get_state(): Returns current controller state
        _rotate(target_angle): Rotates the robot towards the target angle
        _translate(target_pos): Translates the robot towards the target pos
        _brake(): Brake robot between states for stability
        get_curr_pos(): Gets current pos
        is_pos_reached(): Check if goal reached
        is_angle_reached(angle): Checks if yaw reached
        _linear_distance(target_pos): Compute euclidean distance and dir to point
        _calculate_angle(target_pos): Calc angle between two points
        _angular_error(angle): Calc angle between two angles
        _check_distance(): Internal goal reached flag check
        _check_angle(angle): Internal angle reached flag check
        set_goal_point(goal): Set goal pos
        set_goal_theta(theta): Set goal angle
        _yaw_from_quat(quat): Retrieve yaw from quat (rot about z axis)
        _pose_cb(msg): Pose CB
        _publish_control(control): Publish control msg to cmd_vel
    '''
    
    def __init__(self, node, pose_topic, vel_topic, odom_topic, use_pose=False):
        '''
        Init ros node and pos control vars

        Args:
            node (Node): Ros nod
            pose_topic (str): Topic to receive Pose on
            vel_topic (str): Topic to publish cmd to
        '''
        #ROS config
        self.node = node
        self.vel_pub = self.node.create_publisher(Twist, vel_topic, 1)

        if use_pose:
            self.frame = "vehicle_blue"
            self.pose_sub = self.node.create_subscription(TFMessage, pose_topic, self._pose_cb, 1)
        else:
            self.odometry_sub = self.node.create_subscription(Odometry, odom_topic, self._odom_callback, 1)

        #PID process variable controller config
        self.linear_controller = PIDController(kp= 1.0, ki= 0.00, kd= 0.00, max_speed= 0.5)
        self.angular_controller = PIDController(kp= 1.5, ki= 0.5, kd= 0.1, max_speed= 1.0)

        #Position Control process variable config
        self.start_pos = Point(x=0.0, y=0.0, z=0.0)
        self.pos = Point(x= 0.0, y= 0.0, z= 0.0)
        self.theta = 0.0

        self.goal = Point(x= 0.0, y= 0.0, z= 0.0)
        self.goal_theta = 0.0

        #Finished Move Flags
        self.goal_reached = False
        self.theta_reached = False
        self.final_reached = False

        #Interal State Check Flags
        self.angle_reached = False
        self.running = False
        self.accept_radius = 0.15
        self.accept_angle = math.radians(0.005)

        #State Definitions
        self.STOP = 0
        self.ROTATE_TO_GOAL = 1
        self.TRANSLATE_TO_GOAL = 2
        self.ROTATE_TO_FINAL = 3

        self.state = self.STOP

    def start(self, goal, angle):
        '''
        Starts position control

        Args:
            goal (Point): Target pos
            angle (float): Target yaw
        '''
        self.running = True

        self.start_pos = self.pos
        self.goal = goal
        self.goal_theta = angle

        self.angle_reached = False
        self.theta_reached = False
        self.goal_reached = False
        self.final_reached = False

        linear_setpoint = self._linear_distance(target_pos=self.goal)
        angle = self._calculate_angle(target_pos=self.goal)
        angular_setpoint = self._angular_error(angle=angle)
        self.goal_theta = angle

        self.linear_controller.start(setp=linear_setpoint)
        self.angular_controller.start(setp=angular_setpoint)

        self.node.get_logger().info(f'Calculated angle to rotate to as: {angle} radians')
        self.node.get_logger().info(f'Accept_angle is {self.accept_angle}')
        self.state = self.ROTATE_TO_GOAL

    def stop(self):
        '''
        Emergency Stop Robot
        '''
        self.running = False
        self.state = self.STOP

        #Construct 0 Velocity Twist Message
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        #Stop vehicle
        self._publish_control(control=stop_msg)

    def reset(self):
        '''
        Reset controller
        '''
        self.running = False
        self.state = self.STOP
        self.goal_reached = False
        self.goal        

    def run(self):
        '''
        Run position controller as state machine. Rotate to goal point, translate to goal point, rotate to final, stop
        '''
        if self.running is not True:
            return

        #State: Rotate To Goal
        if self.state == self.ROTATE_TO_GOAL:
            angle_to_goal = self._calculate_angle(self.goal)
            if not self.is_angle_reached(angle_to_goal):
                self._rotate(angle_to_goal)
            else:
                self._brake()
                self.state = self.TRANSLATE_TO_GOAL

        #State: Translate to Goal
        if self.state == self.TRANSLATE_TO_GOAL:
            if not self.is_pos_reached():
                self._translate(self.goal)
            else:
                self._brake()
                self.state = self.STOP
                self.final_reached = True
                self.theta_reached = True

        #State: Rotate to Final Pose
        if self.state == self.ROTATE_TO_FINAL:
            if not self.is_angle_reached(self.goal_theta):
                self._rotate(self.goal_theta)
            else:
                self._brake()
                self.state = self.STOP
                self.final_reached = True
                self.stop()

        #State: STOP
        if self.state == self.STOP:
            self.stop()

    def get_state(self):
        '''
        Returns current controller state

        Returns:
            int: The current state 
        '''
        return self.state

    def _rotate(self, target_angle):
        '''
        Rotates the robot towards the target angle 

        Args:
            target_angle (float): target yaw
        '''
        if self.running is True:
            ang_err = target_angle - self.theta
            ang_input = self.angular_controller.update(e=ang_err)

            control_msg = Twist()
            control_msg.linear.x = 0.0
            control_msg.angular.z = ang_input

            self._publish_control(control=control_msg)
            if abs(ang_err) <= self.accept_angle:
                self.angle_reached = True
        else:
            self.stop()

    def _translate(self, target_pos):
        '''
        Translates the robot towards the target pos

        Args:
            target_pos (Point): goal point
        '''
        if self.running is True:
            linear_e = self._linear_distance(target_pos=self.goal)
            linear_input = self.linear_controller.update(e=linear_e)

            control_msg = Twist()
            control_msg.linear.x = linear_input
            control_msg.angular.z = 0.0

            self._publish_control(control=control_msg)

    def _brake(self):
        '''
        Brake robot between states for stability
        '''
        if self.running is True:
            control_msg = Twist()
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0

            self._publish_control(control=control_msg)

    def get_curr_pos(self):
        '''
        Gets current pos

        Returns:
            Point: Current pos
        '''
        return self.pos

    def is_pos_reached(self):
        '''
        Check if goal reached

        Returns:
            bool: True if reached, false else
        '''
        self._check_distance()
        return self.goal_reached

    def is_angle_reached(self, angle):
        '''
        Checks if yaw reached

        Args:
            angle (float): Target yaw

        Returns:
            bool: True if reached, false else
        '''
        self._check_angle(angle=angle)
        return self.angle_reached

    def _linear_distance(self, target_pos):
        '''
        Compute euclidean distance and dir to point

        Args:
            target_pos (Point): Target 

        Returns:
            float: Distance, pos/neg by dir
        '''
        dx = target_pos.x - self.pos.x
        dy = target_pos.y - self.pos.y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Vector from start to target
        v_goal = (target_pos.x - self.start_pos.x, target_pos.y - self.start_pos.y)
        d_goal = math.sqrt(v_goal[0]**2 + v_goal[1]**2)
        
        # Vector from start to current position
        v_pos = (self.pos.x - self.start_pos.x, self.pos.y - self.start_pos.y)
        
        # Dot product to project current position onto line to target
        dot_product = v_goal[0] * v_pos[0] + v_goal[1] * v_pos[1]
        
        # If projection is beyond target, we're overshooting
        if dot_product > d_goal**2:
            return -dist
        
        return dist

    def _calculate_angle(self, target_pos):
        '''
        Calc angle between two points

        Args:
            target_pos (Point): Target pos

        Returns:
            float: Angle between points
        '''
        dx = target_pos.x - self.pos.x
        dy = target_pos.y - self.pos.y

        angle = math.atan2(dy, dx)

        return angle

    def _angular_error(self, angle):
        '''
        Calc angle between two angles

        Args:
            angle (float): Target angle

        Returns:
            float: Angular error
        '''
        x = self.theta - angle
        return self.theta - angle

    def _check_distance(self):
        '''
        Internal goal reached flag check
        '''
        if abs(self._linear_distance(target_pos=self.goal)) <= self.accept_radius:
            self.goal_reached = True
        else:
            self.goal_reached = False

    def _check_angle(self, angle):
        '''
        Internal angle reached flag check

        Args:
            angle (float): target angle
        '''
        if abs(self._angular_error(angle)) <= self.accept_angle:
            self.angle_reached = True
        else:
            self.angle_reached = False

    def set_goal_point(self, goal):
        '''
        Set goal pos

        Args:
            goal (Point): target pos for robot
        '''
        self.goal = goal

    def set_goal_theta(self, theta):
        '''
        Set goal angle

        Args:
            theta (float): Target yaw(rads)
        '''
        self.goal_theta = theta

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

    def _pose_cb(self, msg):
        '''
        Pose CB

        Args:
            msg (TFMessage): TFMessage to pull ground truth from
        '''

        #Doing an implicit ws to cs conversion here
        #It would be better if it had a system to pass an x,y,theta tf at runtime
        for tf in msg.transforms:
            if tf.child_frame_id == self.frame:
                self.pos = Point(
                    x= tf.transform.translation.x,
                    y= tf.transform.translation.y,
                    z= tf.transform.translation.z
                    )
                r = tf.transform.rotation

    def _odom_callback(self, msg):
        self.pos = Point(
            x = msg.pose.pose.position.x,
            y = msg.pose.pose.position.y,
            z = 0.0
            )

        orientation = msg.pose.pose.orientation

        self.theta = self._yaw_from_quat(orientation)

    def _publish_control(self, control):
        '''
        Publish control msg to cmd_vel

        Args:
            control (Twist): control msg
        '''
        self.vel_pub.publish(control)