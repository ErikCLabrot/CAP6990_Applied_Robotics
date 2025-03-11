""" move_state for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines a state for a Finite State Machine that handles waypoint to waypoint motion
tasks, and informs the FSM of the next state depending on whether an obstacle was detected
or if an error occurs
"""


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from threading import Lock
from geometry_msgs.msg import Point, PoseArray
from shapely.geometry import Polygon
from applied_robotics.fsm_state import FSMState
from applied_robotics_utilities.action import MoveToGoal
from applied_robotics_utilities.srv import MoveStateCancel

class MoveState(FSMState):
    def __init__(self, state_name, data_buffer, action_topic='/move_to_goal'):
        """
        Handles robot movement to waypoints using an action client, detects obstacles to trigger replanning

        Methods:
            execute_state: Initiates motion to the next waypoint
            get_status: Retrieves current movement status
            exit_state: Handles state exit, determines transition code based on conditions
            send_goal: Sends goal to action server
            goal_response_cb: Callback for goal response, updates status
            result_cb: Callback for result, checks success and updates status
            obs_cb: Callback for obstacle detection, triggers goal cancellation if needed
            cancel_response_cb: Callback for goal cancellation, updates status

        Attributes:
            fsm: FSM reference for state transitions
            obs_lock: Thread lock for obstacle detection
            obs_sub: ROS subscriber for obstacle data
            action_client: ROS action client for MoveToGoal
            transition_code: Code for state transition
            status: Current state status
            running: Flag for state execution
            action_accepted: Flag for action acceptance
            obs_detected: Flag for obstacle detection
            move_success: Flag for move success
            goal_handle: Handle for the current action goal
        """
        super().__init__(state_name)
        self.data_buffer = data_buffer
        self.return_data = None
        self.obs_lock = Lock()

        self.obs_sub = self.create_subscription(PoseArray, '/obstacles', self.obs_cb, 1)
        self.action_client = ActionClient(self, MoveToGoal, action_topic)
        #self.cancel_service = self.create_service(MoveStateCancel, 'move_state_cancel', self.cancel_service_callback)

        self.transition_code = None
        self.status = None

        self.running = False
        self.action_accepted = False
        self.obs_detected = False
        self.move_success = False

        self.goal_handle = None

    def execute_state(self):
        """
        Initiates motion to the next waypoint

        Args: None
        """

        if self.action_client.wait_for_server():
            self.running = True
            self.status = "RUNNING"
            self.send_goal()

    def get_status(self):
        """
        Retrieves current movement status

        Args: None

        Returns:
            str: Current status
        """
        if self.status is not None:
            return self.status

        self.get_logger().info(f"STATE STATUS: {self.status}")
        return "ERROR"

    def exit_state(self):
        """
        Handles state exit, determines transition code based on conditions

        Args: None

        Returns:
            str: Transition code for FSM
        """
        with self.obs_lock:
            self.get_logger().info(f"DEBUG: obs_detec = {self.obs_detected}")

        #self.get_logger().info(f"MoveState instance id: {id(self)}")

        if len(self.data_buffer["path"]) == 0:
            self.transition_code = "IDLE"

        with self.obs_lock:
            if self.obs_detected:
                self.transition_code = "OBSTACLE_DETECTED"

        if not self.obs_detected:
            self.transition_code = "NEXT_WP"

        #self.get_logger().info(f'Movestate returning tcode: {self.transition_code}')
        return self.transition_code, self.return_data

    def send_goal(self):
        """
        Sends goal to action server

        Args: None
        """
        if not self.data_buffer["path"]:
            self.status = "FINISHED"
            return

        wp = self.data_buffer["path"].pop(0)

        goal_msg = MoveToGoal.Goal()
        #goal_msg.target_point.x = float(wp[0])
        #goal_msg.target_point.y = float(wp[1])

        goal_msg.target_point = wp

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """
        Callback for goal response, updates status

        Args:
            future: Future object containing goal response
        """

        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.status = "ERROR"
            self.action_accepted = False
            return

        self.action_accepted = True
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        """
        Callback for result, checks success and updates status

        Args:
            future: Future object containing result
        """
        result = future.result().result

        if self.obs_detected:
            return

        if result.success:
            self.move_success = True
        else:
            self.move_success = False

        self.status = "FINISHED"

    
    def obs_cb(self, obs_msg):
        """
        Callback for obstacle detection, triggers goal cancellation if needed

        Args:
            obs_msg: PoseArray containing detected obstacles
        """
        obs_coords = self.data_buffer["obstacle_centers"]
        obstacles = self.data_buffer["obstacles"]
        half_size = 0.5
        #self.get_logger().info(f"{obs_list}")
        for pose in obs_msg.poses:
            pos = (round(pose.position.x, 2), round(pose.position.y, 2))

            # Check if this position is already known
            if pos not in obs_coords:
                obs_coords.append(pos)
                self.get_logger().info(f"New obstacle detected at {pos}")
                x = pos[0]
                y = pos[1]
                poly = Polygon([
                        (x - half_size, y - half_size),
                        (x + half_size, y - half_size),
                        (x + half_size, y + half_size),
                        (x - half_size, y + half_size)
                    ])
                obstacles.append(poly)

                with self.obs_lock:
                    self.obs_detected = True
                #self.get_logger().info(f"MoveState instance id: {id(self)}")

            with self.obs_lock:
                if self.obs_detected and self.goal_handle is not None:
                    cancel_future = self.goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self.cancel_response_cb)
        self.return_data = {"obstacles" : obstacles}
    

    def cancel_response_cb(self, future):
        """
        Callback for goal cancellation, updates status

        Args:
            future: Future object containing cancellation response
        """
        self.status = "FINISHED"
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")

    '''
    #Bridging old behavior for now, rewrite to de-scope obstacle detection details from motion
    #Should just be told to cancel, and why. Shouldn't know where obstacles are. 
    #For now, only reason to cancel is new obstacle
    def cancel_service_callback(self, request, response):
        self.obs_detected = True
        self.obs = {"obstacles" : request.obstacle_list}
        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_cb)
    '''

