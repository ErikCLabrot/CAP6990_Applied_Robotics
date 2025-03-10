"""fsm for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines a Finite State Machine. This state machine holds definitions
for states and transtiions, and handles interfacing these states with the underlying
ROS framework.
"""
import rclpy
from rclpy.node import Node 
from rclpy.executors import SingleThreadedExecutor
from shapely.geometry import Point, LineString, Polygon
from threading import Thread, Event
from applied_robotics.move_state import MoveState
from applied_robotics.plan_state import PlanState
from applied_robotics.idle_state import IdleState


class FSM():
    """
    Finite State Machine for waypoint navigation

    Methods:
        run: Main loop, executes current state, handles transitions
        get_next_state: Determines and transitions to the next state
        execute_current_state: Executes the current state
        _default_state_transitions: Initializes state transition mapping

    Attributes:
        wp_list: List of waypoints for navigation
        obs_list: List of detected obstacles
        robot_bbox: Robot's bounding box as a Shapely Polygon
        max_robot_linear: Max linear velocity
        max_robot_angular: Max angular velocity
        goal_position: Goal position as (x, y, theta)
        world_size: Size of the world in meters
        state_map: Dictionary for state transitions
        current_state: Currently active state
        transition_code: Code indicating the next state transition
        mtexec: Executor for State execution
        exec_thread: Thread for Executor
    """    
    def __init__(self):

        self.wp_list = []
        self.obs_list = []
        self.robot_bbox = Polygon([(-1.5, -1.0), (-1.5, 1.0), (1.5, 1.0), (1.5, -1.0)])
        self.max_robot_linear = 1.0
        self.max_robot_angular = 3.14/4

        self.goal_position = (8,8,0)
        self.world_size = 10 #meters

        self.state_map = {}
        self.current_state = None
        self.transition_code = None

        self._default_state_transitions()

        self.mtexec = SingleThreadedExecutor()
        self.exec_thread = Thread(target=self.mtexec.spin)
        self.exec_thread.start()

    def run(self):
        """
        Main loop, executes current state, handles transitions

        Args: None
        """    
        while True:
            if self.current_state == None:
                self.get_next_state()
            status = self.current_state.get_status()
            if status == "ERROR" or status == "FINISHED":
                self.transition_code = self.current_state.exit_state()
                self.get_next_state()

    def get_next_state(self):
        """
        Determines and transitions to the next state

        Args: None
        """

        if self.current_state is None:
            self.current_state = PlanState(fsm=self,
                                           state_name= 'plan_state',
                                           goal_pos=self.goal_position,
                                           map_size=self.world_size,
                                           obs=self.obs_list,
                                           robot_bb=self.robot_bbox,
                                           max_v=self.max_robot_linear,
                                           max_w=self.max_robot_angular)
            self.execute_current_state()
            return

        current_class = self.current_state.__class__

        self.mtexec.remove_node(self.current_state)
        self.current_state.destroy_node()

        next_state = self.state_map[current_class][self.transition_code]

        if next_state == MoveState:
            self.current_state = MoveState(fsm = self, state_name='move_state')

        elif next_state == PlanState:
                        self.current_state = PlanState(fsm=self,
                                           state_name= 'plan_state',
                                           goal_pos=self.goal_position,
                                           map_size=self.world_size,
                                           obs=self.obs_list,
                                           robot_bb=self.robot_bbox,
                                           max_v=self.max_robot_linear,
                                           max_w=self.max_robot_angular)

        elif next_state == IdleState:
            self.current_state = IdleState(self)

        self.execute_current_state()

    def execute_current_state(self):
        """
        Executes the current state

        Args: None
        """

        self.current_state.get_logger().info(f"Type of current_state: {type(self.current_state)}")
        self.mtexec.add_node(self.current_state)
        self.current_state.execute_state()

    def _default_state_transitions(self):
        """
        Initializes state transition mapping

        Args: None
        """
        self.state_map[MoveState] = {
            "NEXT_WP": MoveState,
            "OBSTACLE_DETECTED": PlanState,
            "IDLE": IdleState
        }
        self.state_map[PlanState] = {
            "PLAN_SUCCESS": MoveState,
            "PLAN_FAIL": IdleState,
            "IDLE": IdleState
        }
        self.state_map[IdleState] = {
            "START": MoveState,
            "IDLE": IdleState
        }

def main(args=None):
    rclpy.init(args=args)
    fsm = FSM()

    fsm.run()

    fsm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
