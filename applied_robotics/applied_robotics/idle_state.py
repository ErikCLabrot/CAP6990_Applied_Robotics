"""idle_state for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines an idle state for the FSM. This module is currently a placeholder
for a state that will idle the robot, and expose communication interfaces for 
sending goals or requesting robot actions while the robot is doing nothing
"""


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from applied_robotics.fsm_state import FSMState

class IdleState(FSMState):
    """
    Represents an idle state in the FSM, waits for new waypoints or events to transition

    Methods:
        execute_state: Executes the idle state logic
        get_status: Retrieves current state status
        exit: Handles state exit logic

    Attributes:
        fsm: FSM reference for state transitions
    """

    def __init__(self, state_name):
        super().__init__(state_name)

    def execute_state(self):
        """
        Executes the idle state logic

        Args: None
        """
        pass

    def get_status(self):
        """
        Retrieves current state status

        Args: None

        Returns:
            None
        """
        pass

    def exit_state(self):
        """
        Handles state exit logic

        Args: None
        """     
        pass

