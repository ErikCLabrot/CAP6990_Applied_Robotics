"""fsm_state for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/22/2025

This module defines a base state for the Finite State Machine. This state acts as a parent for all
states to interface with the FSM using. Common interfaces are provided that the FSM will use to execute 
the FSM loop.
"""

import rclpy
from rclpy.node import Node


class FSMState(Node):
    def __init__(self, node_name='fsm_state'):
        """
        Base class for FSM states, provides common interface for state execution and transitions

        Methods:
            execute_state: Executes the state logic
            get_status: Retrieves current state status
            exit_state: Handles state exit logic

        Attributes:
            transition_code: Code indicating the next state transition
            status: Current status of the state
        """

        super().__init__(node_name)
        self.transition_code = None
        self.status = None
        pass

    def execute_state(self):
        """
        Executes the state logic

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
