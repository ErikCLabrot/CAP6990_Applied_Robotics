""" pid_controller for CAP6990 Applied Robotics Assignment_4

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 2/11/2025

This module defines a simple pid controller. P I and D terms are calculated
and applied to an input error calculated outside the pid to produce an output
in the terms of a control variable.
"""
import time


class PIDController:
    '''
    Simple PID Controller

    Attributes:
        kp (float): Proportional gain
        ki (float): Integral gain
        kd (float): Derivative gain
        max_speed (float): Maximum output

    Methods
        __init__(self, kp, ki, kd, max_speed): Initializes the PID controller
        start(self, setp): Starts the PID controller with a given setpoint
        calculate_dt(self): Calculates the time delta
        update(self, e): Performs PID calculation and returns control output
        reset(self): Resets the PID controller
        set_setp(self, setp): Sets a new setpoint for the controller
        set_kp(self, kp): Sets the proportional gain
        set_ki(self, ki): Sets the integral gain
        set_kd(self, kd): Sets the derivative gain
    '''

    def __init__(self, kp, ki, kd, max_speed):
        '''
        Initializes the PID controller.

        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            max_speed (float): Maximum output
        '''
        self.running = False

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed = max_speed

        self.prev_t = time.time()
        self.prev_e = 0.0
        self.integ = 0.0
        self.setp = 0.0
        self.prev_out = 0.0

    def start(self, setp):
        '''
        Start PID

        Args:
            setp (float): setpoint
        '''
        self.reset()
        self.setp = setp
        self.prev_e = setp
        self.running = True

    def calculate_dt(self):
        '''
        Calculate Time Delta

        Returns:
            float: time delta
        '''
        curr_t = time.time()
        dt = curr_t - self.prev_t
        self.prev_t = curr_t
        return dt

    def update(self, e):
        '''
        PID Calculation Loop
        Args:
            e (float): The current error

        Returns:
            float: Control Output
        '''
        dt = self.calculate_dt()

        # Calculate Integral
        self.integ += e * dt

        # Calculate Derivative
        deriv = (e - self.prev_e) / dt if dt > 0 else 0.0
        self.prev_e = e

        # Compute PID Output
        output = (self.kp * e) + (self.ki * self.integ) + (self.kd * deriv)

        # Limit Output to Max Speed
        if output > 0:
            output = min(output, self.max_speed)
        elif output <= 0:
            output = max(output, -self.max_speed)
        return output


    def reset(self):
        '''
            Reset PID
        '''
        self.prev_t = time.time()
        self.integ = 0.0

    def set_setp(self, setp: float):
        '''
        Set new etpoint

        Args:
            setp (float): setpoint
        '''
        self.reset()
        self.setp = setp

    def set_kp(self, kp: float):
        '''
        Sets the proportional
        '''
        self.kp = kp

    def set_ki(self, ki: float):
        '''
        Sets the integral
        '''
        self.ki = ki

    def set_kd(self, kd: float):
        '''
        Sets the derivative
        '''
        self.kd = kd