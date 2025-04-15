"""Extneded Kalman Filter for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 4/8/2025

This module defines the implementation for an extended kalman filter that uses an IMU to
predict based off its sensing of control inputs, and updates based on velocity received from
a lidar odometry module.
"""

import numpy as np

class ekf():
    '''
    This class defines an extended kalman filter that predicts based on IMU velocity sensing,
    and updates its estimates based off lidar odometry.

    Params:
        dt: The time delta for the IMU prediction step, based off imu update rate
        state_vector: State vector for the robot that encompasses the states that define it, as well as additional observable states
        state_covariance: The state covariance matrix for the kalman filter process, tracks how much we should trust states over time
        process_noise_covariance: The covariance matrix that tells us how noisy our 'process' or update step is and how much we should trust predictions
        measurement_matrix: The measurement matrix that applies measurements in the update step to variables in the state matrix 
        measurement_noise_covariance: The covariance matrix that tells how noisy the measurements are in the update step.
    '''
    def __init__(self, imu_update_rate):
        '''
        Function to initialize the EKF. State is initialize zeroed. If a different starting state is desired, use set_state.

        args:
            imu_update_rate: Rate the imu updates at, used to set dt for the prediction step
        '''
        self.dt = 1 / imu_update_rate

        #x_k  - State Vector (x_k = [x, y, theta, v, w)
        self.state_vector = np.zeros((5,1))

        #P_k
        self.state_covariance = np.diag([1e-2, 1e-2, 1e-4, 1e-2, 1e-3])

        #Q
        linear_acceleration_variance = 0.1**2
        angular_velocity_variance = 0.01**2

        q_x = 0.25 * linear_acceleration_variance * self.dt**4
        q_y = 0.25 * linear_acceleration_variance * self.dt**4
        q_theta = 0.25 * angular_velocity_variance * self.dt**4
        q_v = linear_acceleration_variance * self.dt**2
        q_w = angular_velocity_variance * self.dt**2

        self.process_noise_covariance = np.diag([q_x, q_y, q_theta, q_v, q_w])

        #H
        self.measurement_matrix = np.array([
                                    [0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1]
                                ])
        #R
        self.measurement_noise_covariance = np.diag([9.5e-5, 1e-6])

    def set_state(self, state):
        '''
        For initializing the state vector if our start is not (0,0,0,0,0)

        args:
            state: The state to set the state vector to
        '''
        self.state_vector = state

    def get_state(self):
        '''Function to get the state vector'''
        return self.state_vector

    def predict(self, imu_angular_rate, imu_linear_acceleration):
        '''
        Predict step of Extended Kalman Filter, using control inputs and dynamics
        to estimate where robot should be after certain amount of time

        args:
            imu_angular_rate: Angular velocity from IMU reading
            imu_linear_acceleration: Linear acceleration from IMU Reading
        '''

        theta = self.state_vector[2,0]
        linear_velocity = self.state_vector[3,0]

        linear_velocity_new = linear_velocity + imu_linear_acceleration * self.dt

        #Predict State (state_k+1 = state_k + dynamics)
        self.state_vector[0,0] += linear_velocity * np.cos(theta) * self.dt
        self.state_vector[1,0] += linear_velocity * np.sin(theta) * self.dt
        self.state_vector[2,0] += imu_angular_rate * self.dt
        self.state_vector[3,0] = linear_velocity_new
        self.state_vector[4,0] = imu_angular_rate

        self.state_vector[2,0] = (self.state_vector[2,0] + np.pi) % (2 * np.pi) - np.pi

        #Compute the Jacobian matrix, Fk
        jacobian = np.array([
            [1, 0, -linear_velocity * np.sin(theta) * self.dt, np.cos(theta) * self.dt, 0],
            [0, 1, linear_velocity * np.cos(theta) * self.dt, np.sin(theta) * self.dt, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        #Predict Covariance Matrix
        self.state_covariance = jacobian @ self.state_covariance @ jacobian.T + self.process_noise_covariance

    def update(self, measurement_vector):
        '''
        Update step of Extended Kalman Filter, where sensor data is used to update
        state estimate

        args:
            measurement_vector: Measurements from lidar odometry, [v,w].T
        '''
        measurement_vector = np.array([[measurement_vector[0]],
                                      [measurement_vector[1]]])

        #Innovation Step
        residual_covariance = self.measurement_matrix @ self.state_covariance @ self.measurement_matrix.T + self.measurement_noise_covariance

        kalman_gain = self.state_covariance @ self.measurement_matrix.T @ np.linalg.inv(residual_covariance)

        self.state_vector += kalman_gain @ (measurement_vector - self.measurement_matrix @ self.state_vector)

        self.state_covariance = (np.eye(5) - kalman_gain @ self.measurement_matrix) @ self.state_covariance

        self.state_vector[2,0] = (self.state_vector[2,0] + np.pi) % (2 * np.pi) - np.pi