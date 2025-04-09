# CAP6990 Assignment 7

This code is hosted at the following repository:
https://github.com/ErikCLabrot/CAP6990_Applied_Robotics

CAP6990 Assignment 7 is an implementation of an extended kalman filter, intended to improve the odometry estimate of the lidar odometry system. Additionally,
improvements were made to the odometry system developed in assignment 6, based on the ICP system described in the [KISS-ICP](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf)

This project implements an EKF that uses a simulated IMU with associated noise to sense the effects of control inputs applied to the robot. This is used in the 'predict' step of the EKF. Lidar Odometry
is used in the Update step to correct the short term noise and uncertainty of the imu with a more stable odometry estimate, albeit, one that drifts over time. This sensor fusion results in an over all improvmenet to the state estimate for the robot, staying very close to ground truth.

Additionally, improvements to the Lidar Odometry system were implemented. The euclidean nearest neighbor search was replaced with a KDTree (First with a custom implementation, then a library). The point cloud is downsampled using a voxelization schema, and a predicitve outlier filtering schema is implemented. This alterations make the ICP algorithm more robust, and prevent it from drifting for far longer. Most importantly, however, is that it does a far better job of estimating the immediate frame to frame odometry, which in turn feeds into the EKF for improvement.

This project is demonstrated primarily using teleop so that situations can be created that are designed to be difficult for the lidar odometry and EKF to handle, such as 'wiggling', spinning in place, and long linear translations in a feature-sparse environment. 

## Dependencies
This project depends on the following to run:

[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Gazebo Sim 8.7](https://gazebosim.org/api/sim/8/install.html)

[Ros-Gazebo-Bridge for Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/)

[Shapely](https://pypi.org/project/shapely/)

[CVXPY](https://www.cvxpy.org/)

[SciPy](https://scipy.org/) - primarily for KDTree in the Lidar Odometry

## Installation
Create a functioning ROS2 Workspace by following the ROS2 Documentation found [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Unzip the folder into your ROS2 Workspace /src folder. You should have an applied_robotics and applied_robotics_utilities folder in your /src folder now.

Navigate to the root of your workspace and build your workspace using

```bash
colcon build
```

## Usage
The primary launch file for this project launches a teleoperation demo using the teleop-twist-keyboard ROS package. This allows the robot to be controlled at different velocities and in different manners to attempt to 'force' the odometry into situations that are difficult for it to keep a good estimate in. These situations include oscillatory rotational motion (i.e. wiggling), close translations near boxes, translations through areas where there are few lidar features to calculate points off of, etc.

```bash
ros2 launch applied_robotics assignment_7.launch.py
```

This will launch the Gazebo window, and begin displaying log output from the ROS nodes in the terminal, as well as open two plot windows. In order to begin the simulation, the play button in the Gazebo window must be pressed. This will then begin the sense-plan-move loop of the experiment.

The instructions for controlling the robot through teleoperation can be read in the additional terminal window that opens. The recommended speed values that reflect the robots autonomous motion control values are a linear velocity of roughly 1m/s, and a rotational velocity of 0.5rad/s. 

