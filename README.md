# CAP6990 Assignment 6

This code is hosted at the following repository:
https://github.com/ErikCLabrot/CAP6990_Applied_Robotics

CAP6990 Assignment 6 is an implementation of the Iterative Closest Point algorithm, applied to a simulated robot to provide LiDAR based Odometry. 

This project takes a LaserScan produced by the Gazebo Simulation, converts it to cartesian coordinates, applies the ICP algorithm, and integrates this 
over time in order to provide an odometry estimate to the robot. This odometry is then used to supplant the ground truth information used in prior experiments.

Additionally, the difference between the LiDAR odometry and ground truth is plotted, to demonstrate how LiDAR based odometry drifts over time.

## Dependencies
This project depends on the following to run:

[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Gazebo Sim 8.7](https://gazebosim.org/api/sim/8/install.html)

[Ros-Gazebo-Bridge for Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/)

[Shapely](https://pypi.org/project/shapely/)

[CVXPY](https://www.cvxpy.org/)

## Installation
Create a functioning ROS2 Workspace by following the ROS2 Documentation found [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Unzip the folder into your ROS2 Workspace /src folder. You should have an applied_robotics and applied_robotics_utilities folder in your /src folder now.

Navigate to the root of your workspace and build your workspace using

```bash
colcon build
```

## Usage
There are two launch files that can be used to experiment with the LiDAR Odometry. The first demonstrates the same experiment as assignment_5, the CES optimized RRT path finding, except 
with LiDAR odometry instead of ground truth. This experiment can be launched using the following command at the command line:

```bash
ros2 launch applied_robotics assignment_6.launch.py
```

This will launch the Gazebo window, and begin displaying log output from the ROS nodes in the terminal, as well as open two plot windows. In order to begin the simulation, the play button in the Gazebo window must be pressed. This will then begin the sense-plan-move loop of the experiment.

Additionally, a teleoperation launch file is provided to allow for manual control of the robot, while still plotting and demonstrating the difference between LiDAR odometry and ground truth.
This launch file uses the teleoperation_twist_keyboard ROS package. Launching this will open the main Gazebo window, the plot windows, and an additional terminal presuming the project is being 
run in an ubuntu environment. With the additional terminal window for the teleop package open, the robot can be controlled as displayed by the package in the terminal.

This teleoperation experiment can be launched using the following at the command line:

```bash
ros2 launch applied_robotics teleop.launch.py
```
