# CAP6990 Assignment 5

This code is hosted at the following repository:
https://github.com/ErikCLabrot/CAP6990_Applied_Robotics

CAP6990 Assignment 5 is an implementation of the Convex Elastic Smoothing Algorithm as presented by Zhu et al, found [here](https://web.stanford.edu/~pavone/papers/Zhu.Schmerling.ea.CDC15.pdf)

This project takes a path produced by the RRT path planner from Assignment 4, applies the CES algorithm, then navigates a robot around an environment with obstacles placed in locations unknown to the robot.

When the robot detects an obstacle it has not yet seen, it will stop, calculate a path to its goal using RRT, Optimize the path using the CES algorithm, then begin navigation again.

## Dependencies
This project depends on the following to run:

[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Gazebo Sim 8.7](https://gazebosim.org/api/sim/8/install.html)

[Ros-Gazebo-Bridge for Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/)

[Shapely](https://pypi.org/project/shapely/)

[CVXPY](https://www.cvxpy.org/)

## Installation
Create a functioning ROS2 Workspace by following the ROS2 Documentation found [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Unzip the assignment3 folder into your ROS2 Workspace /src folder. You should have an applied_robotics and applied_robotics_utilities folder in your /src folder now.

Navigate to the root of your workspace and build your workspace using

```bash
colcon build
```

## Usage
A launch file to run the motion planning experiment is included in the /launch folder of the ROS workspace. This launch file will bring the simulation world up, launch the requisite ROS nodes, and prepare the experiment. The experiment is to successfully navigate the robot from (-8,-8) to (8,8) without colliding with any of the marker blocks.

When the robot detects a cube it hasn't accounted for yet, it will stop and re-plan. The simulation thread will pause while the plot displaying the newly formulated path is shown. In order to continue the simulation, the plot must be closed after inspection. Additionally, a plot showing the smoothed CES path will appear after closing the RRT plot. This must also be closed for the simulation to continue.

In order to launch the simulation, the following command may be used at command line:

```bash
ros2 launch applied_robotics assignment_5.launch.py
```

This will launch the Gazebo window, and begin displaying log output from the ROS nodes in the terminal. In order to begin the simulation, the play button in the Gazebo window must be pressed. This will then begin the sense-plan-move loop of the experiment.

Additionally, a test script is supplied in the applied_robotics/applied_robotics folder. The ces_test.py file can be run by opening the folder in a shell, and running the following

```bash
python3 ces_test.py
```

This test script contains a plot function and helper functions, as well as the CES algorithm removed from ROS. This allows for the CES algorithm operation to be verified independent of ROS.