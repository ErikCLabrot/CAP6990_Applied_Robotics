# CAP6990 Assignment 4

CAP6990 Assignment 4 is an implementation of the RRT Algorithm applied to a simulated robot, and an implementation of a visual fiduciary marker detector to accomplish pathing a robot with online motion planning. 

## Dependencies
This project depends on the following to run:

[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Gazebo Sim 8.7](https://gazebosim.org/api/sim/8/install.html)

[Ros-Gazebo-Bridge for Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/)

[Shapely](https://pypi.org/project/shapely/)

## Installation
Create a functioning ROS2 Workspace by following the ROS2 Documentation found [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Unzip the assignment3 folder into your ROS2 Workspace /src folder. You should have an assignment_4 and assignment_4_utilities folder in your /src folder now.

Navigate to the root of your workspace and build your workspace using

```bash
colcon build
```

## Usage
A launch file to run the motion planning experiment is included in the /launch folder of the ROS workspace. This launch file will bring the simulation world up, launch the requisite ROS nodes, and prepare the experiment. The experiment is to successfully navigate the robot from (-8,-8) to (8,8) without colliding with any of the marker blocks.

When the robot detects a cube it hasn't accounted for yet, it will stop and re-plan. The simulation thread will pause while the plot displaying the newly formulated path is shown. In order to continue the simulation, the plot must be closed after inspection. 

In order to launch the simulation, the following command may be used at command line:

```bash
ros2 launch assignment_4 assignment_4.launch.py
```

This will launch the Gazebo window, and begin displaying log output from the ROS nodes in the terminal. In order to begin the simulation, the play button in the Gazebo window must be pressed. This will then begin the sense-plan-move loop of the experiment.

The simulation world can be inspected prior to running in order to see the shapes in the world by running the following:

```bash
ros2 launch assignment_4 sim.launch.py
```

Additionally, if the obstacle positions are not satisfactory, their positions can be changed in the aruco_world.world file under the /worlds folder. By altering the <pose> tag for the aruco_box_(id) models, their position in the simulation world can be changed. 