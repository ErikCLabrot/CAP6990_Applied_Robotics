# CAP6990 Assignment 8

This code is hosted at the following repository:
https://github.com/ErikCLabrot/CAP6990_Applied_Robotics

CAP6990 Assignment 7 is an implementation of an offline graph slam pipeline. Data is recorded and written to a file in a manner similar to the g2o format. This data is then used
to construct a pose graph optimization problem in ceres, run the problem, and validate the output with plots and calculations.

Additionally, there is a non-working implementation of the same pipeline using NLOpt contained in the project for future development and refinement.

Features are landmarked by aruco markers on the sides of obstacles in the environment. There is a semi-complete implementation of a lidar based landmarking feature. See the file 
in question for more details.


## Dependencies
This project depends on the following to run:

[ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

[Gazebo Sim 8.7](https://gazebosim.org/api/sim/8/install.html)

[Ros-Gazebo-Bridge for Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/)

[Shapely](https://pypi.org/project/shapely/)

[CVXPY](https://www.cvxpy.org/)

[SciPy](https://scipy.org/) - primarily for KDTree in the Lidar Odometry

[CERES] http://ceres-solver.org/

## Installation
Create a functioning ROS2 Workspace by following the ROS2 Documentation found [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Unzip the folder into your ROS2 Workspace /src folder. You should have an applied_robotics and applied_robotics_utilities folder in your /src folder now.

Navigate to the root of your workspace and build your workspace using

```bash
colcon build
```

A pre-compiled optimizer solution is provided in the offline_graph_optimizer. A CMakeLists.txt is also provided in case you'd like to build it yourself. Navigate to the build directory, and run the following (NOTE: Requires a c++17 compatible compiler)

```bash
cmake ..
make
```

## Usage
Collecting data can be performed by running the following command:

```
ros2 launch applied_robotics assignment_8.launch.py
```

This will create 3 files in the root of your workspace: ground_truth_poses.txt, laserscans.txt, pose_graph_log.txt. Copy these files, and move them to the data directory of offline_graph_optimizer.
Then, navigate to the build folder of offline_graph_optimizer.

The graph can be optimized by running the following:
```bash
./graph_slam_solver ../data/pose_graph_log.txt
```
This will run the solver, and produce a file in the data folder: graph_slam_result.txt

Two additonal python scripts are contained in the data folder to assist in data visualization.

graph_slam_plotter.py will produce a graph of the 3 different trajectories: ground_truth, EKF, and slam_optimized. It will also calculate the RMSE. If the results are not to your liking, 
then edit the info scaling factors in the .cpp file for graph_slam_optimizer, build the project again, and run with the same data. Increasing/decreasing the reliance on landmarks can sometimes
bring the graph in a little better.

You can run graph_slam_plotter with the following:

```bash
python3 graph_slam_solver.py
```

Likewise, there's also a script that will take the saved laserscans in laserscan.txt (which are actually deskewed coordinates, so these files are candidates for renaming in the future), 
compute the necessary transforms, and plot the ekf and graph-slam points, as well as the ground truth points for comparison. This can be run by:

```bash
python3 laserscan_plotter.py
```
