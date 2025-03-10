from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription,SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    share_directory = get_package_share_directory('applied_robotics')
    res_path = PathJoinSubstitution([share_directory, 'models'])
    world_path = PathJoinSubstitution([share_directory, 'worlds'])
    world_file = PathJoinSubstitution([share_directory, 'worlds', 'aruco_world.world'])
    gz_bridge_config = PathJoinSubstitution([share_directory, 'config', 'gz_bridge.yaml'])
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', res_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_file],#[PathJoinSubstitution([world_path, 'aruco_world.world'])],
                'on_exit_shutdown' : 'True'
            }.items(),
        ),

        RosGzBridge(
            config_file= gz_bridge_config,
            bridge_name='gz_bridge'
            ),

        Node(
            package = 'applied_robotics',
            namespace= '',
            executable = 'move_server'),

        ])