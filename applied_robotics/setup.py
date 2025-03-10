from setuptools import find_packages, setup
import os
from glob import glob

package_name = "applied_robotics"

# List of directories that need to be copied as-is
model_dirs = [
    "models",
    "models/materials",
    "models/materials/textures",
    "models/materials/scripts"
]

world_dirs = [
    "worlds"
]

# Collect all files while preserving structure
model_files = []
for model_dir in model_dirs:
    for file in glob(f"{model_dir}/**", recursive=True):
        if os.path.isfile(file):  # Ensure it's a file, not a directory
            destination = os.path.join("share", package_name, os.path.dirname(file))
            model_files.append((destination, [file]))

world_files = [(os.path.join("share", package_name, "worlds"), glob("worlds/*.world"))]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        #include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        #include ros-gz-bridge config
        ('share/' + package_name + '/config',['config/gz_bridge.yaml']),
    ] + model_files + world_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="A ROS2 package for spawning Gazebo cubes with configurable Aruco textures",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        'move_server = applied_robotics.move_server:main',
        'fsm_node = applied_robotics.fsm:main',
        'aruco_node = applied_robotics.aruco_detector:main',
        ],
    },
)