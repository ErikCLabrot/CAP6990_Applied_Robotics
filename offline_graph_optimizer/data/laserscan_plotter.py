"""laserscan plotter for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This module defines graphing functionality to assist in visualizing the results of the graph optimization process
The laserscans associated with specific poses are rotated and translated into the coordinate frame of a given pose and plotted
It's assumed that the stored laserscans are already in coordinate point format and reasonably de-skewed
The code in this module was mostly provided by an llm (ChatGPT Model 4o)
"""

import numpy as np
import matplotlib.pyplot as plt

def load_poses(filename):
    '''load poses from log file in VERTEX_SE2 or POSE format

    args:
        filename: path to file containing pose data

    returns:
        poses: list of np.array [x, y, theta]
    '''
    poses = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("VERTEX_SE2") or line.startswith("POSE"):
                tokens = line.strip().split()
                x, y, theta = map(float, tokens[-3:])
                poses.append(np.array([x, y, theta]))
    print(f"Loaded {len(poses)} poses from {filename}")
    return poses

def load_ground_truth(filename):
    '''load ground truth poses from file

    args:
        filename: path to file containing ground truth pose data

    returns:
        poses: list of np.array [x, y, theta]
    '''
    poses = []
    with open(filename, 'r') as f:
        for line in f:
            tokens = list(map(float, line.strip().split()))
            if len(tokens) == 3:
                poses.append(np.array(tokens))
    print(f"Loaded {len(poses)} ground truth poses from {filename}")
    return poses

def load_cmds(filename):
    '''load velocity commands between poses

    args:
        filename: path to file containing CMD entries

    returns:
        cmds: dict mapping pose id to (v, w) tuple
    '''
    cmds = {}
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("CMD"):
                _, from_id, to_id, v, w = line.strip().split()
                cmds[int(to_id)] = (float(v), float(w))
    print(f"Loaded {len(cmds)} velocity commands (not applied)")
    return cmds

def load_laserscans(filename):
    '''load 2D lidar scans in point format

    args:
        filename: path to file containing scan data, one scan per line

    returns:
        scans: list of 2xN numpy arrays
    '''
    scans = []
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            tokens = list(map(float, line.strip().split()))
            if not tokens:
                continue
            try:
                scan_points = np.array(tokens).reshape(-1, 2).T  # (2, N)
                scans.append(scan_points)
            except ValueError:
                print(f"⚠️ Could not reshape scan at line {i}")
    print(f"Loaded {len(scans)} scans from {filename}")
    return scans

def transform_scan(scan_points, pose):
    '''transform 2D scan points to world frame using pose

    args:
        scan_points: 2xN numpy array of scan points
        pose: [x, y, theta] robot pose

    returns:
        transformed: 2xN numpy array of transformed points
    '''
    x, y, theta = pose
    T = np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta),  np.cos(theta), y],
        [0, 0, 1]
    ])
    homog = np.vstack((scan_points, np.ones((1, scan_points.shape[1]))))  # (3, N)
    world = T @ homog
    return world[:2]

def main():
    '''main function to load data, transform scans, and plot comparisons'''
    original_poses = load_poses("pose_graph_log.txt")
    optimized_poses = load_poses("graph_slam_result.txt")
    ground_truth_poses = load_ground_truth("ground_truth_poses.txt")
    cmds = load_cmds("pose_graph_log.txt")  # Loaded for future use

    scans = load_laserscans("laserscans.txt")
    n = min(len(original_poses), len(optimized_poses), len(ground_truth_poses), len(scans))
    print(f"Processing {n} scan/pose pairs")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    for i in range(n):
        scan = scans[i]

        raw_world = transform_scan(scan, original_poses[i])
        opt_world = transform_scan(scan, optimized_poses[i])
        gt_world  = transform_scan(scan, ground_truth_poses[i])

        ax1.plot(raw_world[0], raw_world[1], 'r.', markersize=1, alpha=0.2)
        ax1.plot(opt_world[0], opt_world[1], 'b.', markersize=1, alpha=0.2)
        ax2.plot(gt_world[0],  gt_world[1],  'g.', markersize=1, alpha=0.2)

    ax1.set_title("EKF (Red) vs SLAM (Blue)")
    ax2.set_title("Ground Truth (Green)")

    for ax in [ax1, ax2]:
        ax.set_aspect('equal')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True)

    plt.suptitle("Transformed LiDAR Scans (No Deskewing)")
    plt.show()

if __name__ == "__main__":
    main()