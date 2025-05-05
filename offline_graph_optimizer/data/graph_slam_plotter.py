"""
Graph SLAM plotter for CAP6990 Applied Robotics Assignment_7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This module defines graphing functionality to assist in visualizing the results of the graph optimization process
This module plots the path and landmarks from ground truth, the EKF, and post pose graph optimization
The RMSE value for the two datasets is also reported
"""

import matplotlib.pyplot as plt
import numpy as np

def load_poses_from_log(filename):
    '''
    Load 2D poses from a log file with VERTEX_SE2 entries

    args:
        filename: path to file containing pose graph log entries

    returns:
        poses: list of (x, y) tuples
    '''
    poses = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("VERTEX_SE2"):
                _, _, x, y, _ = line.strip().split()
                poses.append((float(x), float(y)))
    return poses

def load_poses_and_landmarks_from_result(filename):
    '''
    Load optimized poses and landmarks from a result file

    args:
        filename: path to file containing optimized results

    returns:
        poses: list of (x, y) tuples
        landmarks: list of (x, y) tuples
    '''
    poses = []
    landmarks = []
    with open(filename, 'r') as f:
        for line in f:
            tokens = line.strip().split()
            if tokens[0] == "POSE":
                _, _, x, y, _ = tokens
                poses.append((float(x), float(y)))
            elif tokens[0] == "LANDMARK":
                _, _, x, y = tokens
                landmarks.append((float(x), float(y)))
    return poses, landmarks

def load_ground_truth(filename):
    '''
    Load ground truth 2D poses from a file

    args:
        filename: path to ground truth file

    returns:
        poses: list of (x, y) tuples
    '''
    poses = []
    with open(filename, 'r') as f:
        for line in f:
            x, y, _ = line.strip().split()
            poses.append((float(x), float(y)))
    return poses

def plot_trajectories(original, optimized, ground_truth, landmarks):
    '''
    Plot original, optimized, and ground truth trajectories with landmarks

    args:
        original: list of (x, y) tuples from EKF
        optimized: list of (x, y) tuples from Graph SLAM
        ground_truth: list of (x, y) tuples from ground truth
        landmarks: list of (x, y) tuples
    '''
    ox, oy = zip(*original)
    fx, fy = zip(*optimized)
    gx, gy = zip(*ground_truth)

    plt.figure(figsize=(10, 8))
    plt.plot(ox, oy, 'r--o', label='Original (EKF)')
    plt.plot(fx, fy, 'g-o', label='Optimized (Graph SLAM)')
    plt.plot(gx, gy, 'k-^', label='Ground Truth')

    if landmarks:
        lx, ly = zip(*landmarks)
        plt.scatter(lx, ly, c='blue', marker='x', label='Landmarks')

    plt.legend()
    plt.title("Pose Graph Optimization with Ground Truth")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def compute_rmse(estimated, ground_truth):
    '''
    Compute RMSE between estimated and ground truth 2D poses

    args:
        estimated: list of (x, y) tuples
        ground_truth: list of (x, y) tuples

    returns:
        rmse: float
    '''
    assert len(estimated) == len(ground_truth), "Pose lists must be the same length"
    errors = np.array(estimated) - np.array(ground_truth)
    squared_errors = np.sum(errors**2, axis=1)
    mse = np.mean(squared_errors)
    rmse = np.sqrt(mse)
    return rmse

if __name__ == "__main__":
    original_poses = load_poses_from_log("pose_graph_log.txt")
    optimized_poses, landmarks = load_poses_and_landmarks_from_result("graph_slam_result.txt")
    ground_truth_poses = load_ground_truth("ground_truth_poses.txt")
    rmse_ekf = compute_rmse(original_poses, ground_truth_poses)
    rmse_slam = compute_rmse(optimized_poses, ground_truth_poses)
    print(f"RMSE (EKF vs Ground Truth): {rmse_ekf:.4f}")
    print(f"RMSE (SLAM vs Ground Truth): {rmse_slam:.4f}")
    plot_trajectories(original_poses, optimized_poses, ground_truth_poses, landmarks)
