'''
Pose Graph Optimizer for CAP6990 Assignment 7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This file encapsulates an attempt at solving the pose graph optimization problem with NLOpt
The problem is parsed from file, and J_graph_slam is constructed and then passed to the solver

This attempt was not very successful, with issues in setup resulting in improper output
(i.e. poor optimization or no optimization)

This code is a mixture of hand-written and llm generated (ChatGPT Model 4o)
'''

import numpy as np
import nlopt
import math

# ---------- Utilities ----------
def angle_wrap(a):
    '''wrap angle to [-pi, pi]

    args:
        a: input angle in radians

    returns:
        wrapped angle in radians
    '''
    return (a + np.pi) % (2 * np.pi) - np.pi

def parse_pose_graph(filename):
    '''parse graph slam file format into components

    args:
        filename: path to pose graph log

    returns:
        poses: dict of pose_id to np.array([x, y, theta])
        odom_edges: list of odometry constraints
        landmark_edges: list of landmark observation constraints
        pose_ids: sorted list of pose IDs
        landmark_ids: sorted list of landmark IDs
    '''
    poses = {}
    landmarks = set()
    odom_edges = []
    landmark_edges = []

    with open(filename, 'r') as f:
        for line in f:
            tokens = line.strip().split()
            if not tokens:
                continue

            if tokens[0] == 'VERTEX_SE2':
                idx = int(tokens[1])
                x, y, theta = map(float, tokens[2:5])
                poses[idx] = np.array([x, y, theta])

            elif tokens[0] == 'EDGE_SE2':
                id1, id2 = int(tokens[1]), int(tokens[2])
                dx, dy, dtheta = map(float, tokens[3:6])
                info = list(map(float, tokens[6:]))
                info_matrix = np.array([[info[0], info[1], info[2]],
                                        [info[1], info[3], info[4]],
                                        [info[2], info[4], info[5]]])
                odom_edges.append((id1, id2, np.array([dx, dy, dtheta]), info_matrix))

            elif tokens[0] == 'EDGE_SE2_XY':
                pose_id, landmark_id = int(tokens[1]), int(tokens[2])
                dx, dy = map(float, tokens[3:5])
                info = list(map(float, tokens[5:]))
                info_matrix = np.array([[info[0], info[1]],
                                        [info[1], info[2]]])
                landmark_edges.append((pose_id, landmark_id, np.array([dx, dy]), info_matrix))
                landmarks.add(landmark_id)

    pose_ids = sorted(poses.keys())
    landmark_ids = sorted(landmarks)

    return poses, odom_edges, landmark_edges, pose_ids, landmark_ids

# ---------- Optimization Setup ----------
def build_state_vector(poses, landmarks, pose_ids, landmark_ids):
    '''construct initial state vector from poses and placeholder landmarks

    args:
        poses: dict of pose_id to np.array([x, y, theta])
        landmarks: ignored
        pose_ids: sorted list of pose ids
        landmark_ids: sorted list of landmark ids

    returns:
        x: 1D numpy array of initial guess
    '''
    x = []
    for pid in pose_ids:
        x.extend(poses[pid])
    for lid in landmark_ids:
        x.extend([0.0, 0.0])
    return np.array(x)

def get_pose(x, idx, pose_ids):
    '''extract pose from state vector

    args:
        x: state vector
        idx: pose id
        pose_ids: list of pose ids

    returns:
        pose: np.array([x, y, theta])
    '''
    i = pose_ids.index(idx)
    return x[3*i : 3*i+3]

def get_landmark(x, lid, pose_ids, landmark_ids):
    '''extract landmark from state vector

    args:
        x: state vector
        lid: landmark id
        pose_ids: list of pose ids
        landmark_ids: list of landmark ids

    returns:
        landmark: np.array([x, y])
    '''
    j = landmark_ids.index(lid)
    offset = 3 * len(pose_ids) + 2 * j
    return x[offset : offset+2]

def objective(x, grad, odom_edges, landmark_edges, pose_ids, landmark_ids):
    '''compute total cost function for graph slam

    args:
        x: state vector
        grad: gradient (not used)
        odom_edges: list of odometry edges
        landmark_edges: list of landmark edges
        pose_ids: list of pose ids
        landmark_ids: list of landmark ids

    returns:
        cost: scalar cost
    '''
    cost = 0.0
    for id1, id2, measurement, info in odom_edges:
        x1 = get_pose(x, id1, pose_ids)
        x2 = get_pose(x, id2, pose_ids)
        dx = x2[0] - x1[0]
        dy = x2[1] - x1[1]
        dtheta = angle_wrap(x2[2] - x1[2])
        c = math.cos(-x1[2])
        s = math.sin(-x1[2])
        local = np.array([c * dx - s * dy, s * dx + c * dy, dtheta])
        err = local - measurement
        err[2] = angle_wrap(err[2])
        cost += err.T @ info @ err

    for pid, lid, obs, info in landmark_edges:
        pose = get_pose(x, pid, pose_ids)
        landmark = get_landmark(x, lid, pose_ids, landmark_ids)
        dx = landmark[0] - pose[0]
        dy = landmark[1] - pose[1]
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        pred = np.array([c * dx + s * dy, -s * dx + c * dy])
        err = obs - pred
        cost += err.T @ info @ err

    return cost

# ---------- Main Runner ----------
def run_optimization(file_in, file_out):
    '''run nlopt-based graph optimization and write result to file

    args:
        file_in: input pose graph log
        file_out: output file to write optimized results
    '''
    poses, odom_edges, landmark_edges, pose_ids, landmark_ids = parse_pose_graph(file_in)
    x0 = build_state_vector(poses, landmark_ids, pose_ids, landmark_ids)

    def wrapped_obj(x, grad):
        return objective(x, grad, odom_edges, landmark_edges, pose_ids, landmark_ids)

    opt = nlopt.opt(nlopt.LD_LBFGS, len(x0))
    opt.set_min_objective(wrapped_obj)
    opt.set_xtol_rel(1e-6)
    x_opt = opt.optimize(x0)

    with open(file_out, 'w') as f:
        for i, pid in enumerate(pose_ids):
            px, py, pt = x_opt[3*i:3*i+3]
            f.write(f"VERTEX_SE2 {pid} {px} {py} {pt}\n")

        for j, lid in enumerate(landmark_ids):
            offset = 3 * len(pose_ids) + 2 * j
            lx, ly = x_opt[offset:offset+2]
            f.write(f"LANDMARK {lid} {lx} {ly}\n")

def main():
    '''run optimization on default files'''
    run_optimization('pose_graph_log.txt', 'optimized_graph.txt')

if __name__ == "__main__":
    main()
