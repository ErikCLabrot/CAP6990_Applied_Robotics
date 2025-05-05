/**
Graph Slam Solver for CAP6990 Assignment 7

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 5/4/2025

This file contains the necessary code to optimize a pose graph using the Ceres non-linear solver.
A pose graph stored in a file is read, the required Residual Blocks are constructed (e.g. odometry and landmark observation edges)
and the pose graph is optimized. The file outputs a results file in the data folder containing the optimized locations of the 
graph vertices and landmark positions. The code in this file was mostly provided via LLM (ChatGPT Model 4o).
**/

#include <ceres/ceres.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>

/// Normalize an angle to the range [-pi, pi]
template <typename T>
T NormalizeAngle(T angle) {
    while (angle > T(M_PI)) angle -= T(2.0 * M_PI);
    while (angle < T(-M_PI)) angle += T(2.0 * M_PI);
    return angle;
}

/// Represents a 2D pose (x, y, theta)
struct Pose2D {
    double x, y, theta;
};

/// Represents a landmark observation from a robot pose, with info matrix diagonal
struct LandmarkObservation {
    int pose_id;
    int landmark_id;
    double dx, dy;
    double info_x, info_y;
};

/// Represents a control command (v, w) between two poses
struct CmdEdge {
    int from, to;
    double v, w;
};

/// Represents the information matrix diagonal from an EKF edge (xx, yy, tt)
struct EdgeInfo {
    int from, to;
    double info_x, info_y, info_theta;
};

/// Residual for a 2D landmark observation in the robot frame
struct LandmarkResidual {
    LandmarkResidual(double dx, double dy, double info_x, double info_y, double scale)
        : dx_(dx), dy_(dy), info_x_(info_x), info_y_(info_y), scale_(scale) {}

    /// Applies pose-to-landmark residual with sqrt information weighting
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, T* residual) const {
        T dx = landmark[0] - pose[0];
        T dy = landmark[1] - pose[1];

        T cos_theta = ceres::cos(pose[2]);
        T sin_theta = ceres::sin(pose[2]);

        // Transform landmark to robot frame
        T lx = cos_theta * dx + sin_theta * dy;
        T ly = -sin_theta * dx + cos_theta * dy;

        T weight_x = T(scale_) * T(std::sqrt(info_x_));
        T weight_y = T(scale_) * T(std::sqrt(info_y_));

        residual[0] = weight_x * (lx - T(dx_));
        residual[1] = weight_y * (ly - T(dy_));
        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double info_x, double info_y, double scale) {
        return new ceres::AutoDiffCostFunction<LandmarkResidual, 2, 3, 2>(
            new LandmarkResidual(dx, dy, info_x, info_y, scale));
    }

    double dx_, dy_, info_x_, info_y_, scale_;
};

/// Residual for predicting motion using control inputs and comparing to the next pose
struct MotionModelResidual {
    MotionModelResidual(double v, double w, double dt, double info_x, double info_y, double info_theta)
        : v_(v), w_(w), dt_(dt), info_x_(info_x), info_y_(info_y), info_theta_(info_theta) {}

    /// Predicts motion and returns weighted residual between predicted and actual pose
    template <typename T>
    bool operator()(const T* const x0, const T* const x1, T* residual) const {
        T theta0 = x0[2];
        T dx = T(v_) * T(dt_) * ceres::cos(theta0);
        T dy = T(v_) * T(dt_) * ceres::sin(theta0);
        T dtheta = T(w_) * T(dt_);

        T x0_pred[3] = { x0[0] + dx, x0[1] + dy, x0[2] + dtheta };

        T wx = T(std::sqrt(info_x_));
        T wy = T(std::sqrt(info_y_));
        T wt = T(std::sqrt(info_theta_));

        residual[0] = wx * (x1[0] - x0_pred[0]);
        residual[1] = wy * (x1[1] - x0_pred[1]);
        residual[2] = wt * NormalizeAngle(x1[2] - x0_pred[2]);
        return true;
    }

    static ceres::CostFunction* Create(double v, double w, double dt, double info_x, double info_y, double info_theta) {
        return new ceres::AutoDiffCostFunction<MotionModelResidual, 3, 3, 3>(
            new MotionModelResidual(v, w, dt, info_x, info_y, info_theta));
    }

    double v_, w_, dt_, info_x_, info_y_, info_theta_;
};

/// Entry point: loads pose graph, builds optimization problem, solves it with Ceres
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./graph_slam_solver pose_graph_log.txt\n";
        return 1;
    }

    std::ifstream infile(argv[1]);
    std::string line;

    std::map<int, Pose2D> poses;
    std::map<int, std::array<double, 2>> landmarks;
    std::vector<LandmarkObservation> landmark_obs;
    std::vector<CmdEdge> cmd_edges;
    std::map<std::pair<int, int>, EdgeInfo> edge_infos;

    // Parameters to tune influence of constraints
    double landmark_info_scale = 0.01;
    double motion_info_scale = 1.0;

    // Parse log file lines
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "VERTEX_SE2") {
            int id; double x, y, theta;
            ss >> id >> x >> y >> theta;
            poses[id] = { x, y, theta };
        } else if (tag == "CMD") {
            int from, to; double v, w;
            ss >> from >> to >> v >> w;
            cmd_edges.push_back({ from, to, v, w });
        } else if (tag == "EDGE_SE2_XY") {
            int pose_id, lmk_id;
            double dx, dy, info_val, dummy;
            ss >> pose_id >> lmk_id >> dx >> dy >> info_val >> dummy;
            landmark_obs.push_back({ pose_id, lmk_id, dx, dy, info_val, info_val });
            if (landmarks.find(lmk_id) == landmarks.end())
                landmarks[lmk_id] = { 0.0, 0.0 };
        } else if (tag == "EDGE_SE2") {
            int from, to; double dx, dy, dtheta;
            double xx, xy, xt, yy, yt, tt;
            ss >> from >> to >> dx >> dy >> dtheta >> xx >> xy >> xt >> yy >> yt >> tt;
            edge_infos[{from, to}] = { from, to, xx, yy, tt };
        }
    }

    ceres::Problem problem;

    // Add pose parameter blocks
    for (auto& [id, pose] : poses) {
        problem.AddParameterBlock(&pose.x, 3);
        if (id == 0) problem.SetParameterBlockConstant(&pose.x); // fix first pose
    }

    // Add landmark parameter blocks
    for (auto& [id, lmk] : landmarks)
        problem.AddParameterBlock(lmk.data(), 2);

    // Add landmark observation residuals
    for (const auto& obs : landmark_obs) {
        ceres::CostFunction* cost = LandmarkResidual::Create(
            obs.dx, obs.dy, obs.info_x, obs.info_y, landmark_info_scale);
        problem.AddResidualBlock(cost, nullptr, &poses[obs.pose_id].x, landmarks[obs.landmark_id].data());
    }

    // Add motion model residuals, using EDGE_SE2 info to weight residuals
    for (const auto& cmd : cmd_edges) {
        auto key = std::make_pair(cmd.from, cmd.to);
        if (edge_infos.count(key)) {
            EdgeInfo info = edge_infos[key];

            double scaled_info_x = info.info_x * motion_info_scale;
            double scaled_info_y = info.info_y * motion_info_scale;
            double scaled_info_theta = info.info_theta * motion_info_scale;

            ceres::CostFunction* cost = MotionModelResidual::Create(
                cmd.v, cmd.w, 0.1, scaled_info_x, scaled_info_y, scaled_info_theta);
            problem.AddResidualBlock(cost, nullptr, &poses[cmd.from].x, &poses[cmd.to].x);
        } else {
            std::cerr << "Warning: Missing EDGE_SE2 info for CMD from " << cmd.from << " to " << cmd.to << "\n";
        }
    }

    // Solver settings
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Write result to file
    std::ofstream out("../data/graph_slam_result.txt");
    for (const auto& [id, pose] : poses)
        out << "POSE " << id << " " << pose.x << " " << pose.y << " " << pose.theta << "\n";
    for (const auto& [id, lmk] : landmarks)
        out << "LANDMARK " << id << " " << lmk[0] << " " << lmk[1] << "\n";
    out.close();

    return 0;
}