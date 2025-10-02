// graph_slam_solver.cpp

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <array>
#include <sstream>

struct Pose2D {
    double x, y, theta;
};

struct LandmarkObservation {
    int pose_id;
    int landmark_id;
    double dx, dy;
    double info_x, info_y;
};

struct CmdEdge {
    int from, to;
    double v, w;
};

struct LandmarkResidual {
    LandmarkResidual(double dx, double dy, double info_x, double info_y)
        : dx_(dx), dy_(dy), info_x_(info_x), info_y_(info_y) {}

    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, T* residual) const {
        T dx = landmark[0] - pose[0];
        T dy = landmark[1] - pose[1];

        T cos_theta = ceres::cos(pose[2]);
        T sin_theta = ceres::sin(pose[2]);

        T lx = cos_theta * dx + sin_theta * dy;
        T ly = -sin_theta * dx + cos_theta * dy;

        residual[0] = lx - T(dx_);
        residual[1] = ly - T(dy_);
        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double info_x, double info_y) {
        return new ceres::AutoDiffCostFunction<LandmarkResidual, 2, 3, 2>(
            new LandmarkResidual(dx, dy, info_x, info_y));
    }

    double dx_, dy_, info_x_, info_y_;
};

struct MotionModelResidual {
    MotionModelResidual(double v, double w, double dt)
        : v_(v), w_(w), dt_(dt) {}

    template <typename T>
    bool operator()(const T* const x0, const T* const x1, T* residual) const {
        T theta0 = x0[2];
        T delta_x = v_ * dt_ * ceres::cos(theta0);
        T delta_y = v_ * dt_ * ceres::sin(theta0);
        T delta_theta = w_ * dt_;

        T x0_pred[3] = {x0[0] + delta_x, x0[1] + delta_y, x0[2] + delta_theta};

        residual[0] = x1[0] - x0_pred[0];
        residual[1] = x1[1] - x0_pred[1];
        residual[2] = ceres::internal::NormalizeAngle(x1[2] - x0_pred[2]);

        return true;
    }

    static ceres::CostFunction* Create(double v, double w, double dt) {
        return new ceres::AutoDiffCostFunction<MotionModelResidual, 3, 3, 3>(
            new MotionModelResidual(v, w, dt));
    }

    double v_, w_, dt_;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./graph_slam_solver log.txt\n";
        return 1;
    }

    std::ifstream infile(argv[1]);
    std::string line;

    std::map<int, Pose2D> poses;
    std::map<int, std::array<double, 2>> landmarks;
    std::vector<LandmarkObservation> landmark_obs;
    std::vector<CmdEdge> cmd_edges;

    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "VERTEX_SE2") {
            int id;
            double x, y, theta;
            ss >> id >> x >> y >> theta;
            poses[id] = {x, y, theta};
        } else if (tag == "CMD") {
            int from, to;
            double v, w;
            ss >> from >> to >> v >> w;
            cmd_edges.push_back({from, to, v, w});
        } else if (tag == "EDGE_SE2_XY") {
            int pose_id, lmk_id;
            double dx, dy, info_val, dummy;
            ss >> pose_id >> lmk_id >> dx >> dy >> info_val >> dummy;
            landmark_obs.push_back({pose_id, lmk_id, dx, dy, info_val, info_val});
            if (landmarks.find(lmk_id) == landmarks.end())
                landmarks[lmk_id] = {0.0, 0.0}; // Initialize landmark
        }
        // EDGE_SE2 is ignored
    }

    ceres::Problem problem;

    // Add pose variables
    for (auto& [id, pose] : poses) {
        problem.AddParameterBlock(&pose.x, 3);
        if (id == 0) problem.SetParameterBlockConstant(&pose.x); // fix first pose
    }

    // Add landmark variables
    for (auto& [id, lmk] : landmarks)
        problem.AddParameterBlock(lmk.data(), 2);

    // Motion model residuals from CMD
    for (const auto& cmd : cmd_edges) {
        ceres::CostFunction* cost = MotionModelResidual::Create(cmd.v, cmd.w, 0.1);  // dt = 0.1s
        problem.AddResidualBlock(cost, nullptr, &poses[cmd.from].x, &poses[cmd.to].x);
    }

    // Landmark residuals
    for (const auto& obs : landmark_obs) {
        ceres::CostFunction* cost = LandmarkResidual::Create(obs.dx, obs.dy, obs.info_x, obs.info_y);
        problem.AddResidualBlock(cost, nullptr, &poses[obs.pose_id].x, landmarks[obs.landmark_id].data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    std::ofstream out("graph_slam_result.txt");
    for (const auto& [id, pose] : poses)
        out << "POSE " << id << " " << pose.x << " " << pose.y << " " << pose.theta << "\n";
    for (const auto& [id, lmk] : landmarks)
        out << "LANDMARK " << id << " " << lmk[0] << " " << lmk[1] << "\n";

    out.close();
    return 0;
}