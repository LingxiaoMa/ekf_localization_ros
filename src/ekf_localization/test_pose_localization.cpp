#include "ekf_localization/pose_localization.hpp"
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>

using namespace ekf_localization;

int main() {
    std::cout << "==== [EKF TEST] Pose Localization Integration Test ====" << std::endl;

    pose_localization loc;

    // === 模拟 1: 连续里程计数据（沿 X 轴移动） ===
    for (int i = 0; i < 5; ++i) {
        double t = i * 1.0; // 每秒一次
        Eigen::Vector3d pos(i * 1.0, 0.0, 0.0);  // 每秒前进 1m
        Eigen::Quaterniond q(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())); // 不旋转
        loc.addOdomPose(t, q, pos);
    }

    // 输出当前状态
    ekf_state s1 = loc.getState();
    std::cout << "[After Odom Prediction] pos = " << s1.x(0) << ", " << s1.x(1) << ", " << s1.x(2)
              << " | quat = (" << s1.x(3) << ", " << s1.x(4) << ", " << s1.x(5) << ", " << s1.x(6) << ")\n";

    // === 模拟 2: 视觉定位校正（偏右 0.2m） ===
    Eigen::Vector3d vision_pos(4.8, 0.2, 0.0);
    Eigen::Quaterniond vision_q(Eigen::AngleAxisd(M_PI/18, Eigen::Vector3d::UnitZ())); // 约10°旋转
    loc.addVisionPose(5.0, vision_q, vision_pos);

    // 输出更新后状态
    ekf_state s2 = loc.getState();
    std::cout << "[After Vision Update] pos = " << s2.x(0) << ", " << s2.x(1) << ", " << s2.x(2)
              << " | quat = (" << s2.x(3) << ", " << s2.x(4) << ", " << s2.x(5) << ", " << s2.x(6) << ")\n";

    // === 模拟 3: 继续里程计移动 ===
    for (int i = 6; i <= 10; ++i) {
        double t = i * 1.0;
        Eigen::Vector3d pos(vision_pos(0) + (i - 5) * 1.0, vision_pos(1), 0.0);
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/18, Eigen::Vector3d::UnitZ())); // 保持同角度
        loc.addOdomPose(t, q, pos);
    }

    // 输出最新状态
    ekf_state s3 = loc.getState();
    std::cout << "[After More Odom] pos = " << s3.x(0) << ", " << s3.x(1) << ", " << s3.x(2)
              << " | quat = (" << s3.x(3) << ", " << s3.x(4) << ", " << s3.x(5) << ", " << s3.x(6) << ")\n";

    // === 模拟 4: 时间插值查询 ===
    double query_time = 7.5;
    auto [q_interp, pos_interp] = loc.getPose(query_time);
    std::cout << "[Interpolated at " << query_time << "s] pos = " << pos_interp.transpose()
              << " | quat = " << q_interp.coeffs().transpose() << std::endl;

    std::cout << "==== [EKF TEST] Finished ====" << std::endl;
    return 0;
}
