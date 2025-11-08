#include "ekf_localization/pose_ekf.hpp"
#include "ekf_localization/ekf_state.hpp"
#include "ekf_localization/ekf_control.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace ekf_localization;

int main() {
    std::cout << "==== [EKF TEST] Pose EKF simple test ====" << std::endl;

    // ---------------- 初始化 EKF ----------------
    pose_ekf ekf;

    ekf_state init_state;
    init_state.time = 0.0;
    init_state.x = Eigen::VectorXd::Zero(7);
    init_state.x(6) = 1.0; // 单位四元数
    init_state.P = Eigen::MatrixXd::Identity(7,7) * 0.1;

    ekf.init(init_state);

    std::cout << "Initial state:" << std::endl;
    std::cout << "pos = " << init_state.x.head<3>().transpose() << std::endl;
    std::cout << "quat = " << init_state.x.tail<4>().transpose() << std::endl;

    // ---------------- 控制输入 ----------------
    ekf_control u;
    u.time = 1.0;
    u.u = Eigen::VectorXd::Zero(7);
    u.u(0) = 1.0;  // dx = 1 m
    u.u(1) = 0.0;  // dy
    u.u(2) = 0.0;  // dz
    // 姿态增量：绕 z 轴旋转 45 度
    Eigen::AngleAxisd rot(M_PI/4, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond dq(rot);
    u.u(3) = dq.x(); u.u(4) = dq.y(); u.u(5) = dq.z(); u.u(6) = dq.w();

    // ---------------- 执行预测 ----------------
    ekf.predict(1.0, u);

    auto state_pred = ekf.getState();
    std::cout << "\nAfter predict:" << std::endl;
    std::cout << "pos = " << state_pred.x.head<3>().transpose() << std::endl;
    std::cout << "quat = " << state_pred.x.tail<4>().transpose() << std::endl;

    // ---------------- 模拟观测更新 ----------------
    Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
    z(0) = 1.1; // x 稍微偏差
    z(1) = 0.1; // y 有观测误差
    z(2) = 0.0;
    z(3) = dq.x(); z(4) = dq.y(); z(5) = dq.z(); z(6) = dq.w();

    ekf.update(1.0, z);

    auto state_upd = ekf.getState();
    std::cout << "\nAfter update:" << std::endl;
    std::cout << "pos = " << state_upd.x.head<3>().transpose() << std::endl;
    std::cout << "quat = " << state_upd.x.tail<4>().transpose() << std::endl;

    // ---------------- 归一化检查 ----------------
    double qnorm = state_upd.x.segment<4>(3).norm();
    std::cout << "\nQuaternion norm = " << qnorm << std::endl;

    std::cout << "==== [EKF TEST] Finished ====" << std::endl;
    return 0;
}
