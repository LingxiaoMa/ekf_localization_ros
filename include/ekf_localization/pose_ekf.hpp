#pragma once
#include <Eigen/Dense>
#include "ekf_localization/ekf_state.hpp"
#include "ekf_localization/ekf_control.hpp"

namespace ekf_localization{
    class pose_ekf
    {
    private:
    double alpha_q = 0.05;
    double alpha_r = 0.05;

    double time;
    Eigen::MatrixXd Qn_; // process noise
    Eigen::Vector3d last_delta_pos_;
    bool qn_initialized_ = false;


    Eigen::MatrixXd Rn_; // observation noise
    Eigen::MatrixXd P_; // covariance matrix
    Eigen::VectorXd x;

    public:
    pose_ekf();

    void setProcessNoise(const Eigen::MatrixXd& Qn);
    void setMeasurementNoise(const Eigen::MatrixXd& Rn);
    void init(const ekf_state& state);
    ekf_state getState() const;

    void predict(double time, const ekf_control& control);
    void update(double time, const Eigen::VectorXd& z);
    };
}