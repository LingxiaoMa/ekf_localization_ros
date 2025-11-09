#pragma once
#include <mutex>
#include <deque>
#include "ekf_localization/pose_ekf.hpp"
#include "ekf_localization/ekf_state.hpp"
#include "ekf_localization/ekf_control.hpp"

namespace ekf_localization{
    class pose_localization
    {
        private:
        mutable std::mutex mtx_;
        pose_ekf ekf_;
        
        std::deque<ekf_state> states_;
        std::deque<ekf_control> controls_;

        bool odom_initialized_;
        Eigen::Quaterniond pre_odom_q_;
        Eigen::Vector3d pre_odom_pos_;

        public:
        pose_localization();

        void addOdomPose(double time, const Eigen::Quaterniond& q, const Eigen::Vector3d& pos);
        void addVisionPose(double time, const Eigen::Quaterniond& q, const Eigen::Vector3d& pos);
        
        ekf_state getState() const;

        std::pair<Eigen::Quaterniond, Eigen::Vector3d> getPose(double time) const;
    };
}