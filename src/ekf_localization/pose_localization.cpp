#include "ekf_localization/pose_localization.hpp"
#include <iostream>
#include <Eigen/Geometry>

namespace ekf_localization {

    pose_localization::pose_localization(){
        odom_initialized_ = false;
        pre_odom_pos_ = Eigen::Vector3d::Zero();
        pre_odom_q_ = Eigen::Quaterniond(1,0,0,0);
    }

    void pose_localization::addOdomPose(double time, const Eigen::Quaterniond& q, const Eigen::Vector3d& pos){
        std::lock_guard<std::mutex> lock(mtx_);

        ekf_control control;
        control.time = time;
        control.u = Eigen::VectorXd::Zero(7);
        control.u(6) = 1;
        if(odom_initialized_){
            Eigen::Quaterniond delta_q = pre_odom_q_.inverse() * q;
            delta_q.normalize();

            Eigen::Vector3d delta_pos = pre_odom_q_.inverse() * (pos - pre_odom_pos_);

            control.u.head<3>() = delta_pos;
            control.u(3) = delta_q.x();
            control.u(4) = delta_q.y();
            control.u(5) = delta_q.z();
            control.u(6) = delta_q.w();
        }
        controls_.push_back(control);
        if(this->controls_.size() > 1022){
            controls_.pop_front();
        }
        pre_odom_pos_ = pos;
        pre_odom_q_ = q;
        odom_initialized_ = true;

        ekf_.predict(time, control);
        states_.push_back(ekf_.getState());
    }

    void pose_localization::addVisionPose(double time, const Eigen::Quaterniond& q,
                            const Eigen::Vector3d& pos){
        std::lock_guard<std::mutex> lock(mtx_);

        Eigen::VectorXd z(7);
        z.head<3>() = pos;
        z(3) = q.x();
        z(4) = q.y();
        z(5) = q.z();
        z(6) = q.w();
        if(states_.empty()){
            ekf_state init_state;
            init_state.time = time;
            init_state.x = z;
            init_state.P = Eigen::MatrixXd::Identity(7,7);
            ekf_.init(init_state);
            states_.push_back(ekf_.getState());
            return;
        }else{
            ekf_state state = this->states_.front();
            std::deque<ekf_state> new_states;
            while (!states_.empty() && states_.front().time <= time)
            {
                state = states_.front();
                states_.pop_front();
            }
            
            this->ekf_.init(state);
            this->ekf_.update(time, z);
            new_states.push_back(ekf_.getState());

            while (!controls_.empty() && controls_.front().time < time)
            {
                controls_.pop_front();
            }
            for(const auto& ctrl : controls_){
                ekf_.predict(ctrl.time, ctrl);
                new_states.push_back(ekf_.getState());
            }
            states_ = std::move(new_states);
        }                         
    }  
    
    ekf_state pose_localization::getState() const{
        std::lock_guard<std::mutex> lock(mtx_);
        if (!states_.empty()){
            return states_.back();
        }
        std::cerr << "[pose_localization] Warning: getState() called but states_ is empty." << std::endl;
        return ekf_.getState();
        }

    
    
    std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose_localization::getPose(double time) const {
        std::lock_guard<std::mutex> lock(mtx_);

        if(states_.empty() || time < states_.front().time){
            std::cerr<< "[pose_localization] No valid state for requested time " << time << std::endl;
            return {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
        }

        if(time >= states_.back().time){
            ekf_state state = states_.back();
            Eigen::Quaterniond q(state.x(6), state.x(3), state.x(4), state.x(5));
            Eigen::Vector3d pos = state.x.head<3>();
            return {q, pos};
        }

        ekf_state pre_state = states_.front();
        ekf_state next_state = states_.back();
        for (size_t i = 1; i < states_.size(); i++){
            if (states_[i].time >= time){
                next_state = states_[i];
                break;
            }
            pre_state = states_[i];
        }

        if(pre_state.time >= next_state.time){
            std::cerr<<"[pose_localization] Invalid timestamp order in states_." << std::endl;
            return {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
        }

        double alpha = ((time- pre_state.time) / (next_state.time - pre_state.time));
        Eigen::Vector3d pre_pos = pre_state.x.head<3>();
        Eigen::Vector3d next_pos = next_state.x.head<3>();
        Eigen::Vector3d interp_pos = pre_pos + alpha * (next_pos - pre_pos);

        Eigen::Quaterniond pre_q(pre_state.x(6), pre_state.x(3), pre_state.x(4), pre_state.x(5));
        Eigen::Quaterniond next_q(next_state.x(6), next_state.x(3), next_state.x(4), next_state.x(5));

        pre_q.normalize();
        next_q.normalize();

        Eigen::Quaterniond interp_q = pre_q.slerp(alpha, next_q);
        interp_q.normalize();

        return {interp_q, interp_pos};
    }
}