#include "ekf_localization/pose_ekf.hpp"
#include <iostream>
#include <Eigen/Geometry>

namespace ekf_localization
{
    pose_ekf::pose_ekf(){
        time = 0.0;

        // initialize position and pose vector
        x = Eigen::VectorXd::Zero(7);
        x(6) = 1;

        P_ = Eigen::MatrixXd::Identity(7,7);
        Qn_ = Eigen::MatrixXd::Identity(7,7) * 1e-4;
        Rn_ = Eigen::MatrixXd::Identity(7,7) * 1e-2;
    }


    void pose_ekf::setProcessNoise(const Eigen::MatrixXd& Qn){
        if(Qn.rows() == 7 && Qn.cols() == 7){
            Qn_ = Qn;
        }
        else{
            std::cerr<< "[EKF] Invalid Qn size, expected 7*7."<<std::endl;
        }
    }

    void pose_ekf::setMeasurementNoise(const Eigen::MatrixXd& Rn){
        if(Rn.rows() == 7 && Rn.cols() == 7){
            Rn_ = Rn;
        }else{
            std::cerr<<"[EKF] Invalid Rn size, expected 7*7"<<std::endl;
        }
    }

    void pose_ekf::init(const ekf_state& state){
        time = state.time;
        x = state.x;
        P_ = state.P;
    }

    ekf_state pose_ekf::getState() const{
        ekf_state s;
        s.time = this->time;
        s.x = this->x;
        s.P = this->P_;
        return s;
    }

    void pose_ekf::predict(double new_time, const ekf_control& control){
        double dt = new_time - this->time;
        if(dt<0 || control.u.size() != 7){
            std::cerr << "[EKF] Invalid time difference in predict: dt < 0" << std::endl;
            return;
        }

        // get current quaternion pose
        Eigen::Quaterniond q_pre(x(6), x(3), x(4), x(5));
        q_pre.normalize();

        // get difference of position and quaternion
        Eigen::Vector3d delta_pos = control.u.head<3>();
        Eigen::Quaterniond delta_q(control.u(6), control.u(3),control.u(4), control.u(5));
        delta_q.normalize();

        // rotate delta position from robot base to world base
        Eigen::Vector3d rotated_delta = q_pre._transformVector(delta_pos);
        x.head<3>() += rotated_delta;

        // apply quaternion prediction to x
        Eigen::Quaterniond new_q = q_pre * delta_q;
        x(3) = new_q.x();
        x(4) = new_q.y();
        x(5) = new_q.z();
        x(6) = new_q.w();

        // Adaptive Qn
        double cur_norm = delta_pos.norm();

        if(!qn_initialized_){
            last_delta_pos_ = delta_pos;
            qn_initialized_ = true;
        }

        double prev_norm = last_delta_pos_.norm();
        last_delta_pos_ = delta_pos;
        double resdual = std::abs(cur_norm - prev_norm);

        for(int i = 0; i < 3;  i++){
            double q_old = Qn_(i, i);
            double q_target = resdual * resdual;
            double q_new = (1-alpha_q)*q_old + alpha_q* q_target;

            double q_min = 1e-6;
            double q_max = 1e-1;
            if(q_new < q_min) q_new = q_min;
            if(q_new > q_max) q_new = q_max;

            Qn_(i,i) = q_new;
        }

        double w = delta_q.w();
        w = std::max(-1.0, std::min(1.0, w));
        double angle = 2.0 * std::acos(w);         
        double target_rot = angle * angle;          
        const double alpha_q_rot = 0.01;

        for(int i = 3; i < 7; ++i){
            double q_old = Qn_(i,i);
            double q_new = (1 - alpha_q_rot)*q_old + alpha_q_rot * target_rot;

            double q_min = 1e-6;
            double q_max = 1e-1;
            if(q_new < q_min) q_new = q_min;
            if(q_new > q_max) q_new = q_max;

            Qn_(i,i) = q_new;
        }

        //Simplified state transforming matrix
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(7,7); // TODO:can be further optimized
        P_ = F * P_ * F.transpose() + Qn_;

        time = new_time;

    }

    void pose_ekf::update(double new_time, const Eigen::VectorXd& z){
        if(new_time < this->time || z.size() != 7){
            std::cerr<< "[EKF] Invalid updated input"<<std::endl;
            return;
        }

        Eigen::VectorXd z_mea = z;
        double dot_pro = x(3)*z(3) + x(4)*z(4) + x(5)*z(5) + x(6)*z(6);
        if(dot_pro<0){
            z_mea.segment<4>(3) = -z_mea.segment<4>(3);
        } 

        Eigen::VectorXd y = z_mea - x;

        double res_pos = (z_mea.head<3>() - x.head<3>()).norm();
        for(int i = 0; i < 3; i++){
            double R_old = Rn_(i,i);
            double R_new = (1-alpha_r)*R_old + alpha_r * res_pos * res_pos;
            Rn_(i,i) = std::max(1e-6, std::min(R_new, 1.0));
        }

        Eigen::Quaterniond q_pred(x(6), x(3), x(4), x(5));
        Eigen::Quaterniond q_meas(z_mea(6), z_mea(3), z_mea(4), z_mea(5));

        double d = q_pred.dot(q_meas);
        d = std::max(-1.0, std::min(1.0, d));
        double angle = 2.0 * acos(d);

        for(int i = 3; i < 7; i++){
            double R_old = Rn_(i,i);
            double R_new = (1-alpha_r)*R_old + alpha_r * angle * angle;
            Rn_(i,i) = std::max(1e-6, std::min(R_new, 1.0));
        }


        Eigen::MatrixXd S = this->P_ + this->Rn_;
        Eigen::MatrixXd K = this->P_ * S.inverse();

        x = x + K * y;
        
        // normalize quanterion
        Eigen::Vector4d q_vec = x.segment<4>(3);
        double q_norm = q_vec.norm();
        if(q_norm > 0){
            x.segment<4>(3) = q_vec / q_norm;
        }

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);
        P_ = (I - K) * P_ * (I-K).transpose() + K * Rn_ * K.transpose();

        time = new_time;

    }

} // namespace ekf_localization
