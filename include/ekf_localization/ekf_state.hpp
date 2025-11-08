#pragma once
#include <Eigen/Dense>
#include <iostream>

namespace ekf_localization{
    struct ekf_state
    {
        double time; // timestamp
        Eigen::VectorXd x; // position and pose
        Eigen::MatrixXd P; // covariance matrix

        // constructor
        ekf_state(){
            // data initialize
            time = 0.0;
            x = Eigen::VectorXd::Zero(7);
            x(6) = 1.0; 
            P = Eigen::MatrixXd::Identity(7,7);
        }

        // return position
        Eigen::Vector3d position() const{
            return Eigen::Vector3d(x(0),x(1),x(2));
        }

        // return quaternion
        Eigen::Quaterniond pose() const{
            return Eigen::Quaterniond(x(6), x(3), x(4), x(5)); // [w,x,y,z]
        }

        void print() const{
            printf("[ekf_state] time == %d\n", time);
            printf("position == (%3f, %3f, %3f)\n", x(0),x(1),x(2));
            printf("pose == (%3f, %3f, %3f, %3f)\n", x(6), x(3), x(4), x(5));
        }
    };
    
}