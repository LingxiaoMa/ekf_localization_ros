#pragma once 
#include <Eigen/Dense>
#include <iostream>

namespace ekf_localization
{
    struct ekf_control
    {
        /* data */
        double time;
        Eigen::VectorXd u;

        ekf_control(){
            this->time = 0.0;
            this->u = Eigen::VectorXd::Zero(7);
            u(6) = 1;
        }

        // return position
        Eigen::Vector3d position() const{
            return Eigen::Vector3d(u(0),u(1),u(2));
        }

        // return quaternion
        Eigen::Quaterniond pose() const{
            return Eigen::Quaterniond(u(6), u(3), u(4), u(5)); // [w,x,y,z]
        }

        void print() const{
            printf("[ekf_control] time == %d\n", time);
            printf("position == (%3f, %3f, %3f)\n", u(0),u(1),u(2));
            printf("pose == (%3f, %3f, %3f, %3f)\n", u(6), u(3), u(4), u(5));
        }
    };
    
} // namespace ekf_localization
