#include "ekf_localization/ekf_control.hpp"

int main(){
    ekf_localization::ekf_control control;

    // 修改状态
    control.time = 1.23;
    control.u(0) = 3.0;  // u
    control.u(1) = 2.0;  // y
    control.u(2) = 1.0;  // z
    control.u(3) = 0.0;  // qu
    control.u(4) = 0.0;  // qy
    control.u(5) = 0.0;  // qz
    control.u(6) = 1.0;  // qw (单位旋转)

    control.print();
    return 0;
}