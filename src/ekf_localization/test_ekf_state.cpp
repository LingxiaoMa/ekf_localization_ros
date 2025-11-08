#include "ekf_localization/ekf_state.hpp"

int main(){
    ekf_localization::ekf_state state;

    // 修改状态
    state.time = 1.23;
    state.x(0) = 1.0;  // x
    state.x(1) = 2.0;  // y
    state.x(2) = 3.0;  // z
    state.x(3) = 0.0;  // qx
    state.x(4) = 0.0;  // qy
    state.x(5) = 0.0;  // qz
    state.x(6) = 1.0;  // qw (单位旋转)

    state.print();
    return 0;
}