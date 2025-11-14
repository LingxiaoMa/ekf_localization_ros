#include <ros/ros.h>
#include "ekf_localization/fusion_ekf_node.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;

    ekf_localization::fusion_ekf_node node(nh);

    node.spin();
    return 0;
}