#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "ekf_localization/pose_localization.hpp"
#include <ekf_localization/controlInput.h>

namespace ekf_localization
{
    class fusion_ekf_node{
        public:
        fusion_ekf_node(ros::NodeHandle& nh);
        void spin();

        private:
        void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void visionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void publishEkfPose(double time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& q);

        private:
        ros::NodeHandle nh;

        ros::Subscriber control_sub_;
        ros::Subscriber vision_sub_;

        ros::Publisher ekf_pose_pub_;

        pose_localization ekf_;
        double last_publish_time;
    };

} // namespace ekf_localization

