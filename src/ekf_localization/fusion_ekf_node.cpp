#include "ekf_localization/fusion_ekf_node.hpp"
#include <Eigen/Geometry>

namespace ekf_localization{
    fusion_ekf_node::fusion_ekf_node(ros::NodeHandle& nh_) : nh(nh_), last_publish_time(0.0)
    {
        control_sub_ = nh.subscribe("/odom_input", 20, 
        &fusion_ekf_node::odomCallback, this);

        vision_sub_ = nh.subscribe("/vision_input", 20,
        &fusion_ekf_node::visionCallback, this);

        ekf_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/fusion_ekf_pose", 20);

        ROS_INFO("[fusion_ekf_node] started!");

    }

    void fusion_ekf_node::odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double t = msg->header.stamp.toSec();

        Eigen::Vector3d pos(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);

        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);

        ekf_.addOdomPose(t, q, pos);
    }

    void fusion_ekf_node::visionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        double t = msg->header.stamp.toSec();

        Eigen::Vector3d pos(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );

        ekf_.addVisionPose(t, q, pos);

        auto state = ekf_.getState();

        publishEkfPose(state.time, state.position(), state.pose());
    }

    void fusion_ekf_node::publishEkfPose(double time, const Eigen::Vector3d& pos, const Eigen::Quaterniond& q){
        geometry_msgs::PoseStamped out;
        out.header.frame_id = "map";
        out.header.stamp = ros::Time(time);

        out.pose.position.x = pos.x();
        out.pose.position.y = pos.y();
        out.pose.position.z = pos.z();

        out.pose.orientation.w = q.w();
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();

        ekf_pose_pub_.publish(out);
    }


    void fusion_ekf_node::spin()
{
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
}