#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

nav_msgs::Path path;
ros::Publisher path_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    path.header = msg->header;
    path.poses.push_back(*msg);

    // 限制轨迹长度（防止 RViz 卡）
    if (path.poses.size() > 2000)
        path.poses.erase(path.poses.begin());

    path_pub.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_to_path_node");
    ros::NodeHandle nh("~");

    std::string pose_topic, path_topic;
    nh.param<std::string>("pose_topic", pose_topic, "/fusion_ekf_pose");
    nh.param<std::string>("path_topic", path_topic, "/fusion_ekf_path");

    ros::Subscriber sub = nh.subscribe(pose_topic, 20, poseCallback);
    path_pub = nh.advertise<nav_msgs::Path>(path_topic, 20);

    ros::spin();
    return 0;
}
