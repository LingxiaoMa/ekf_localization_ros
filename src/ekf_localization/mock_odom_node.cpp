#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_odom_node");
    ros::NodeHandle nh;

    ros::Publisher pub =
        nh.advertise<geometry_msgs::PoseStamped>("/odom_input", 20);

    ROS_INFO("[mock_odom_node] started!");

    ros::Rate rate(20);  // 20 Hz

    double true_x = 0.0;
    double true_y = 0.0;

    double odom_drift = 0.0;       // 累积漂移
    double drift_rate = 0.0002;    // 每次累 0.0002 m（越走越偏）

    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, 0.01);  // odom 小噪声

    while (ros::ok())
    {
        // --- Ground truth motion ---
        true_x += 0.05;  // 每步走0.05m

        // --- Add drift ---
        odom_drift += drift_rate;

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";

        // odom = 真值 + 漂移 + 小噪声
        msg.pose.position.x = true_x + odom_drift + noise(gen);
        msg.pose.position.y = true_y + noise(gen);
        msg.pose.position.z = 0;

        msg.pose.orientation.w = 1.0;

        pub.publish(msg);

        rate.sleep();
    }
}
