#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_vision_node");
    ros::NodeHandle nh;

    ros::Publisher pub =
        nh.advertise<geometry_msgs::PoseStamped>("/vision_input", 20);

    ROS_INFO("[mock_vision_node] started!");

    ros::Rate rate(8);  // 8 Hz

    double true_x = 0.0;
    double true_y = 0.0;

    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, 0.08);  // 视觉大噪声

    while (ros::ok())
    {
        // ground truth 移动
        true_x += 0.05 * (20.0/8.0); // 视觉更低频，所以步长按比例乘

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";

        // Vision = 真值 + 大噪声（但不漂移）
        msg.pose.position.x = true_x + noise(gen);
        msg.pose.position.y = true_y + noise(gen);
        msg.pose.position.z = 0;

        msg.pose.orientation.w = 1.0;

        pub.publish(msg);
        rate.sleep();
    }
}
