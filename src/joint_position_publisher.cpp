#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_position_publisher");

    ros::NodeHandle nh;

    ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint1_position_controller/command", 10);
    ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint2_position_controller/command", 10);
    ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint3_position_controller/command", 10);
    ros::Publisher joint4_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint4_position_controller/command", 10);
    ros::Publisher joint5_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint5_position_controller/command", 10);
    ros::Publisher joint6_pub = nh.advertise<std_msgs::Float64>("/rrr_arm/joint6_position_controller/command", 10);

    std_msgs::Float64 joint1_msgs;
    std_msgs::Float64 joint2_msgs;
    std_msgs::Float64 joint3_msgs;
    std_msgs::Float64 joint4_msgs;
    std_msgs::Float64 joint5_msgs;
    std_msgs::Float64 joint6_msgs;

    // joint1_msgs.data = 1.0;
    // joint2_msgs.data = 1.0;
    // joint3_msgs.data = 1.0;
    // joint4_msgs.data = 1.0;
    // joint5_msgs.data = 1.0;
    // joint6_msgs.data = 1.0;

    ros::Rate loop_rate(10);

    double time = 0.0;

    while (ros::ok()) {

        joint1_msgs.data = sin(time);
        joint2_msgs.data = sin(time + M_PI / 4);
        joint3_msgs.data = sin(time + M_PI / 2);
        joint4_msgs.data = sin(time + 3 * M_PI / 4);
        joint5_msgs.data = sin(time + M_PI);
        joint6_msgs.data = sin(time + 5 * M_PI / 4);

        joint1_pub.publish(joint1_msgs);
        joint2_pub.publish(joint2_msgs);
        joint3_pub.publish(joint3_msgs);
        joint4_pub.publish(joint4_msgs);
        joint5_pub.publish(joint5_msgs);
        joint6_pub.publish(joint6_msgs);

        time += 0.1;

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;

}