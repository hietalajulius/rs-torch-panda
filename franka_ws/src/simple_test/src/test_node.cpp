#include<ros/ros.h>
#include<std_msgs/Int16.h>
#include<chrono>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;

    ROS_INFO("Node started");
    ros::Time start = ros::Time::now();
    ros::Duration elapsed_time_ = ros::Duration(0.0);

    ros::Publisher pub = nh.advertise<std_msgs::Int16>("/positions", 10);

    ros::Rate rate(1000);
    int i = 0;
    while (ros::ok()) {
        i += 1;
        std_msgs::Int16 msg;
        msg.data = i;
        pub.publish(msg);
        rate.sleep();
    }

    ROS_INFO("Exit");
}