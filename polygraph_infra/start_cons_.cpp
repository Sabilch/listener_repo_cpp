#include "ros/ros.h"
#include <std_msgs/String.h>


void writeMsgToLog(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Hearing:[%s] ", msg->data.c_str());
    ros::Rate rate(1);
    while (ros::ok())
    {
    ros::spinOnce();
    rate.sleep();
    }
}


int main (int args, char **argv) {

    ros::init(args, argv, "start_cons_");

    ros::NodeHandle nh;

    ros::Subscriber topic_cons = nh.subscribe("/prod_out", 1 ,writeMsgToLog);

    ros::spin();

    return 0;
}