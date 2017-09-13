#include "prob_header.h"




int main(int argc,char** argv)
{
    ros::init(argc, argv, "PID_control");
    ros::Time::init();
    ros::Rate rate(2);
    PID_controller contorller;
    geometry_msgs::Twist cmd_msg;
    while (ros::ok()){

        cmd_msg=contorller.Control();
        contorller.vel_pub.publish(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
