//
// Created by jbs on 17. 9. 3.
//

#ifndef PROG_AGENT_PROB_HEADER_H
#define PROG_AGENT_PROB_HEADER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

using namespace std;


#define PI 3.141592


double atan3(double Y,double X){

    double theta=atan2(Y,X);
    if(theta<0)
        theta =2*PI+theta;
    return theta;
}

double distance(geometry_msgs::Pose pos1,geometry_msgs::Pose pos2){
    double d;
    d=sqrt(pow(pos1.position.x-pos2.position.x,2)+pow(pos1.position.y-pos2.position.y,2));
   
    return d;
}



struct cmd_vel{
public:
    double linear_vel;
    double ang_vel;

    void set(double l_vel,double a_vel){
        linear_vel=l_vel;
        ang_vel=a_vel;

    }
};



class PID_controller {
private:

    geometry_msgs::Pose cur_pos;
    geometry_msgs::Pose target_pos;

    double Kv, Ktheta;



public:
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist msg;	


    // callback function
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        cur_pos.position.x = msg->pose.pose.position.x;
        cur_pos.position.y = msg->pose.pose.position.y;
    }


    // constructor
    PID_controller() {
        ros::NodeHandle nh_pub("out");
        ros::NodeHandle nh_sub("in");

        vel_pub = nh_pub.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
        // if it doesn't work then
        //vel_pub=nh_pub.advertise<geometry_msgs::Twist_>("/mobile_base/commands/velocity ",1000);

        odom_sub = nh_sub.subscribe("/odom", 1000, &PID_controller::OdomCallback,this);


        Kv = 0.1;
        Ktheta = 0.1;

        nh_pub.setParam("/Kv", Kv);
        nh_pub.setParam("/Ktheta", Ktheta);
	
        target_pos.position.x = 4.0;
        target_pos.position.y = 4.0;

    }

    //control input


    geometry_msgs::Twist Control() {
	double d=sqrt(pow(cur_pos.position.x-target_pos.position.x,2)+pow(cur_pos.position.y-target_pos.position.y,2));

   	if(Kv*d<2)
        msg.linear.x = Kv * d;
	else
	msg.linear.x=2;
	

        double theta_d = atan2(target_pos.position.y - cur_pos.position.y, target_pos.position.x - cur_pos.position.x);
	

        msg.angular.z = Ktheta * (theta_d - cur_pos.orientation.z);
	if(msg.angular.z>0.6)
	msg.angular.z=0.6;

	return msg;

    }


};





#endif //PROG_AGENT_PROB_HEADER_H
