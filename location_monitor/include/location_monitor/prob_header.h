//
// Created by jbs on 17. 9. 3.
//

#ifndef PROG_AGENT_PROB_HEADER_H
#define PROG_AGENT_PROB_HEADER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <tuple>
#include <iterator>
#include <stdio.h>
#include <math.h>


using namespace std;


typedef tuple<int,int> idx;
typedef geometry_msgs::Pose pos;
typedef tuple<bool,bool> sensor_data;

typedef map<idx,pos> pos_map;
typedef map<idx,sensor_data> sensor_map;
typedef map<idx,double> probablity_map;

#define PI 3.141592


double atan3(double Y,double X){

    double theta=atan2(Y,X);
    if(theta<0)
        theta =2*PI+theta;
    return theta;
}

double distance(pos pos1,pos pos2){
    double d;
    d=sqrt(pow(pos1.position.x-pos2.position.x,2)+pow(pos1.position.y-pos2.position.y,2));
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
    geometry_msgs::Twist msg;
    double Kv, Ktheta;


    // callback function
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        cur_pos.position.x = msg->pose.pose.position.x;
        cur_pos.position.y = msg->pose.pose.position.y;
    }


public:
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;



    // constructor
    PID_controller() {
        ros::NodeHandle nh_pub("out");
        ros::NodeHandle nh_sub("in");

        vel_pub = nh_pub.advertise<geometry_msgs::Twist_>("/cmd_vel_mux/input/navi", 1000);
        // if it doesn't work then
        //vel_pub=nh_pub.advertise<geometry_msgs::Twist_>("/mobile_base/commands/velocity ",1000);

        odom_sub = nh_sub.subscribe("/odom", 10, OdomCallback);


        Kv = 0.5;
        Ktheta = 0.1;

        nh_pub.getParam("Kv", Kv);
        nh_pub.getParam("Ktheta", Ktheta);


        target_pos.position.x = 4.0;
        target_pos.position.y = 4.0;

    }

    //control input
    geometry_msgs::Twist Control() {
        msg.linear.x = Kv * distance(target_pos, cur_pos);
        double theta_d = atan3(target_pos.position.y - cur_pos.position.y, target_pos.position.x - cur_pos.position.x);
        msg.angular.z = Ktheta * (theta_d - cur_pos.orientation.z);

    }


};





#endif //PROG_AGENT_PROB_HEADER_H
