//
// Created by jbs on 17. 9. 3.
//

#ifndef PROG_AGENT_PROB_HEADER_H
#define PROG_AGENT_PROB_HEADER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <map>
#include <tuple>
#include <iterator>
#include <stdio.h>
#include <math.h>
#include "prob_agent.h"
#include "prob_workspace.h"

using namespace std;


typedef tuple<int,int> idx;
typedef geometry_msgs::Pose2D pos;
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
    d=sqrt(pow(pos1.x-pos2.x,2)+pow(pos1.y-pos2.y,2));
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







#endif //PROG_AGENT_PROB_HEADER_H
