//
// Created by jbs on 9/1/17.
//

#ifndef PROG_AGENT_FUNCTIONS_H
#define PROG_AGENT_FUNCTIONS_H

#include <math.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.141592
typedef geometry_msgs::Pose2D pos;

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


#endif //PROG_AGENT_FUNCTIONS_H
