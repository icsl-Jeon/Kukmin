//
// Created by jbs on 17. 9. 3.
//
#include <iostream>
#include <ros/ros.h>
#include "prob_header.h"
#include <stdio.h>



int main(int argc,char **argv){
    ros::init(argc,argv,"prob_agent");

    ros::Publisher state;

    int Nx=4;
    int Ny=4;
    idx target_idx(3,3);
    double sensor_angle=PI/1.5;
    double sensor_radian=2;
    // spawn position
    pos spawn_pos;
    spawn_pos.x=1; spawn_pos.y=0.5; spawn_pos.theta=PI/4;
    workspace ws(Nx,Ny,target_idx,1);
    agent agent1(sensor_angle,sensor_radian,spawn_pos,Nx,Ny);












}
