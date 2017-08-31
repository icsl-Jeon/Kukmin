#include <iostream>
#include <ros/ros.h>
#include "prob_agent.h"


int main() {
    int Nx=4;
    int Ny=4;
    idx target_idx(3,3);
    double sensor_angle=PI/2;
    double sensor_radian=2;
    pos spawn_pos;
    spawn_pos.x=1.5; spawn_pos.y=1.5; spawn_pos.theta=PI/4;
    workspace ws(Nx,Ny,target_idx,1);
    agent agent1(sensor_angle,sensor_radian,spawn_pos,Nx,Ny);
    agent1.data_renewal(ws);
    //cout<<get<0>(agent1.data[target_idx])<<end;
    cout<<"hello"<<endl;
}