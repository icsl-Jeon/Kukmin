#include <iostream>
#include <ros/ros.h>
#include "prob_agent.h"
#include <stdio.h>
// 연구 노트 p67

int main() {
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


    double dt=0.5;
    double threshold=0.5;
    //control test


    for(int i=0;i<100;i++){
        agent1.data_update(ws);
        agent1.prob_map_update();
        agent1.prob_map_plot();
        cmd_vel cmd_vel1=agent1.following_control(ws,1,1);


        agent1.state.theta+=cmd_vel1.ang_vel*dt;

        agent1.state.x+=cmd_vel1.linear_vel*dt*cos(agent1.state.theta);
        agent1.state.y+=cmd_vel1.linear_vel*dt*sin(agent1.state.theta);
        printf("\n");
        agent1.agent_state();


    }



}