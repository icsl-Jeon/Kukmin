#include <iostream>
#include <ros/ros.h>
#include "prob_agent.h"
#include <stdio.h>
// 연구 노트 p67

int main() {
    int Nx=4;
    int Ny=4;
    idx target_idx(3,3);
    double sensor_angle=PI/2;
    double sensor_radian=2;
    // spawn position
    pos spawn_pos;
    spawn_pos.x=0.5; spawn_pos.y=0.5; spawn_pos.theta=PI/4;
    workspace ws(Nx,Ny,target_idx,1);
    agent agent1(sensor_angle,sensor_radian,spawn_pos,Nx,Ny);
    agent1.data_renewal(ws);

    double dt=0.5;
    double threshold=0.5;
    //control test

    agent1.prob_map_plot();
    agent1.prob_map_renewal();
    printf("\n");
    agent1.prob_map_plot();
    cmd_vel cmd_vel1=agent1.following_control(ws,1,1);
    cout<<"linear: "<<cmd_vel1.linear_vel<<endl;
    cout<<"angular: "<<cmd_vel1.ang_vel<<endl;

    agent1.state.theta+=cmd_vel1.ang_vel*dt;

    agent1.state.x+=cmd_vel1.linear_vel*dt*cos(agent1.state.theta);
    agent1.state.y+=cmd_vel1.linear_vel*dt*sin(agent1.state.theta);

    agent1.agent_state();

/*
    for(int i=0;i<200;i++){
        agent1.prob_map_renewal();
        cmd_vel cmd_vel1=agent1.following_control(ws,2,1);


        agent1.state.theta+=cmd_vel1.ang_vel*dt;

        agent1.state.x+=cmd_vel1.linear_vel*dt*cos(agent1.state.theta);
        agent1.state.y+=cmd_vel1.linear_vel*dt*sin(agent1.state.theta);

    }

*/
    cout<<"position:[ "<<agent1.state.x<<" . "<<agent1.state.y<<" ]"<<endl;
    /*
    while(threshold>agent1.distance(ws)){

    }
    */







}