//
// Created by jbs on 17. 8. 31.
//
#ifndef HEADER_H
#define HEADER_H

#include "prob_header.h"
#include "prob_workspace.h"

class agent{
public:
    // sensor angle and radius
    double sensor_ang,sensor_rad;
    //state varabile
    geometry_msgs::Pose2D state;
    //env parameters
    int Nx, Ny;
    // data histoty of observation
    sensor_map data;
    // probability map
    probablity_map prob_map;



    //constructor
    agent(double ,double , const pos& ,int ,int );
    //print agent_state
    void agent_state(){
        cout<<"[x,y,theta]=["<<state.x<<" , "<<state.y<<" , "<<state.theta<<"]"<<endl;
    }
    // data update w.r.t current state of agent
    void data_update(workspace& );
    // prob map update
    void prob_map_update();
    // control input
    cmd_vel following_control(workspace& ,double ,double );
    // prob_map print
    void prob_map_plot();

};




agent::agent(double ang, double rad, const pos &spawn_position, int Nx, int Ny) {
    //radian
    sensor_ang=ang;
    //radius
    sensor_rad=rad;
    state=spawn_position;
    //ROS_INFO_STREAM("agent is spawned at ["<<pos[0]<<" , "<<pos[1]<<"]");
    for(int i=1;i<=Nx;i++){
        for(int j=1;j<=Ny;j++){

            idx cur_idx(i,j);
            data.insert(pair<idx,sensor_data>(cur_idx,sensor_data(false,false)));
        }
    }
    // cell information
    this->Nx=Nx; this->Ny=Ny;


    //prob_map intialization

    probablity_map::iterator it=prob_map.begin();
    probablity_map::iterator it_end=prob_map.end();

    double init_prob=(double)1/(Nx*Ny);

    for(int i=1;i<=Nx;i++)
        for(int j=1;j<=Ny;j++){
            idx cur_idx(i,j);
            prob_map.insert(pair<idx,double>(cur_idx,init_prob));
        }
}


void agent::prob_map_plot(){
    for(int i=Ny;i>=1;i--)
        for(int j=1;j<=Nx;j++) {
            idx cur_idx(j, i);
            printf("%2.4f ",prob_map[cur_idx]);
            if(j==Nx)
                printf("\n");
        }
};


cmd_vel agent::following_control(workspace& ws,double Kv,double Ktheta){


    idx x_max(1,1);


    probablity_map::iterator it=prob_map.begin();
    probablity_map::iterator it_end=prob_map.end();


    while(it!=it_end){
        if((distance(ws.m[it->first],state)<3) && prob_map[it->first]>=prob_map[x_max]) {
            x_max = it->first;
        }
        it++;

    }
    cout<<"following point idx: [ "<<get<0>(x_max)<<" , "<<get<1>(x_max)<<"]"<<endl;
    pos x_max_pos=ws.m[x_max];

    cmd_vel cmd_vel1;
    cmd_vel1.linear_vel=Kv*distance(x_max_pos,state);
    double theta_d=atan3(x_max_pos.y-state.y,x_max_pos.x-state.x);
    cmd_vel1.ang_vel=Ktheta*(theta_d-state.theta);

    return cmd_vel1;
};

void agent::prob_map_update() {
    sensor_map::iterator it= data.begin();
    sensor_map::iterator it_end=data.end();
    double prob_sum=0.0; //probablity should be divided

    while(it!=it_end){
        bool isobserved=get<0>(it->second);
        bool  istarget=get<1>(it->second);
        idx cur_idx=it->first;
        if(isobserved)
            if(istarget)
                prob_map[it->first]=0.9;
            else
                prob_map[it->first]=0;
        prob_sum+=prob_map[it->first];

        it++;
    }

// make the sum of probability
    it= data.begin();
    while(it!=it_end) {
        prob_map[it->first] = prob_map[it->first] / prob_sum;
        it++;
    }

}

void agent::data_update(workspace& ws){
    pos_map::iterator it= ws.m.begin();
    pos_map::iterator it_end=ws.m.end();

    while(it!=it_end){

        idx cur_idx=it->first;
        pos cur_pos=it->second;
        double theta=atan3(cur_pos.y-state.y,cur_pos.x-state.x);
        sensor_data cur_sensor_data;
        // check whether that position in sensor boundary
        get<0>(cur_sensor_data)= sqrt(pow(cur_pos.x-state.x,2)+pow(cur_pos.y-state.y,2))<sensor_rad &&
                                 (theta>state.theta-sensor_ang/2 && theta<state.theta+sensor_ang/2);

        // check whether that position is actually target location
        get<1>(cur_sensor_data)=get<0>(cur_sensor_data)&&(cur_idx== ws.target_loc);
        double sibal=sqrt(pow(cur_pos.x-state.x,2)+pow(cur_pos.y-state.y,2));
        data[cur_idx]=cur_sensor_data;
        ++it;

    }
}







#endif // HEADER_H