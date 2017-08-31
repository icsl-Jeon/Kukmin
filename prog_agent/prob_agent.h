//
// Created by jbs on 17. 8. 31.
//
#ifndef HEADER_H
#define HEADER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <map>
#include <tuple>
#include <iterator>
#include <math.h>

using namespace std;

#define PI 3.141592

typedef tuple<int,int> idx;
typedef geometry_msgs::Pose2D pos;
typedef bool isobserved; //did i see the cell?
typedef bool istarget; // if then does that cell have target?
typedef tuple<isobserved,istarget> sensor_data;

typedef map<idx,pos> pos_map;
typedef map<idx,sensor_data> sensor_map;


double atan3(double Y,double X){

    double theta=atan2(Y,X);
    if(theta<0)
        theta =2*PI+theta;
    return theta;

}


class workspace{

public:

    // total Nrow x Ncol cells with cell dim=ds
    double ds;
    int Nx, Ny;
    idx target_loc;
    pos_map m;
    workspace(int nx,int ny,const idx& target_idx,double ds){
        Ny=ny; Nx=nx;
        this->ds=ds;
        // location will be given
        get<0>(target_loc)=get<0>(target_idx);
        get<1>(target_loc)=get<1>(target_idx);

        for(int i=1;i<=Nx;i++)
            for(int j=1;j<=Ny;j++){
                //pos cur_pos(ds/2+ds*(i-1),ds/2+ds*(j-1));
                pos cur_pos;
                cur_pos.x=ds/2+ds*(i-1); cur_pos.y=ds/2+ds*(j-1);
                idx cur_idx(i,j);
                m.insert(pair<idx,pos>(cur_idx,cur_pos));
            }
        }
};

class agent{
private:
    // sensor angle and radius
    double sensor_ang,sensor_rad;
    //state varabile
    geometry_msgs::Pose2D state;
    int Nx, Ny;
public:
    sensor_map data;

    agent(double ang,double rad, const pos& spawn_position,int Nx,int Ny){
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
    }

    void data_renewal(workspace ws){
        pos_map::iterator it= ws.m.begin();
        pos_map::iterator it_end=ws.m.end();

        while(it!=it_end){
            idx cur_idx=it->first;
            pos cur_pos=it->second;
            double theta=atan3(cur_pos.y-state.y,cur_pos.x-state.x);
            sensor_data cur_sensor_data;
            // check whether that position in sensor boundary
            get<0>(cur_sensor_data)= sqrt(pow(cur_pos.x-state.x,2)+pow(cur_pos.y-state.y,2))<sensor_rad ||
                    (theta>state.theta-sensor_ang/2 || theta<state.theta+sensor_ang/2);

            // check whether that position is actually target location
            get<1>(cur_sensor_data)=get<0>(cur_sensor_data)||(cur_idx== ws.target_loc);

            data[cur_idx]=cur_sensor_data;


        }

    }
};


#endif // HEADER_H