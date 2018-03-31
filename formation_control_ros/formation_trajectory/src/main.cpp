/*************************************************************************
    > File Name: main.cpp
    > Author: ma6174
    > Mail: ma6174@163.com 
    > Created Time: Wed 12 Apr 2017 01:20:34 PM UTC
 ************************************************************************/

#include<iostream>
#include"ros/ros.h"
#include "april_pro/camera_pos.h"
#include "std_msgs/Empty.h"
#include "traj.h"
#include <fstream>

#define int16_t short
int debug_flag_ = 1;
ros::Time old1, old2;
april_pro::camera_pos pos_get1, pos_get2, pos_out1, pos_out2;
bool fresh_flag1=false, fresh_flag2=false;
ros::Publisher path1,path2,alllock,alllandon;
Target_t target_out1, target_out2;
Pos_t stable1,stable2;
std_msgs::Empty empty;
void position1_handle(const april_pro::camera_pos &msg)
{
    float interval = (msg.header.stamp-old1).toSec();
    if (interval>1.0f){
        old1 = msg.header.stamp;
        return;
    }
    old1 =msg.header.stamp;
    pos_get1 = msg;
    fresh_flag1 = true;
    target_out1 = trajectory_generate1(interval,10*pos_get1.x,10*pos_get1.y,pos_get1.yaw,stable1);
}

void position2_handle(const april_pro::camera_pos &msg)
{
    float interval = (msg.header.stamp-old2).toSec();
    if (interval>1.0f){
        old2 = msg.header.stamp;
        return;
    }
    old2 =msg.header.stamp;
    pos_get2 = msg;
    fresh_flag2 = true;
    target_out2 = trajectory_generate2(interval,10*pos_get2.x,10*pos_get2.y,pos_get2.yaw,stable2);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "formation_trajectory");
	ros::NodeHandle n;
	old1 = ros::Time::now();
    old2 = ros::Time::now();
	path1 = n.advertise<april_pro::camera_pos>("/odroid1/target",1000);
    path2 = n.advertise<april_pro::camera_pos>("/odroid2/target",1000);
    alllock   =    n.advertise<std_msgs::Empty>("/alllock",1);
    alllandon =    n.advertise<std_msgs::Empty>("/alllandon",1);
	ros::Subscriber sub1 = n.subscribe("/odroid1/position",1,&position1_handle);
	ros::Subscriber sub2 = n.subscribe("/odroid2/position",1,&position2_handle);
    static int tick=0;
    ros::Rate loop_rate(100);
    std::ofstream outfile;
    std::string data_path_out = "trajectory_real";
    data_path_out  += ".csv";
    outfile.open(data_path_out.c_str());
    if(!outfile) std::cout<<"error"<<std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
        if (fresh_flag1&&fresh_flag2)
        {
                       int error_status = getFormationTarget(stable1, stable2, &target_out1, &target_out2);
             if (debug_flag_)
	    {
            outfile<<pos_get1.x<<","<<pos_get1.y<<","
                   <<pos_get2.x<<","<<pos_get2.y<<"\n";
            } 

 	tick++;
	if (tick < 20)
	{
		if ((pos_get1.x-pos_get2.x)*(pos_get1.x-pos_get2.x) + ((pos_get1.y-pos_get2.y)*(pos_get1.y-pos_get2.y))<(64 + 15) * (64 + 15))
		{
			error_status = -1;
		}
		else
		{
			error_status = 1;
		}
		 

	}
	
	//	 int    error_status = 1;  
	  if (error_status == -1)
            {
                alllock.publish(empty);
	//	std::cout<<"postion1 is "<<stable1.x<<",  "<<stable1.y<<std::endl;
	//	std::cout<<"positon2 is "<<stable2.x<<",  "<<stable2.y<<std::endl;
                continue;
            }
            if (error_status == 0)
            {
                alllandon.publish(empty);
                continue;
            }
	
            pos_out1.x = target_out1.x1;
            pos_out1.y = target_out1.y1;
            pos_out1.yaw = target_out1.yaw1;
            pos_out1.id = target_out1.end_flag;
            pos_out2.x = target_out2.x1;
            pos_out2.y = target_out2.y1;
            pos_out2.yaw = target_out2.yaw1;
            pos_out2.id = target_out2.end_flag;
            path1.publish(pos_out1);
            path2.publish(pos_out2);
            fresh_flag1 = false;
            fresh_flag2 = false;
            //
		if (target_out1.end_flag&&target_out2.end_flag)
		{
			alllandon.publish(empty);
		}
            
        }
        loop_rate.sleep();
    }
    outfile.close();


	return 0;
}


