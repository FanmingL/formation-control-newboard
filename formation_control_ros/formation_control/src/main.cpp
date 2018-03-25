#include <iostream>
#include "ros/ros.h"
#include "uav_msgs/uav_position.h"
#include "uav_msgs/uav_control.h"
#include "uav_msgs/uav_information.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

ros::Publisher 	lock1,lock2,lock3,
		unlock1,unlock2,unlock3,
		thrcontrol,
		mode1,mode2,mode3,
		allstatus;

ros::Subscriber	arinfo1,arinfo2,arinfo3,
		allunlock,alllock,alltakeoff,allmode,alllandon;	

ros::ServiceServer 	allunlock_service,alllock_service,
			alltakeoff_service,allmode_service,alllandon_service;

void arinfo1_callback(const uav_msgs::uav_information &msg);
void arinfo2_callback(const uav_msgs::uav_information &msg);
void arinfo3_callback(const uav_msgs::uav_information &msg);
void allunlock_callback(const std_msgs::Empty &msg);
void alllock_callback(const std_msgs::Empty &msg);
void alltakeoff_callback(const std_msgs::Empty &msg);
void alllandon_callback(const std_msgs::Empty &msg);
void allmode_callback(const std_msgs::UInt8 &msg);
void all_lock(void);
void all_unlock(void);
void all_takeoff(void);		
void all_landon(void);		
void all_mode(int exp_mode);
bool allunlock_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
bool alllock_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
bool alltakeoff_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
bool alllandon_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
bool allmode_ser(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res);
void Check(void);
void TimeInit(void);

int OnLineFlag=0;

int main(int argc,char** argv){
	ros::init(argc,argv,"formation_control");
	ros::NodeHandle n;		
	TimeInit(); 
	lock1	=	n.advertise<std_msgs::Empty>("/odroid1/lock",1);
	lock2	=	n.advertise<std_msgs::Empty>("/odroid2/lock",1);
	lock3	=	n.advertise<std_msgs::Empty>("/odroid3/lock",1);
	unlock1	=	n.advertise<std_msgs::Empty>("/odroid1/unlock",1);
	unlock2	=	n.advertise<std_msgs::Empty>("/odroid2/unlock",1);
	unlock3	=	n.advertise<std_msgs::Empty>("/odroid3/unlock",1);
	mode1	=	n.advertise<std_msgs::UInt8>("/odroid1/mode",1);
	mode2	=	n.advertise<std_msgs::UInt8>("/odroid2/mode",1);
	mode3	=	n.advertise<std_msgs::UInt8>("/odroid3/mode",1);
	allstatus=	n.advertise<std_msgs::UInt8>("/allstatus",1);
	thrcontrol=	n.advertise<uav_msgs::uav_control>("/allthr",1);
	
	arinfo1	=	n.subscribe("/odroid1/arinfo",1,&arinfo1_callback);
	arinfo2	=	n.subscribe("/odroid2/arinfo",1,&arinfo2_callback);
	arinfo3	=	n.subscribe("/odroid3/arinfo",1,&arinfo3_callback);

	allunlock=	n.subscribe("/allunlock",1,&allunlock_callback);
	alllock	=	n.subscribe("/alllock",1,&alllock_callback);
	alltakeoff=	n.subscribe("/alltakeup",1,&alltakeoff_callback);
	allmode	=	n.subscribe("/allmode",1,&allmode_callback);
	alllandon=	n.subscribe("/alllandon",1,&alllandon_callback);
	
	allunlock_service=n.advertiseService("/allunlock",&allunlock_ser);
	alllock_service=n.advertiseService("/alllock",&alllock_ser);
	allmode_service=n.advertiseService("/allmode",&allmode_ser);
	alltakeoff_service=n.advertiseService("/alltakeoff",&alltakeoff_ser);
	alllandon_service=n.advertiseService("/alllandon",&alllandon_ser);
	
	ros::Rate loop_rate(100);
	usleep(1200000);
	std::cout<<"Init Done\n";
	while (ros::ok())
	{
		Check();
		ros::spinOnce();
		loop_rate.sleep();
	
	}

	return 0;
}
const int ArNumber=2;
ros::Time InfoTimes[ArNumber];

void TimeInit(void) 
{
	for (int i=0;i<ArNumber;i++)
	{
		InfoTimes[i]=ros::Time::now();	
	}
}

static float GetTimeLength(int TimeIndex)
{
	return ((ros::Time::now()-InfoTimes[TimeIndex]).toSec());
}

void Check(void)
{
	static int XTimes=0;
	XTimes++;
	if ((XTimes%5)==0)
	{
		XTimes=0;
		return;
	}
	for (int i=0;i<ArNumber;i++)
	{
		float time_gap=GetTimeLength(i);
		if (time_gap>1)
		{
//			std::cout<<"odroid"<<i+1<<" is offline\n";
			OnLineFlag &= (~(0x01<<i));
		}
		else 
		{
			OnLineFlag |= (0x01<<i);			
		}
	}
	std_msgs::UInt8 status_msg;
	status_msg.data=OnLineFlag;
	allstatus.publish(status_msg);
}

void arinfo1_callback(const uav_msgs::uav_information &msg)
{
	InfoTimes[0]=msg.header.stamp;

	static int lock_last=1;
	if (lock_last==0&&msg.lock==1)
	{			
		all_lock();	
	}
	if (lock_last==1&&msg.lock==0)
	{
		all_unlock();
	}
	lock_last=msg.lock;
}

void arinfo2_callback(const uav_msgs::uav_information &msg)
{

	InfoTimes[1]=msg.header.stamp;
	static int lock_last=1;
	if (lock_last==0&&msg.lock==1)
	{			
		all_lock();	
	}
	if (lock_last==1&&msg.lock==0)
	{
		all_unlock();
	}
	lock_last=msg.lock;

}

void arinfo3_callback(const uav_msgs::uav_information &msg)
{

	InfoTimes[2]=msg.header.stamp;
	static int lock_last=1;
	if (lock_last==0&&msg.lock==1)
	{			
		all_lock();	
	}
	if (lock_last==1&&msg.lock==0)
	{
		all_unlock();
	}
	lock_last=msg.lock;

}

void allunlock_callback(const std_msgs::Empty &msg)
{
	all_unlock();
}

void alllock_callback(const std_msgs::Empty &msg)
{
	all_lock();
}

void alltakeoff_callback(const std_msgs::Empty &msg)
{
	all_takeoff();

}

void alllandon_callback(const std_msgs::Empty &msg)
{
	all_landon();
}

void allmode_callback(const std_msgs::UInt8 &msg)
{
	all_mode(msg.data);
}


bool alllock_ser(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res)
{
	all_landon();
	all_lock();
	return true;
}


bool allunlock_ser(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res)
{
	all_unlock();
	return true;
}

bool allmode_ser(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res)
{
	if (req.data)all_mode(3);
	else all_mode(1);
	return true;
}

bool alltakeoff_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
	all_takeoff();
	return true;
}

bool alllandon_ser(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
	all_landon();
	return true;
}

void all_unlock(void)
{
	std_msgs::Empty unlock_msg;
	unlock1.publish(unlock_msg);
	unlock2.publish(unlock_msg);
	unlock3.publish(unlock_msg);
	std::cout<<"Unlock Done.\n";
}

void all_lock(void)
{
	std_msgs::Empty lock_msg;
	lock1.publish(lock_msg);
	lock2.publish(lock_msg);
	lock3.publish(lock_msg);
	std::cout<<"Lock Done.\n";
}

void all_takeoff(void)
{
	uav_msgs::uav_control con;
	con.thr=1700;
	thrcontrol.publish(con);
	std::cout<<"take off Done.\n";
}

void all_mode(int exp_mode)
{
	std_msgs::UInt8 mode_msg;
	mode_msg.data=exp_mode;
	mode1.publish(mode_msg);
	mode2.publish(mode_msg);
	mode3.publish(mode_msg);
	std::cout<<"mode "<<exp_mode<<"  Done.\n";
}

void all_landon(void)
{
	uav_msgs::uav_control con;
	con.thr=1400;
	thrcontrol.publish(con);
	std::cout<<"land on Done.\n";


}


