#include<vector>
#include<cmath>
#include<cstdio>
#include<iostream>
#include "traj.h"

using namespace std;

/*
 * 功能：编队支持函数
 * 日期：18/3/24
 * 作者：ycnalin
 * 所有输入输出单位均是以“MM”为基础
 * 如长度为MM，速度为MM/s
 */
static float position_x[25], position_y[25];
static int16_t fill_cnt_x, fill_cnt_y;
static float vec = 0.0f;

static bool startflag = true;	//初始化标志
static bool endflag = false;	//抵达终点标志
static int point_count=0;	//记录经过目标点个数

static float deg2;
static Target_t target = {0,0,0,0,0,0};

static Point_t point1={400,500},point2={2400,500},point3=point1;//,point4={2200,500},point5=point1;
static Pos_t stablePos,oldPos,currentPos;
const static int point_num = 3;
static vector<Point_t >trajectory;


float calc_dist1(void)
{
	Point_t st, ed, a;
	a.x = stablePos.x, a.y = stablePos.y;
	st = trajectory[point_count - 1];
	ed = trajectory[point_count];

	vec = (ed.x - st.x) * (a.x - st.x) + (ed.y - st.y)*(a.y - st.y);
	vec = vec / sqrt(mypow((ed.x - st.x), 2) + mypow((ed.y - st.y), 2));
	if (abs(st.x - ed.x) < 0.0001) return abs(st.x - a.x);
	float k = (st.y - ed.y) / (st.x - ed.x);
	return abs(k * a.x - a.y + st.y - k*st.x) / sqrt(k*k + 1);
}

void init1(void)
{
	trajectory.push_back(point1);

	trajectory.push_back(point2);
	trajectory.push_back(point3);
//	trajectory.push_back(point4);
//	trajectory.push_back(point5);
//	trajectory.push_back(point6);

	//起点直达！！！！！
	target.x1 = trajectory[0].x;
	target.y1 = trajectory[0].y;
	point_count = 0; //the first target;

	if (!(point_num == 1))	//计算yaw角度
	{
		float temp = atan2((trajectory[1].y - trajectory[0].y), (trajectory[1].x - trajectory[0].x));
		temp = round(To_180_degrees(temp*rad2degree + 90.0));
		target.yaw1 = temp;
	}
	else
	{
		target.yaw1 = 0;
		return;
	}
	target.x2 = trajectory[0].x;
	target.y2 = trajectory[0].y;
	target.yaw2 = target.yaw1;

	startflag = false;
}

void Calc_Pos1(float interval, float coordinate_x, float coordinate_y, float yaw)
{
	oldPos = stablePos;
	Moving_Average(position_x, 5, &fill_cnt_x, coordinate_x, &currentPos.x);
	Moving_Average(position_y, 5, &fill_cnt_y, coordinate_y, &currentPos.y);
	LPF_1(5, interval, currentPos.x,stablePos.x);
	LPF_1(5, interval, currentPos.y,stablePos.y);
	currentPos.x_sp = 0.80*(stablePos.x - oldPos.x) / interval + 0.20*oldPos.x_sp;// cm/s
	currentPos.y_sp = 0.80*(stablePos.y - oldPos.y) / interval + 0.20*oldPos.y_sp;
	LPF_1(20, interval, currentPos.x_sp, stablePos.x_sp);
	LPF_1(20, interval, currentPos.y_sp, stablePos.y_sp);
	stablePos.yaw = 0.80*yaw*rad2degree + 0.20*stablePos.yaw;
	stablePos.yaw = To_180_degrees(yaw);
	stablePos.dist = sqrt((stablePos.x - target.x1)*(stablePos.x - target.x1) + (stablePos.y - target.y1)*(stablePos.y - target.y1));
}
void change_target1(void)
{
	float div = stablePos.dist;
	if (point_count > 0 && point_count < point_num) div = calc_dist1();
	
	if (div > 70) return;
	if (stablePos.x_sp > 500 || stablePos.y_sp > 500) return;
//	if (absint(stablePos.yaw - target.yaw1) > 90) return;
	//*******************************************************
	float temp = sqrt(mypow(trajectory[point_count].x-stablePos.x,2)+mypow(trajectory[point_count].y - stablePos.y, 2));
	if (temp>100&&temp < Max_Step) //距离目标一步之遥
	{
		target.x1 = trajectory[point_count].x;
		target.y1 = trajectory[point_count].y;
	}
	else if (temp<100)	//抵达一个目标点
	{
		target.x1 = trajectory[point_count].x;	//调节yaw，x y 不变
		target.y1 = trajectory[point_count].y;
		if (++point_count == point_num) {
			endflag = true;
			target.yaw1 = 0;
			return;
		}
	//	float deg = atan2((trajectory[point_count].y - trajectory[point_count -1].y), (trajectory[point_count].x - trajectory[point_count - 1].x));
	//	deg = round(To_180_degrees(deg*rad2degree + 90.0));
	//	target.yaw1 = deg;
		
	}
	else if (point_count>0 && div<70)
	{
		float deg = atan2((trajectory[point_count].y - trajectory[point_count - 1].y), (trajectory[point_count].x - trajectory[point_count - 1].x));
		deg += pi / 2.0;
		deg = deg > pi ? (deg - pi*2.0) : deg;

		target.x1 = trajectory[point_count - 1].x + sin(deg) * (vec + Max_Step);
		target.y1 = trajectory[point_count - 1].y - cos(deg) * (vec + Max_Step);
		deg2 = deg;
		/*		target.x1 += sin(deg)*Max_Step;
		target.y1 -= cos(deg)*Max_Step;*/
	}
}

Target_t trajectory_generate1(float interval, float coordinate_x, float coordinate_y, float yaw, Pos_t &stablePos_)
{
	if (startflag) init1();
    target.x2 = target.x1;
    target.y2 = target.y1;
    
	Calc_Pos1(interval, coordinate_x, coordinate_y, yaw);

	if(!endflag&&point_count<point_num)
		change_target1();
	if (endflag)
	{
		target.end_flag=1;
	}else
	{
		target.end_flag=0;
	}
    stablePos_ = stablePos;
	return target;
}

float mypow(float x,int c)
{
	int i = 0;
	float temp=1.0;
	for (; i < c; i++)
	{
		temp *= x;
	}
	return temp;
}


void Moving_Average(float movearray[], int16_t len, int16_t *fil_cnt, float in, float *out)
{
	int16_t width_num = len;
	float last;

	if (++*fil_cnt >= width_num)
		*fil_cnt = 0; //now

	last = movearray[*fil_cnt];
	movearray[*fil_cnt] = in;
	*out += (in - (last)) / (float)(width_num);
}

float To_180_degrees(float x)
{
	return (x>180 ? (x - 360) : (x<-180 ? (x + 360) : x));
}

float limit(float x,float min, float max)
{
	if (x < min)
		return min;
	if (x > max)
		return max;
	return x;
}

float absint(float x)
{
	if (x > 0)
		return x;
	else
		return -x;
}
