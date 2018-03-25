/*************************************************************************
	> File Name: taj.h
	> Author: LuoFanming 
	> Mail: 151190065@smail.nju.edu.cn
	> Created Time: 2018/03/24
 ************************************************************************/

#ifndef _TAJ_H
#define _TAJ_H
#include <cmath>
#include <iostream>
// 飞行器半径
const int Radius = 320 * 2;
// 安全距离30
const int SafeDist = 300;
// 迫降距离
const int LandingDist = 150;
// 停机距离
const int StopDist = 100;

const float pi = 3.1415926535898f;
/******************************编队参数******************************/
//注：此为平行飞行队形
const int FlyDist = 1300;
// 弧度制，主机theta,从机则为pi-theta,
const float Theta = -pi/2;
/******************************Ending******************************/

const float rad2degree = 180.0 / 3.141592653;
const int Max_Step = 400;    //最大步长，单位毫米
#define LPF_1(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))
#define int16_t short
#define abs(x) ( (x) < 0 ? (-(x)) : (x)  )
typedef struct
{
    float x1;
    float y1;
    float yaw1;
    float x2;    //store the previous target;
    float y2;
    float yaw2;
    int end_flag;
}Target_t;

typedef struct
{
    float x;    //mm
    float y;    //mm
    float yaw;
    float x_sp;
    float y_sp;
    float dist;
} Pos_t;
typedef struct {
    float x;
    float y;
    //}point1 = {944,922}, point2 = {2656,922}, point3 = {1271,1928}, point4 ={1800,300}, point5 = {2329,1928},point6 = point1;
}Point_t;
int getFormationTarget(Pos_t stablePos1, Pos_t stablePos2, Target_t* target1, Target_t* target2);
float calc_dist1(void);
void init1(void);
void Calc_Pos1(float interval,  float coordinate_x, float coordinate_y, float yaw);
void change_target1(void);
Target_t trajectory_generate1(float interval, float coordinate_x, float coordinate_y, float yaw, Pos_t &stable_pos);

float calc_dist2(void);
void init2(void);
void Calc_Pos2(float interval,  float coordinate_x, float coordinate_y, float yaw);
void change_target2(void);
Target_t trajectory_generate2(float interval, float coordinate_x, float coordinate_y, float yaw, Pos_t &stable_pos);

float mypow(float x,int c);
void Moving_Average(float movearray[], int16_t len, int16_t *fil_cnt, float in, float *out);
float To_180_degrees(float x);
float limit(float x,float min, float max);
float absint(float x);
//point1={700,1800},point2={2700,1800},point3=point1;


#endif
