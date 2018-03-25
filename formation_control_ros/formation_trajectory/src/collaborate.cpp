#include "traj.h"

/*
* 功能：编队支持函数
* 日期：18/3/24
* 作者：ycnalin
* 所有输入输出单位均是以“MM”为基础
* 如长度为MM，速度为MM/s
*/




Pos_t oldStablePos2;//存储从机上次位置；初始值因为从机启动位置；全局变量

/*
* @arguments： stablePos1 主机滤波后位置；stablePos2对应从机位置，由网络传递
* @arguments: target1 主机根据运动路径生成的目标；target2 主机给从机的目标，由网络传递
* @returnVal: (-1)-停机，(0)-降落，(1)-正常运行，(其他)-保留
*/
int getFormationTarget(Pos_t stablePos1, Pos_t stablePos2, Target_t* target1, Target_t* target2) {
	float xr, yr, desireX, desireY, deviateX, deviateY;
	xr = stablePos2.x - stablePos1.x;
	yr = stablePos1.y - stablePos2.y; //y轴向里
	float dist = sqrt(xr * xr + yr * yr);
	float theta = atan2(yr, xr);

	//safe check begin
	if (dist < Radius + StopDist)
		return -1;
	if (dist < Radius + LandingDist)
		return 0;

	//calculate relative pos
	desireX = stablePos1.x + cos(Theta)*FlyDist;
	desireY = stablePos1.y + sin(-Theta)*FlyDist;
	deviateX = stablePos2.x - desireX; //devX>0 && devY>0 飞行器2实际位置在理想位置为原点的第一象限; devX<0 && Theta<pi/2时，有相撞可能
	deviateY = desireY - stablePos2.y; //devX>0 && devY<0 飞行器2实际位置在理想位置为原点的第四象限; devX>0 && Theta>pi/2时，有相撞可能

	//case 1: 主机等待从机，条件为滞后100mm,注意不考虑从机超前主机，因为超前会自动等待
	if (theta < Theta && ( abs(desireX) >= 50)) {
		target1->x1 = target1->x2; //主机使用上次目标不更新
		target1->y1 = target1->y2; //注意需修改circle文件charge_target函数，使target.x2,y2为上次位置
		target2->x1 = target1->x1 + cos(Theta)*FlyDist; //从机跟随主机目标
		target2->y1 = target1->y1 + sin(-Theta)*FlyDist;
		return 1;
	}

	//从机目标位置选择：1、跟随主机的实际位置；2、跟随主机的目标位置
	//尝试目标跟随，减少滞后性
	target2->x2 = target2->x1;//保存飞行器2的旧目标
	target2->y2 = target2->y1;
	target2->x1 = target1->x1 + cos(Theta)*FlyDist; //从机跟随主机目标
	target2->y1 = target1->y1 + sin(-Theta)*FlyDist;
	return 1;
}
