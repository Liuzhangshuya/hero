#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 0 //9.78
#include "struct_typedef.h"

enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//设置参数
struct SolveTrajectoryParams
{
    fp32 k;             //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    fp32 current_v;      //当前弹速
    fp32 current_pitch;  //当前pitch
    fp32 current_yaw;    //当前yaw

    //目标参数
    fp32 xw;             //ROS坐标系下的x
    fp32 yw;             //ROS坐标系下的y
    fp32 zw;             //ROS坐标系下的z
    fp32 vxw;            //ROS坐标系下的vx
    fp32 vyw;            //ROS坐标系下的vy
    fp32 vzw;            //ROS坐标系下的vz
    fp32 tar_yaw;        //目标yaw
    fp32 v_yaw;          //目标yaw速度
	  fp32 set_yaw;
    fp32 ref_yaw;
    fp32 alo_max;
    fp32 alo_min;
		int shoot_flag;
		fp32 lastv_yaw;      //上一次目标yaw速度
    fp32 r1;             //目标中心到前后装甲板的距离
    fp32 r2;             //目标中心到左右装甲板的距离
    fp32 dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    fp32 s_bias;         //枪口前推的距离
    fp32 z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

//用于存储目标装甲板的信息
struct tar_pos
{
    fp32 x;           //装甲板在世界坐标系下的x
    fp32 y;           //装甲板在世界坐标系下的y
    fp32 z;           //装甲板在世界坐标系下的z
    fp32 yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};
//单方向空气阻力模型
extern fp32 monoDirectionalAirResistanceModel(fp32 s, fp32 v, fp32 angle);
//完全空气阻力模型
extern fp32 completeAirResistanceModel(fp32 s, fp32 v, fp32 angle);
//pitch弹道补偿
extern fp32 pitchTrajectoryCompensation(fp32 s, fp32 y, fp32 v);
//根据最优决策得出被击打装甲板 自动解算弹道
extern void autoSolveTrajectory(fp32 *pitch, fp32 *yaw, fp32 *aim_x, fp32 *aim_y, fp32 *aim_z);
extern void autoSolveTrajectory2(fp32 *pitch, fp32 *yaw, fp32 *aim_x, fp32 *aim_y, fp32 *aim_z);
void fire_ctrl(float allow_fire_ang_max, float allow_fire_ang_min);
	
float get_delta_ang_pi(float ang1, float ang2);
extern uint8_t	suggest_fire;

extern fp32 aim_x1, aim_y1, aim_z1,imu_yaw;

void initSTValue(void);

#endif /*__SOLVETRAJECTORY_H__*/