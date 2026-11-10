#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "remote_control.h"
#include "pid.h"
#include "chassis_task.h"
#include "voltage_task.h"

// NEW SuperCap
#include "CAN_receive.h"
#include "supercap_sdk.h"
#include "referee.h"




/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c
  * @brief      chassis power control底盘功率控制
  * @note       读取裁判系统功率和限制功率，通过配套手册中的公式计算chassis.out
  * @history
  *  Version    Date            Author          Modification
  *  V1.0		    2025.1.9    		 DB             add chassis power control
  *
  @verbatim
  ==============================================================================
	TODO:裁判系统与C板通信测试 CAN
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
	
	
	
	
//t=τ
// (20/16384)*(0.3)*(187/3591)/9.55       Pm_parameter=t/9.55       t=It*Kt    Kt=(0.3)*(187/3591)  It=20/16384

//extern cap_measure_t cap_measure; // capacitor data structure
extern RC_ctrl_t rc_ctrl;
uint16_t cap_measure_power=0;	//超电功率
int8_t cap_state = 0;
void Power_Control(chassis_move_t *chassis_move)
{

	uint16_t max_power_limit = 70;   //限制功率
	uint8_t cap_measure_voltage = 0;
	fp32 chassis_max_power = 0;	
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;
	
	//DB 20250109
//	fp32 Powerlimit_Kt=(0.3)*(187/3591);
//	fp32 Powerlimit_It=20/16384;
//	fp32 Powerlimit_t=Powerlimit_It*Powerlimit_Kt;
//	fp32 Pm_parameter=Powerlimit_t/9.55;//机械功率Pm参数
	//
	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55       toque_coefficient=t/9.55       t=It*Kt    Kt=(0.3)*(187/3591)  It=20/16384
	
	
	fp32 a = 1.23e-07;						 // k1
	fp32 k1 = 2.0456e-07;	
	fp32 k2 = 1.453e-07;					 // k2
	fp32 constant = 4.081f;//重新写 静态功率
	
//读取超电给的 底盘功率 和 功率限制   cap_measure_power max_power_limit
//	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	PID_calc(&chassis_move->chassis_buffer_pid, chassis_move->chassis_power_buffer, 30);
//	get_chassis_max_power(&max_power_limit);
 	input_power = chassis_move->chassis_max_power - chassis_move->chassis_buffer_pid.out; // Input power floating at maximum power

	//CAN_cmd_SuperCap(chassis_move->capflag); // set the input power of capacitor controller
	// NEW SuperCap
	// TODO 找初始化的位置
	
	SuperCap_InitDefaultControl(&SuperCap_Control);
	get_chassis_power_and_buffer(
			&SuperCap_Control.referee_power_limit, 
			&SuperCap_Control.referee_energy_buffer);
	CAN_cmd_SuperCap(&SuperCap_Control);

/*
接收cap_measure_voltage超电电压
接收max_power_limit限制功率
接收cap_measure_power超电功率
接收chassis_power底盘功率

最终计算输出最大功率为chassis_max_power

最终计算输出最大功率为      chassis_max_power = max_power_limit + cap_measure_power(或者为超电可补充的常数)
*/
//	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
//	{
//		cap_state = cap_state + 1;
//	}
//	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
//	{
//		cap_state = cap_state - 1;
//	}
//	if(cap_state > 9 )
//	{
//		cap_state = 9;
//	}
//	if(cap_state < 0 )
//	{
//		cap_state = 0;
//	}
	//计算在超级电容补充与否的情况下  底盘所能承受的最大功率chassis_max_power

//	if (chassis_move->cap_percent > 5)
//	{
//		//if (cap_state == 0)
//		if (0)
//		{
//			chassis_max_power = input_power + 100; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
//		}
//		else
//		{
//			chassis_max_power =50;
//		}
//	}
//	else
//	{
//		chassis_max_power = 50;
//	}

	// NEW SuperCap
	// if(chassis_move->capflag == 0)
	// 	chassis_max_power = 55 + 5 ;
	// else if(chassis_move->capflag == 1)
	// 	chassis_max_power = 60 + 5;
	// else if(chassis_move->capflag == 2)
	// 	chassis_max_power = 65 + 5;
	// else if(chassis_move->capflag == 3)
	// 	chassis_max_power = 70 + 5;
	// else if(chassis_move->capflag == 4)
	// 	chassis_max_power = 75 + 5;
	// else if(chassis_move->capflag == 5)
	// 	chassis_max_power = 80 + 5;
	// else if(chassis_move->capflag == 6)
	// 	chassis_max_power = 85 + 5;
	// else if(chassis_move->capflag == 7)
	// 	chassis_max_power = 90 + 5;
	// else if(chassis_move->capflag == 8)
	// 	chassis_max_power = 100 + 5;
	// else if(chassis_move->capflag == 9)
	// 	chassis_max_power = 120 + 5;
	chassis_max_power = SuperCap_Feedback.chassis_power_limit_w;

	
//Pin
	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = chassis_move->motor_speed_pid[i].out * toque_coefficient * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm +
								k2 * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm +
								a * chassis_move->motor_speed_pid[i].out * chassis_move->motor_speed_pid[i].out + constant;

		if (initial_give_power < 0) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		fp32 power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}
//kt2+wt/9.55+k2w2+a-Pin=0  求t
	
			fp32 b = toque_coefficient * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k1 * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

			if (chassis_move->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_move->motor_speed_pid[i].out = 16000;
				}
				else
					chassis_move->motor_speed_pid[i].out = temp;
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_move->motor_speed_pid[i].out = -16000;
				}
				else
					chassis_move->motor_speed_pid[i].out = temp;
			}
		}
	}
}
