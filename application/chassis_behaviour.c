  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================     
    如果要添加一个新的行为模式
    1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "gimbal_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "usart.h"
extern gimbal_control_t gimbal_control;
extern UART_HandleTypeDef huart6;


//static void servo_pwm_set(uint16_t pwm, uint8_t i);


/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


static void chassis_spin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);




//留意，这个底盘行为模式变量
//chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

		static uint8_t follow_flag = 0;            //小陀螺后底盘跟随云台标志位
		static fp32 follow_time = 0;               //小陀螺后底盘跟随云台计时
		static uint8_t key_chassis_spin_flag = 0;  //键盘小陀螺控制标志位
		static fp32 key_chassis_spin_time = 0;     //键盘小陀螺控制标计时
		static fp32 spin_wait_time = 0;	           //底盘小陀螺等待计时
		static uint8_t key_chassis_slow_flag = 0;  //底盘降速控制标志位
		//static chassis_move_mode->capflag=0;
		static int no_move_time = 0;
		
/******************************************    一    **********************************************/	

//		//系统初始化为无力，优先级最高
//    if(SYSTEM_GetSystemState() == SYSTEM_STARTING)  
//		{
//			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
//			return;
//		}
	
/******************************************    一    **********************************************/	

/******************************************    二    **********************************************/	
		
		//右拨杆在下，无力模式，优先级第二
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[0]))
    {
//			chassis_move_mode->chassis_sub_mode = CHASSIS_SUB_NORMAL;
      chassis_move_mode->chassis_mode = cHASSIS_ZERO_FORCE;
			//所有标志位计时归零
			follow_flag = 0;    
		  follow_time = 0;    
		  key_chassis_spin_flag = 0;  
			key_chassis_spin_time = 0; 
      spin_wait_time = 0;
			key_chassis_slow_flag = 0;
			
			return ;
    }
		
/******************************************    二    **********************************************/	
		
/******************************************    三    **********************************************/	
//		
//		//底盘不动一段时间
//		if(chassis_move_mode->chassis_mode == cHASSIS_NO_MOVE)
//		{
//			no_move_time++;
//			//如果云台完成初始化，或者不动计时大于20000，退出不动模式
//			if(gimbal_control.gimbal_motor_mode != GIMBAL_INIT || no_move_time > 4000)  
//			{	
//				no_move_time = 0;
//			}
//			else 
//			{
//				return;
//			}
//		}
		
/******************************************    三    **********************************************/	


/******************************************   正常   **********************************************/	
		
		//右拨杆在中间正常
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[0]) && (key_chassis_spin_flag == 0))
    {
					
			// chassis_move_mode->chassis_mode = cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;	
			
			//判断上一次是否为小陀螺模式，
			if(chassis_move_mode->last_chassis_mode == CHASSIS_SPIN)
			{
				follow_flag = 1;				
			}				
			switch(follow_flag)
			{
				case 0:
				{
					chassis_move_mode->chassis_mode = cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;	
					follow_time = 0;
					break;
				}
				case 1:   //如果上一次为小陀螺模式，则用底盘跟随云台控制一段时间
				{
					chassis_move_mode->chassis_mode = cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
					if(fabs(gimbal_control.gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
					{
						follow_time ++;
						if(follow_time > 350)
						{
							follow_flag = 0;								
						}
					}
					break;
				}
			}		
    }

/******************************************   正常    **********************************************/	

		
/***************************************** 小陀螺判断 *********************************************/		
		
		//右拨杆在上，或者标志位为1，小陀螺   
		if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[0]) || key_chassis_spin_flag == 1)
    {
			spin_wait_time ++;

			if(spin_wait_time > 200)  //等待一段时间再小陀螺
			{
				chassis_move_mode->chassis_mode = CHASSIS_SPIN; 
//         chassis_move_mode->chassis_mode = cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
      }
    }
		else   //否则等待计时归零，按键计时归零
		{
			spin_wait_time = 0;
			key_chassis_spin_time = 0;		
		}
/***************************************** 小陀螺判断 *********************************************/		
		
		
/***************************************** F键小陀螺 *********************************************/
		
		if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_F)  
		{
				key_chassis_spin_flag = 1;   //开启小陀螺
		}
		else if(key_chassis_spin_flag == 1 &&(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_C))  //如果正在小陀螺，再按一下C键关闭
		{
				key_chassis_spin_flag = 0; 
				chassis_move_mode->chassis_mode = cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
		}		
		
	
//	if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_R)  
//		{
//				chassis_move_mode->capflag = 1;   //R开启超电
//		}
//		else if(chassis_move_mode->capflag == 1 &&(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V))  //如果超电开启，按v关闭
//		{
//				chassis_move_mode->capflag  = 0; 
//		}

/*************************************** SHIFT底盘jian速 *******************************************/		


		
/*************************************** SHIFT底盘加速 *******************************************/		
		
		//只要从无力模式切换出去，就先切换为不动模式
		if (chassis_move_mode->last_chassis_mode == cHASSIS_ZERO_FORCE && chassis_move_mode->chassis_mode != cHASSIS_ZERO_FORCE)
		{
				chassis_move_mode->chassis_mode = cHASSIS_NO_MOVE;
		}
		
		
/*************************************** 超级电容功率 *******************************************/		
//	if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z){
//		
//	HAL_UART_Transmit_DMA(&huart6,  (uint8_t *)"+", 1);
//	chassis_move_mode->power_limit +=5;
//	vTaskDelay(200);
//	
//	}  
//	if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_X){
//		
//	HAL_UART_Transmit_DMA(&huart6,  (uint8_t *)"-", 1);
//	chassis_move_mode->power_limit -=5;
//	vTaskDelay(200);
//	}  
//	if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_C){
//		
//			HAL_UART_Transmit_DMA(&huart6,  (uint8_t *)"1", 1);
//			vTaskDelay(500);
//	}  	
//	if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V){
//		
//	chassis_move_mode->total_current_limit=64000;//120w
	
	
//	} 
/*************************************** 超级电容功率 *******************************************/		

	
//		if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
//			{ 	
//			servo_pwm_set(3,100);	
//			}
//		else{
//		
//		servo_pwm_set(3,10);		
//		}  



		
		
		
		
		
		  
}



/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_move_rc_to_vector->chassis_mode == cHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == cHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == cHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_mode == cHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_SPIN)
    {
        chassis_spin_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}


/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
    static fp32 swing_time = 0.0f;
    
    static fp32 swing_angle = 0.0f;
    //max_angle is the max angle that chassis will ratate
    //max_angle 是sin函数的幅值
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //swing_time  plus the add_time in one control cycle
    //swing_time 在一个控制周期内，加上 add_time
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    static uint8_t swing_flag = 0;

    //judge if swing
    //判断是否要摇摆
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    //判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //swing_time  range [0, 2*PI]
    //sin函数不超过2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }


    *angle_set = swing_angle;
}


/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

static void chassis_spin_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    //*wz_set = (4-fabs(chassis_move_rc_to_vector->vx_set)-fabs(chassis_move_rc_to_vector->vy_set)) * 2.30f;  //正 1.3
		*wz_set = SPIN_SPEED;
}