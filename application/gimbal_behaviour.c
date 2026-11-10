/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================        
    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

#include "user_lib.h"
#include "AutoGimbal.h"
#include "SolveTrajectory.h"
#include "remote_control.h"
//当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()
#define SPIN_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SPIN_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_spin_angle_control(fp32 *yaw,  fp32 *pitch,gimbal_control_t *gimbal_control_set);
static void gimbal_AUTO_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
//云台行为状态机
//static Gimbal_behaviour_e gimbal_behaviour = gIMBAL_ZERO_FORCE;


/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
		static uint8_t lock_wait_flag = 0;        //大陀螺后锁模式标志位
		static fp32 lock_wait_time = 0;           //大陀螺后锁模式计时
		static uint8_t key_gimbal_spin_flag = 0;  //键盘大陀螺控制标志位
		static fp32 key_gimbal_spin_time = 0;     //键盘大陀螺控制标计时
		
		chassis_move_mode->last_gimbal_motor_mode = chassis_move_mode->gimbal_motor_mode;

		
		//左通道在下，无力模式，优先级第三		
    if(switch_is_down(chassis_move_mode->gimbal_rc_ctrl->rc.s[0]))
    {
      chassis_move_mode->gimbal_motor_mode = GIMBAL_ZERO_FORCE;
			//所有标志位计时归零
			lock_wait_flag = 0;      
			lock_wait_time = 0;          
			key_gimbal_spin_flag = 0;  
			key_gimbal_spin_time = 0;     
			return ; 
    }


	
		//左通道在中，绝对角度控制模式
    if (switch_is_mid(chassis_move_mode->gimbal_rc_ctrl->rc.s[0])&& key_gimbal_spin_flag == 0)
    {			

			//判断上一次是否为大陀螺模式，
			if (chassis_move_mode->last_gimbal_motor_mode == GIMBAL_SPIN)
			{
				lock_wait_flag = 1;				
			}
			
			switch (lock_wait_flag)
			{
				case 0:
				{
                    chassis_move_mode->gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE; // 云台电机编码值相对角度控制;
                    lock_wait_time = 0;
					break;
				}
				case 1:   //如果上一次为大陀螺模式，则用绝对角度控制，此时底盘模式为跟随云台，会进行归位
				{
					chassis_move_mode->gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
                    //在允许的误差角度停留一段时间，则进入云台锁定模式
					if(fabs(chassis_move_mode->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR)
					{
						lock_wait_time ++;
						if(lock_wait_time > 400)
						{
							lock_wait_flag = 0;								
						}
					}
					break;
				}
			}						
    }

/******************************************   正常   **********************************************/	

/***************************************** 大陀螺判断 *********************************************/		
		
		//右拨杆在上，或者标志位为1，大陀螺
		if (switch_is_up(chassis_move_mode->gimbal_rc_ctrl->rc.s[0]) || key_gimbal_spin_flag == 1)
    {
      //chassis_move_mode->gimbal_motor_mode = GIMBAL_SPIN;  // 20240312 cac
       chassis_move_mode->gimbal_motor_mode = GIMBAL_AUTO; //!!!!!!!!!!!!!!!!!!!!!!!!!!
    } 
		else   //按键计时归零
		{
      key_gimbal_spin_time = 0;				
		}		

/***************************************** 大陀螺判断 *********************************************/		

/***************************************** Q键大陀螺 *********************************************/

//		//Q键控制大陀螺
		if (chassis_move_mode->gimbal_rc_ctrl->key.v & SPIN_ON_KEYBOARD)
		{
		  if(key_gimbal_spin_flag == 0)
		  {
			  if (chassis_move_mode->RC_gimbal_ctrl_time.rc_keyQ_time < RC_KEY_LONG_TIME)
				 {
					 chassis_move_mode->RC_gimbal_ctrl_time.rc_keyQ_time++;
				   if(chassis_move_mode->RC_gimbal_ctrl_time.rc_keyQ_time == RC_KEY_LONG_TIME)
					 {
					   key_gimbal_spin_flag = 1;   //开启大陀螺
						 chassis_move_mode->RC_gimbal_ctrl_time.rc_keyQ_time = 0;	
					 }
				}
			}
			else if(key_gimbal_spin_flag == 1 && key_gimbal_spin_time > 1000)  //如果正在大陀螺，再按一下Q键关闭
			{
				key_gimbal_spin_flag = 0; 
			}		
		}
		else
		{
			chassis_move_mode->RC_gimbal_ctrl_time.rc_keyQ_time = 0;
		}
//		
/***************************************** Q键大陀螺 *********************************************/
		
		if(key_gimbal_spin_flag == 1)
		{
		  key_gimbal_spin_time ++;   //按键控制大陀螺开启，计时增加
		}
		

		//从无力模式切出，即进入初始化模式
		if (chassis_move_mode->last_gimbal_motor_mode == GIMBAL_ZERO_FORCE && chassis_move_mode->gimbal_motor_mode != GIMBAL_ZERO_FORCE)
		{
				chassis_move_mode->gimbal_motor_mode = GIMBAL_INIT;
		}	

}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

//    static fp32 rc_add_yaw;
//    static fp32 rc_add_pitch;

    /*******************************************************************************************/
    /*******************************************************************************************/

    // // 将遥控器的数据处理死区
    // rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);

    // rc_add_yaw = (yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen);

    // gimbal_control_set->chassis_w_offset = rc_add_yaw * 3.00f; // 变大

    // // 将控制增加量赋值
    // *add_yaw = rc_add_yaw;

    /*******************************************************************************************/
    /*******************************************************************************************/
    
		 if (gimbal_control_set->gimbal_cali.step != 0 && gimbal_control_set->gimbal_cali.step != 5)
    {
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
    }
		else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }

    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }

    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }

    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }

    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_SPIN)
    {
        gimbal_absolute_spin_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }

    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_motor_mode == GIMBAL_AUTO)
    {
        if (packet_vision.tracking == 1)
            gimbal_AUTO_angle_control(add_yaw, add_pitch, gimbal_control_set);
        else 
        {
            add_pitch = 0;
            add_yaw = 0;
        }
    }
}

/**



//**
//  * @brief          云台在某些行为下，需要射击停止
//  * @param[in]      none
//  * @retval         1: no move 0:normal
//  */

//bool_t gimbal_cmd_to_shoot_stop(void)
//{
//    if (gimbal_control_t.gimbal_motor_mode == GIMBAL_INIT || gimbal_control_t.gimbal_motor_mode == GIMBAL_CALI || gimbal_control_t.gimbal_motor_mode == GIMBAL_ZERO_FORCE)
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}


/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel *PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;


    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if (0)
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //保存掉头的目标值
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //不断控制到掉头的目标值，正转，反装是随机
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TURN_SPEED;
            }
            else
            {
                *yaw -= TURN_SPEED;
            }
        }
        //到达pi （180°）后停止
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}



/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;


}

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_absolute_spin_angle_control(fp32 *yaw,  fp32 *pitch,gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		   
		static int16_t yaw_channel = 0, pitch_channel = 0;
	  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
	  rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

		
//		gimbal_control_set->chassis_w_offset = (yaw_channel * YAW_RC_SEN  - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN ) * 3.00f;      //变大


		*yaw = (yaw_channel * YAW_RC_SEN  - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN ) ;
		*pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
		
}
//static void gimbal_AUTO_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
//{
//    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }
//    // autoSolveTrajectory(pitch, yaw, &aim_x1, &aim_y1, &aim_z1);
//    // *yaw -= 0.5f * PI; // 20240312 cac

//    fp32 yaw_angle_target, pitch_angle_target;
//    autoSolveTrajectory(&pitch_angle_target, &yaw_angle_target, &aim_x1, &aim_y1, &aim_z1);
//    *yaw = yaw_angle_target - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
//    *pitch = pitch_angle_target - gimbal_control_set->gimbal_pitch_motor.absolute_angle;
//}
      

static void gimbal_AUTO_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		
    autoSolveTrajectory(&gimbal_control_set->gimbal_pitch_motor.aoto_angle_set, &gimbal_control_set->gimbal_yaw_motor.aoto_angle_set, &aim_x1, &aim_y1, &aim_z1);

    *yaw =gimbal_control_set->gimbal_yaw_motor.aoto_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    *pitch = gimbal_control_set->gimbal_pitch_motor.aoto_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;

}
