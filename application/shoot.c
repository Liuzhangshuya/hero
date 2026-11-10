/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"


#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

// union UnionFloat
// {
//     uint32_t u;
//     float f;
// };

// float uint6_cov_float(uint16_t value)
// {
//     const fp32 magic = {(254U - 15U) << 23};
//     const fp32 was_infnan = {(127U + 16U) << 23};
//     UnionFloat out;

//     out.u = (value & 0x7FFFU) << 13; /* exponent/mantissa bits */
//     out.f *= magic.f;                /* exponent adjust */
//     if (out.f >= was_infnan.f)       /* make sure Inf/NaN survive */
//     {
//         out.u |= 255U << 23;
//     }
//     out.u |= (value & 0x8000U) << 16; /* sign bit */

//     return out.f;
// }

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void
shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
static void shoot_bullet_control_back(void);


shoot_control_t shoot_control;          //射击数据

void calculate_motor3508_total_angle(shoot_control_t *motor_data)
{
	if(motor_data->offset_flag == 0)
	{
		motor_data->offset_ecd=motor_data->ecd;
		motor_data->round_cnt = 0;
		motor_data->offset_flag=1;
	}
	else
	{
		if(motor_data->ecd - motor_data->last_ecd >  4096) motor_data->round_cnt--;
		if(motor_data->ecd - motor_data->last_ecd < -4096) motor_data->round_cnt++;
	}
	motor_data->total_ecd=motor_data->round_cnt * (int64_t)8192 + motor_data->ecd - motor_data->offset_ecd;
	
}




/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    shoot_control.shoot_mode = SHOOT_STOP;
    static const fp32 R_3508_speed_pid[3] = {10, 0.1, 0};
    static const fp32 L_3508_speed_pid[3] = {10, 0.1, 0};
		static const fp32 T_2006_speed_pid[3] = {10, 0.1, 0};
		 static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD}; // 800 0.5 0

		
    shoot_control.last_s = RC_SW_UP;
    shoot_control.new_s = RC_SW_UP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.R_3508.motor_measure = get_R_3508_measure_point();
    shoot_control.L_3508.motor_measure = get_L_3508_measure_point();
		shoot_control.T_2006.motor_measure = get_T_2006_measure_point();
    //初始化PID
		PID_init(&shoot_control.trigger_motor_angle_pid, PID_POSITION, Trigger_angle_pid, 13000, 13000);//10000,5000(gai)
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.R_3508_pid, PID_POSITION, R_3508_speed_pid, 15000, 7000);
    PID_init(&shoot_control.L_3508_pid, PID_POSITION, L_3508_speed_pid, 15000, 7000);
		PID_init(&shoot_control.T_2006_pid, PID_POSITION, T_2006_speed_pid, 10000, 7000);
    //更新数据
    shoot_feedback_update();
    shoot_control.R_3508.give_current = 0;
    shoot_control.L_3508.give_current = 0;
		shoot_control.T_2006.give_current = 0;
    //PWM控制
    // ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    // ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    // shoot_control.fric_pwm1 = FRIC_OFF;
    // shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
        shoot_control.R_3508.speed_set = 0.0f;
        shoot_control.L_3508.speed_set = 0.0f;
			  shoot_control.T_2006.speed_set = 0.0f;
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

        PID_calc(&shoot_control.R_3508_pid, shoot_control.R_3508.motor_measure->speed_rpm, shoot_control.R_3508.speed_set);
        shoot_control.R_3508.give_current = (int16_t)(shoot_control.R_3508_pid.out);

        PID_calc(&shoot_control.L_3508_pid, shoot_control.L_3508.motor_measure->speed_rpm, shoot_control.L_3508.speed_set);
        shoot_control.L_3508.give_current = (int16_t)(shoot_control.L_3508_pid.out);
			 
				PID_calc(&shoot_control.T_2006_pid, shoot_control.T_2006.motor_measure->speed_rpm, shoot_control.T_2006.speed_set);
        shoot_control.T_2006.give_current = (int16_t)(shoot_control.T_2006_pid.out);
        shoot_laser_off();
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
            // if ((fabsf((shoot_control.angle) - (shoot_control.set_angle))) < 0.005f)
            shoot_control.set_angle = shoot_control.angle;
        }
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
				shoot_bullet_control();
				
        //trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if (shoot_control.shoot_mode != SHOOT_STOP)
    {   
        shoot_laser_on(); //激光开启
        shoot_control.R_3508.speed_set = 9000;
        shoot_control.L_3508.speed_set = -9000;
			  shoot_control.T_2006.speed_set = 100;//400

			if(shoot_control.shoot_mode != SHOOT_BULLET)
			{
				PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
			}
        PID_calc(&shoot_control.R_3508_pid, shoot_control.R_3508.motor_measure->speed_rpm, shoot_control.R_3508.speed_set);
        shoot_control.R_3508.give_current = (int16_t)(shoot_control.R_3508_pid.out);

        PID_calc(&shoot_control.L_3508_pid, shoot_control.L_3508.motor_measure->speed_rpm, shoot_control.L_3508.speed_set);
        shoot_control.L_3508.give_current = (int16_t)(shoot_control.L_3508_pid.out);
        
				PID_calc(&shoot_control.T_2006_pid, shoot_control.T_2006.motor_measure->speed_rpm, shoot_control.T_2006.speed_set);
        shoot_control.T_2006.give_current = (int16_t)(shoot_control.T_2006_pid.out);
        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
					
        {
            shoot_control.given_current = 0;
        }
    }


    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(shoot_control.new_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(shoot_control.new_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    if (shoot_control.shoot_mode == SHOOT_READY_FRIC )
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
//		if (shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.R_3508.speed_set && shoot_control.L_3508.speed_set && abs(shoot_control.R_3508.speed_set - shoot_control.R_3508.motor_measure->speed_rpm) <= 2000 && abs(shoot_control.L_3508.speed_set - shoot_control.L_3508.motor_measure->speed_rpm) <= 2000)
//    {
//        shoot_control.shoot_mode = SHOOT_READY_BULLET;
//    }
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET && switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }

    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(shoot_control.last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;
            if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    

    if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }

    // if (shoot_control.shoot_mode == SHOOT_BULLET && switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.move_flag == 0)
    // {
    //     shoot_control.shoot_mode = SHOOT_READY_BULLET;
    // }
        get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
		    //如果云台状态是 无力状态，就关闭射击
    //if (gimbal_cmd_to_shoot_stop())
		if(switch_is_down(shoot_control.shoot_rc->rc.s[RIGHT_RC_MODE_CHANNEL]))
		{
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    if (shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL] != shoot_control.new_s)
    {
        shoot_control.last_s = shoot_control.new_s;
        shoot_control.new_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
    }
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    static fp32 speed_fliter_1_angle = 0.0f;
    static fp32 speed_fliter_2_angle = 0.0f;
    static fp32 speed_fliter_3_angle = 0.0f;
    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    static const fp32 fliter_angle_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		
		shoot_control.ecd = shoot_control.shoot_motor_measure->ecd;
		shoot_control.last_ecd = shoot_control.shoot_motor_measure->last_ecd;
		calculate_motor3508_total_angle(&shoot_control);
		
    shoot_control.ecd_float = (int)(shoot_control.shoot_motor_measure->ecd);
    // 微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }

    if (up_time > 0)
    {
        shoot_control.fric1_ramp.max_value = FRIC_UP;
        shoot_control.fric2_ramp.max_value = FRIC_UP;
        up_time--;
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }


}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void26218
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    // 每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0 )
    {
        shoot_control.set_total_ecd =shoot_control.total_ecd + 25941;	    
        shoot_control.move_flag = 1;
				shoot_control.T_2006.speed_set = 100; //正值往里转
    }
		
    if ((shoot_control.move_flag == 1) && llabs(shoot_control.total_ecd - shoot_control.set_total_ecd)<1500)
    {
				shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
				shoot_control.block_time = 0;
				shoot_control.set_total_ecd = shoot_control.total_ecd;
				shoot_control.T_2006.speed_set = 100;
		
    }
		else if ((shoot_control.move_flag == 1) && llabs(shoot_control.total_ecd - shoot_control.set_total_ecd)>300)
		{
			 shoot_control.block_time++;
		}
		
		if( shoot_control.block_time > 1000)
    {
        shoot_control.set_total_ecd = shoot_control.total_ecd - 500;	
    }
		PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.total_ecd, shoot_control.set_total_ecd);
    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_angle_pid.out);
}


//static void shoot_bullet_control(void)
//{

//    // 每次拨动 1/4PI的角度
//    if (shoot_control.move_flag == 0)
//    {
//        shoot_control.set_total_ecd =shoot_control.total_ecd + 26218;	    
//        shoot_control.move_flag = 1;
//    }
//		
//		PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.total_ecd, shoot_control.set_total_ecd);
//    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_angle_pid.out);
//		
//    if ((shoot_control.move_flag == 1) && llabs(shoot_control.total_ecd - shoot_control.set_total_ecd)<2000)
//    {
//				shoot_control.move_flag = 0;
//        shoot_control.shoot_mode = SHOOT_READY_BULLET;
//    }

//}

static void shoot_bullet_control_back(void)
{
    // 每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_total_ecd =shoot_control.total_ecd - 500;	    
        shoot_control.move_flag = 1;
    }
		
    if ((shoot_control.move_flag == 1) && llabs(shoot_control.total_ecd - shoot_control.set_total_ecd)<300)
    {
				shoot_control.trigger_back_flag = 1;
        shoot_control.move_flag = 0;
    }
		PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.total_ecd, shoot_control.set_total_ecd);
    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_angle_pid.out);
}