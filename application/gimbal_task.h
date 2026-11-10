/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
//pitch speed close-loop PID params, max out and max iout


#define RC_KEY_SHOT_TIME   300        //�����л�ģʽ�̰�ʱ��
#define RC_KEY_LONG_TIME   500        //�����л�ģʽ����ʱ��
#define RC_KEY_SUPPLY_TIME 700        //��겹������ʱ��
#define RC_KEY_LONG_LONG_TIME 2000    //�����л�ģʽ������ʱ��

//pitch �ٶȻ� PID�����Լ� PID��������������� �
#define PITCH_SPEED_PID_KP        12000.0f//7200
#define PITCH_SPEED_PID_KI        0.0f//60
#define PITCH_SPEED_PID_KD        0.0f//0.5
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        30000.0f   //6666.0
#define YAW_SPEED_PID_KI        0.0f  //4.0
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 8.0f//10
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP        8.0f  //6.0
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        9.0f 
#define YAW_ENCODE_RELATIVE_PID_KI        4.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//PITCH AUTO AIM
#define PITCH_AUTO_ABSOLUTE_PID_KP 22.0f  
#define PITCH_AUTO_ABSOLUTE_PID_KI 0.0f
#define PITCH_AUTO_ABSOLUTE_PID_KD 0.0f
#define PITCH_AUTO_ABSOLUTE_PID_MAX_OUT 50.0f
#define PITCH_AUTO_ABSOLUTE_PID_MAX_IOUT 0.0f

//pitch speed close-loop PID params, max out and max iout
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_AUTO_PID_KP        7000.0f				
#define PITCH_SPEED_AUTO_PID_KI        65.0f					
#define PITCH_SPEED_AUTO_PID_KD        0.0f					
#define PITCH_SPEED_AUTO_PID_MAX_OUT   30000.0f			
#define PITCH_SPEED_AUTO_PID_MAX_IOUT  5000.0f	

//YAW AUTO AIM
#define YAW_AUTO_ABSOLUTE_PID_KP        11.0f
#define YAW_AUTO_ABSOLUTE_PID_KI        0.80f
#define YAW_AUTO_ABSOLUTE_PID_KD        0.0f
#define YAW_AUTO_ABSOLUTE_PID_MAX_OUT   200.0f
#define YAW_AUTO_ABSOLUTE_PID_MAX_IOUT  0.0f

//yaw speed close-loop PID params, max out and max iout
//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_AUTO_PID_KP        8500.0f
#define YAW_SPEED_AUTO_PID_KI        85.0f
#define YAW_SPEED_AUTO_PID_KD        0.0f
#define YAW_SPEED_AUTO_PID_MAX_OUT   30000.0f
#define YAW_SPEED_AUTO_PID_MAX_IOUT  5000.0f

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//turn 180��
//��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_R
//turn speed
//��ͷ��̨�ٶ�
#define TURN_SPEED    0.04f
//���԰�����δʹ��
//#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000005f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  0//	ƽ��ͷ1	���ϻ�ͷ0
#define YAW_TURN    1

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//��̨��ʼ������ֵ�����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

		
/******************************************     tianjain    **********************************************/

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, // ��̨����
  GIMBAL_INIT,           // ��̨��ʼ��
  GIMBAL_LOCK,           // ��̨����
  GIMBAL_ABSOLUTE_ANGLE, // ��̨�����Ǿ��ԽǶȿ���
  GIMBAL_RELATIVE_ANGLE, // ��̨�������ֵ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     // ��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
  GIMBAL_SPIN,           // С����ģʽ
  GIMBAL_CALI,
  GIMBAL_AUTO
} gimbal_behaviour_e;

typedef enum
{
  GIMBAL_SUB_SUPPLY,
	GIMBAL_SUB_OBLI,
	GIMBAL_SUB_NAMORL,
} gimbal_sub_behaviour_e;

typedef enum
{
	LOCK_MOTOR_INIT,
	LOCK_MOTOR_NAMORL,
} lock_motor_behaviour_e;

		
/******************************************    tianjain   **********************************************/	

//typedef enum
//{
//    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
//    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
//    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
//} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
		gimbal_PID_t gimbal_motor_auto_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
		pid_type_def gimbal_motor_AUTO_gyro_pid;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
		fp32 auto_add_angle;
		fp32 aoto_angle_set;
} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{

	  const RC_ctrl_t *gimbal_rc_ctrl;
	  RC_ctrl_time_t RC_gimbal_ctrl_time;
	
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
	
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
//		UPTrans_Motor_t lock_motor;
	
	
	  gimbal_step_cali_t gimbal_cali;
	
//	
//		gimbal_sub_behaviour_e gimbal_sub_motor_mode;
	
    gimbal_behaviour_e gimbal_motor_mode;
    gimbal_behaviour_e last_gimbal_motor_mode;
	
	  lock_motor_behaviour_e lock_motor_mode;
	  lock_motor_behaviour_e last_lock_motor_mode;
	
	  uint16_t gimbal_lock_loop;
  	uint16_t direction_change;
	  fp32 chassis_w_offset;
		unsigned char ShootMode;
		 fp32 yaw_add_yaw_angle;
		 fp32 pitch_add_pitch_angle;
} gimbal_control_t;

/******************************************    tianjain   **********************************************/	
/******************************************    tianjain   **********************************************/	

typedef union
{
	short f;
	char c[2];
}float2uchar;
/**@brief:���սṹ�����?
*@param:pitch���� yaw����
*/
typedef struct 
{
	float2uchar pitch_data_;
	float2uchar yaw_data_;
	unsigned char IsHaveArmor;
	unsigned char distance;
	unsigned char	IsShootBuff ;
}GetData;
typedef struct 
{
	float2uchar pitch_data_;
	float2uchar yaw_data_;
	uint8_t ShootSpeed ;
	unsigned char	ShootMode ;
}SendData;

extern GetData *get_AUTO_control_point(void);
extern GetData get_data;//�������ݽṹ��




/******************************************    tianjain   **********************************************/	
/******************************************    tianjain   **********************************************/	

extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

extern const gimbal_control_t *get_gimbal_control_point(void);

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         ����1 �����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current, fri_R_can_set_current, fri_L_can_set_current;
#endif
