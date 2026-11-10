/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������?,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,
	CAN_3508_TRIGGER_ID = 0x205,
	CAN_CAP_ID = 0x211,
  CAN_FRICTION_MOTOR_R_ID = 0x206,
  CAN_FRICTION_MOTOR_L_ID = 0x207,//can1

  CAN_YAW_MOTOR_ID = 0x205,
  CAN_PIT_MOTOR_ID = 0x206,
	CAN_T_2006_MOTOR_ID = 0x207,//can2                
//注意回调函数必须判断后分开写
	CAN_CHASSIS_ALL_ID = 0x200,
  CAN_SuperCap_ID = 0x211,
  CAN_GIMBAL_ALL_ID = 0x1FF,
  CAN_GIMBAL_ALL_ID_2 = 0x2FF

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef union {
    float fp32;
    uint32_t int32;
} FloatToInt;

typedef struct
{
		uint16_t  last_target_power;
    uint16_t  input_voltage;
    uint16_t  capacitor_voltage;
    int16_t   given_current;
    uint16_t  target_power;
} SuperCap_measure_t;

// NEW SuperCap
#include "supercap_sdk.h"
extern SuperCap_Feedback_t SuperCap_Feedback;
extern SuperCap_Control_t SuperCap_Control;


/**
  * @brief          ���͵�����Ƶ���?(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���?, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���?, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���?, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���?
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t Dial, int16_t rev);
extern void CAN_cmd_gimbal_2FF(int16_t yaw, int16_t pitch, int16_t DIAL, int16_t rev);
extern void CAN_cmd_fri(int16_t rev, int16_t friR, int16_t fri_L, int16_t rev1);

//extern void CAN_cmd_SuperCap(int8_t capflag);
extern void CAN_cmd_SuperCap(SuperCap_Control_t* control);

/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);


/**
  * @brief          ���͵�����Ƶ���?(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���?, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���?, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���?, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���?, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��?
  * @param[in]      none
  * @retval         �������ָ��?
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��?
  * @param[in]      none
  * @retval         �������ָ��?
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������? 2006�������ָ��?
  * @param[in]      none
  * @retval         �������ָ��?
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_R_3508_measure_point(void);
extern const motor_measure_t *get_L_3508_measure_point(void);
extern const motor_measure_t *get_T_2006_measure_point(void);
//extern const motor_measure_t *get_dial_3508_measure_point(void);
/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          ���ص��̵��? 3508�������ָ��?
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��?
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
