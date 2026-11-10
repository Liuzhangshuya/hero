/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#include "CAN_receive.h"

#include "cmsis_os.h"
#include "chassis_task.h"
#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//motor data read
#define get_SuperCap_measure(ptr, data)                                                   \
    {                                                                                     \
        (ptr)->last_target_power = (ptr)->target_power;                                   \
        (ptr)->input_voltage =         (uint16_t)((data)[1] << 8 | (data)[0]);            \
        (ptr)->capacitor_voltage =     (uint16_t)((data)[3] << 8 | (data)[2]);            \
        (ptr)->given_current =         (uint16_t)((data)[5] << 8 | (data)[4]);            \
        (ptr)->target_power =          (uint16_t)((data)[7] << 8 | (data)[6]);            \
    }

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd =                   (uint16_t)((data)[0] << 8 | (data)[1]);      \
        (ptr)->speed_rpm =             (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current =         (uint16_t)((data)[4] << 8 | (data)[5]);      \
        (ptr)->temperate =             (data)[6];                                   \
    }

#define get_super_cap_measure(ptr, data)                            \
{                                                                   \
		(ptr)->chassis_max_power = (int16_t)((data)[0] << 8 | (data)[1]);            \
		(ptr)->chassis_power_buffer = (uint16_t)((data)[2] << 8 | (data)[3]);    \
		data_cap.int32 = ((data)[4] << 24) | ((data)[5] << 16) |((data)[6] << 8)  |(data)[7];        \
		(ptr)->cap_percent = data_cap.fp32;      \
}	

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[10];
SuperCap_measure_t  SuperCap;
FloatToInt data_cap;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static CAN_TxHeaderTypeDef gimbal_tx_message_2;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static uint8_t              SuperCap_can_send_data[8];

// NEW SuperCap
SuperCap_Feedback_t SuperCap_Feedback = {0};
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (hcan->Instance == CAN1) //底盘 + 摩擦轮
    {
      switch (rx_header.StdId)
      {     
      case CAN_3508_M1_ID:
        get_motor_measure(&motor_chassis[0], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 0);
        break;
      case CAN_3508_M2_ID:
        get_motor_measure(&motor_chassis[1], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 1);
        break;
      case CAN_3508_M3_ID:
        get_motor_measure(&motor_chassis[2], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 2);
        break;
      case CAN_3508_M4_ID:
        get_motor_measure(&motor_chassis[3], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 3);
        break;
			case CAN_3508_TRIGGER_ID:
        get_motor_measure(&motor_chassis[4], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 4);
        break;
      case CAN_FRICTION_MOTOR_R_ID:
        get_motor_measure(&motor_chassis[5], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 5);
        break;
      case CAN_FRICTION_MOTOR_L_ID:
        get_motor_measure(&motor_chassis[6], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 6);
        break;
			
      default:
      {
        break;
      }
      }
    }
    else if(hcan->Instance == CAN2) //云台 
    {
      switch (rx_header.StdId)
      {
     		case CAN_T_2006_MOTOR_ID:
        get_motor_measure(&motor_chassis[7], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 7);
        break;
      case CAN_YAW_MOTOR_ID:
        get_motor_measure(&motor_chassis[8], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 8);
        break;
      case CAN_PIT_MOTOR_ID:
        get_motor_measure(&motor_chassis[9], rx_data);
        detect_hook(CHASSIS_MOTOR1_TOE + 9);
        break;//找对就行主要看发送
			// case CAN_CAP_ID:
      //   get_super_cap_measure(&chassis_move, rx_data);
      //   detect_hook(CHASSIS_MOTOR1_TOE + 6);
      //   break;
      // NEW SuperCap
      case SUPERCAP_RECEIVE_CAN_ID:
        SuperCap_ParseRxData(rx_data, &SuperCap_Feedback);
        chassis_move.chassis_max_power = SuperCap_Feedback.chassis_power_limit_w;
        chassis_move.cap_percent = SuperCap_Feedback.cap_energy_percent;
        detect_hook(CHASSIS_MOTOR1_TOE + 6);
        break;
      default:
      {
        break;
      }
      }
    }
//    switch (rx_header.StdId)//不清楚留给谁的
//    {
//    
//    case CAN_SuperCap_ID:
//    {
//      //						static uint8_t i = 0;
//      get_SuperCap_measure(&SuperCap, rx_data);
//      break;
//    }
//    default:
//    {
//      break;
//    }
//    }
    // switch (rx_header.StdId)
    // {
    // case CAN_3508_M1_ID:
    //   get_motor_measure(&motor_chassis[0], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 0);
    //   break;
    // case CAN_3508_M2_ID:
    //   get_motor_measure(&motor_chassis[1], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 1);
    //   break;
    // case CAN_3508_M3_ID:
    //   get_motor_measure(&motor_chassis[2], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 2);
    //   break;
    // case CAN_3508_M4_ID:
    //   get_motor_measure(&motor_chassis[3], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 3);
    //   break;
    // case CAN_TRIGGER_MOTOR_ID:
    //   get_motor_measure(&motor_chassis[4], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 4);
    //   break;
    // case CAN_FRICTION_MOTOR_R_ID:
    //   get_motor_measure(&motor_chassis[5], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 5);
    //   break;
    // case CAN_FRICTION_MOTOR_L_ID:
    //   get_motor_measure(&motor_chassis[6], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 6);
    //   break;

    // case CAN_YAW_MOTOR_ID:
    //   get_motor_measure(&motor_chassis[8], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 8);
    //   break;
    // case CAN_PIT_MOTOR_ID:
    //   get_motor_measure(&motor_chassis[9], rx_data);
    //   detect_hook(CHASSIS_MOTOR1_TOE + 9);
    //   break;
    //   //        {
    //   //            static uint8_t i = 0;
    //   //            //get motor id
    //   //            i = rx_header.StdId - CAN_3508_M1_ID;
    //   //            get_motor_measure(&motor_chassis[i], rx_data);
    //   //            detect_hook(CHASSIS_MOTOR1_TOE + i);
    //   //            break;
    //   //        }
    // case CAN_SuperCap_ID:
    // {
    //   //						static uint8_t i = 0;
    //   get_SuperCap_measure(&SuperCap, rx_data);
    //   break;
    // }
    // default:
    // {
    //   break;
    // }
    // }
}





/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t T_2006, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (T_2006 >> 8);//顶部摩擦轮
    gimbal_can_send_data[5] = T_2006;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
void CAN_cmd_fri(int16_t TRIGGER, int16_t friR, int16_t fri_L, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (TRIGGER >> 8);
  gimbal_can_send_data[1] = TRIGGER;
  gimbal_can_send_data[2] = (friR >> 8);
  gimbal_can_send_data[3] = friR;
  gimbal_can_send_data[4] = (fri_L >> 8);
  gimbal_can_send_data[5] = fri_L;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x209) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x20A) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
//void CAN_cmd_gimbal_2FF(int16_t yaw, int16_t pitch, int16_t TRIGGER, int16_t rev)
//{
//  uint32_t send_mail_box;
//  gimbal_tx_message_2.StdId = CAN_GIMBAL_ALL_ID_2;
//  gimbal_tx_message_2.IDE = CAN_ID_STD;
//  gimbal_tx_message_2.RTR = CAN_RTR_DATA;
//  gimbal_tx_message_2.DLC = 0x08;
//  gimbal_can_send_data[0] = (yaw >> 8);
//  gimbal_can_send_data[1] = yaw;
//  gimbal_can_send_data[2] = (pitch >> 8);
//  gimbal_can_send_data[3] = pitch;
//  gimbal_can_send_data[4] = (TRIGGER >> 8);
//  gimbal_can_send_data[5] = TRIGGER;
//  gimbal_can_send_data[6] = (rev >> 8);
//  gimbal_can_send_data[7] = rev;
//  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message_2, gimbal_can_send_data, &send_mail_box);
//}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)   底盘
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

// void CAN_cmd_SuperCap(int8_t capflag)
// {
//     uint32_t send_mail_box;
//     chassis_tx_message.StdId = 0x210;
//     chassis_tx_message.IDE = CAN_ID_STD;
//     chassis_tx_message.RTR = CAN_RTR_DATA;
//     chassis_tx_message.DLC = 0x08;
//     SuperCap_can_send_data[0] = capflag ;
   

//     HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, SuperCap_can_send_data, &send_mail_box);
// }

// NEW SuperCap
SuperCap_Control_t SuperCap_Control = {0};
void CAN_cmd_SuperCap(SuperCap_Control_t* control) {
    uint32_t send_mail_box;
    uint8_t tx_buffer[8];
    SuperCap_PackTxData(control, tx_buffer);
    chassis_tx_message.StdId = SUPERCAP_SEND_CAN_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, tx_buffer, &send_mail_box);
}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[8];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[9];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[4];
}
const motor_measure_t *get_R_3508_measure_point(void)
{
  return &motor_chassis[5];
}
const motor_measure_t *get_L_3508_measure_point(void)
{
  return &motor_chassis[6];
}
const motor_measure_t *get_T_2006_measure_point(void)
{
    return &motor_chassis[7];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
