#ifndef AUTOGIMBAL_H
#define AUTOGIMBAL_H

#include "string.h"
#include "main.h"
#include "gimbal_task.h"
#include "stdbool.h"

#define BUFLENGTH 128   // �����յ�����
#define RXDATALENGTH 16 // ��Ч����
#define TXDATALENGTH 14 // ��Ч����
#define AUTO_TIME 1
#define AUTO_INIT_TIME 201
/***б��PWMϵ��***/
#define YAW_AUTO_SEN -0.000015f   // 0.0015f
#define PITCH_AUTO_SEN -0.000015f // 0.0015f //0.005
/***��������������ַ���С���?��***/

/**@brief:���ͽṹ�����?
 *@param:pitch���� yaw����
 */
__packed struct __PackToVision_Def 
{
    // uint8_t head;

    // float	pitchAngle;
    // float	yawAngle;
    // uint8_t color;
    // uint8_t bulletSpeed;
    // uint8_t mode;
    // uint8_t end;

    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    uint8_t task_mode : 2;
    uint8_t reserved : 5;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
};

typedef struct __PackToVision_Def PackToVision_Def;

typedef struct
{
    uint8_t header;
    bool tracking;
    uint8_t id;
    uint8_t armors_num;
    uint8_t reserved;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum;
} packetdef;

typedef union
{
    // uint8_t UsartData[12];
    PackToVision_Def PackToVision;
    uint8_t UsartData[sizeof(PackToVision_Def)];
    
} PackToVisionUnionDef;
extern const fp32 *INS_num;
extern struct SolveTrajectoryParams st;
extern packetdef packet_vision;

extern void AUTO_control_init(void);
extern void PC_vosion_task(void const *argument);
extern void Usart1Receive_IDLE(void);
extern uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);
// const gimbal_control_t *get_gimbal_control_point();
#endif
