/**********************************************************************************************************
 * @�ļ�     AutoGimbal.c
 * @˵��     usart1��ʼ����PCͨ��
 * @�汾  	 V1.0
 * @����     ʢ��塢�������������
 * @����     2023.4.8
 **********************************************************************************************************/
#include "AutoGimbal.h"
#include "math.h"
#include "crc8_crc16.h"
#include "gimbal_task.h"
#include "referee.h"
#include "remote_control.h"
#include "SolveTrajectory.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t sbus_rx_buffer[BUFLENGTH];
uint8_t sbus_tx_buffer[BUFLENGTH];

const fp32 *INS_num;
// BUF RresPi;
GetData get_data;	// �������ݽṹ��
SendData send_data; // �������ݽṹ��
uint8_t data_length;
uint32_t data_length_version;
static uint32_t num_get = 0;
PackToVisionUnionDef PackToVisionUnion;
float aim_x1, aim_y1, aim_z1;
// CRC
// uint16_t CRC16_INIT = 0xFFFF;
uint16_t W_CRC_TABLE[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
	0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
	0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
	0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
	0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
	0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
	0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
	0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
	0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
	0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
	0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
	0x3de3, 0x2c6a, 0x1ef1, 0x0f78};
static const gimbal_control_t *AutoGimbal_data;
packetdef packet_vision;
struct SolveTrajectoryParams st;
static union
{
	float flo;
	char buf[4];
} HexFloat;
// number1,number2
float write_hex_to_float(uint8_t number1, uint8_t number2, uint8_t number3, uint8_t number4)
{
	HexFloat.buf[0] = number1;
	HexFloat.buf[1] = number2;
	HexFloat.buf[2] = number3;
	HexFloat.buf[3] = number4;
	float fdata = HexFloat.flo;
	return fdata;
}

uint8_t DFFF[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
int auto_flag;
// CRCУ��
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == 0x00)
		return 0xFFFF;
	while (dwLength--)
	{
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == 0x00) || (dwLength <= 2))
		return false;

	w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, 0xFFFF);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == 0x00) || (dwLength <= 2))
		return;

	w_crc = Get_CRC16_Check_Sum((uint8_t *)(pchMessage), dwLength - 2, 0xFFFF);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

void AUTO_control_init(void)
{
	auto_flag = 0;
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // ʹ�ܴ��ڿ����м�
	HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, BUFLENGTH);
	AutoGimbal_data = get_gimbal_control_point();
}
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {

// if(huart==&huart1)
// {
// 	memset(sbus_tx_buffer,0,TXDATALENGTH);
// 	unsigned char pitch_bit_,yaw_bit_;
// 	if(AutoGimbal_data->gimbal_pitch_motor.absolute_angle<0)
// 	{		pitch_bit_ = 0x00;
// 			send_data.pitch_data_.f=-AutoGimbal_data->gimbal_pitch_motor.absolute_angle;
// 	}
// 	else
// 	{ 	pitch_bit_ = 0x01;
// 			send_data.pitch_data_.f=AutoGimbal_data->gimbal_pitch_motor.absolute_angle;
// 	}

// 	if(AutoGimbal_data->gimbal_yaw_motor.absolute_angle<0)
// 	{		yaw_bit_ = 0x00;
// 			send_data.yaw_data_.f=-AutoGimbal_data->gimbal_yaw_motor.absolute_angle;
// 	}
// 	else
// 	{		yaw_bit_ = 0x01;
// 			send_data.yaw_data_.f=AutoGimbal_data->gimbal_yaw_motor.absolute_angle;
// 	}
// //		send_data.yaw_data_.f = 5;
// //		send_data.pitch_data_.f= 10;
// 	sbus_tx_buffer[0] ='s' ;//0x73;

// 	sbus_tx_buffer[1] = send_data.pitch_data_.f;
// 	sbus_tx_buffer[2] = send_data.pitch_data_.c[1];

// 	sbus_tx_buffer[3] = pitch_bit_;

// 	sbus_tx_buffer[4] = send_data.yaw_data_.f;
// 	sbus_tx_buffer[5] = send_data.yaw_data_.c[1];

// 	sbus_tx_buffer[6] = yaw_bit_;

// //	sbus_tx_buffer[6] = send_data.pitch_data_.c[0];
// //	sbus_tx_buffer[7] = send_data.pitch_data_.c[1];
// //	sbus_tx_buffer[8] = send_data.pitch_data_.c[2];
// //	sbus_tx_buffer[9] = send_data.pitch_data_.c[3];
// 	sbus_tx_buffer[10] = yaw_bit_;

// //���ٵȼ�0x01--14m/s 0x02--17m/s 0x03--28m/s������ģʽ0x00,�����������ģʽ0x01,��β0xFF
// 		sbus_tx_buffer[11]=get_robot_id();
// 		sbus_tx_buffer[12]=AutoGimbal_data->ShootMode;
// 		sbus_tx_buffer[13]=0xdd;
// 	//	HAL UART Receive DMA(&huart1,sbus_rx_buffer,BUFLENGTH)
// 	}
// }


void Usart1Receive_IDLE(void)
{
	HAL_UART_DMAStop(&huart1);
	
	static uint8_t index = 0;
	data_length_version = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	Verify_CRC16_Check_Sum(sbus_rx_buffer, sizeof(sbus_rx_buffer) - data_length_version);
	if (sbus_rx_buffer[0] == 0xA5)
	{
		packet_vision.header = sbus_rx_buffer[index];
		index += 1;
		packet_vision.tracking = sbus_rx_buffer[index] & 0x01;
		packet_vision.id = (sbus_rx_buffer[index] & 0x0e) >> 1;
		packet_vision.armors_num = (sbus_rx_buffer[index] & 0x70) >> 4;
		packet_vision.reserved = (sbus_rx_buffer[index] & 0x80) >> 7;
		index += 1;
		packet_vision.x = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.y = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.z = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.yaw = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.vx = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.vy = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.vz = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.v_yaw = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.r1 = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.r2 = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;
		packet_vision.dz = write_hex_to_float(sbus_rx_buffer[index], sbus_rx_buffer[index + 1], sbus_rx_buffer[index + 2], sbus_rx_buffer[index + 3]);
		index += 4;

		switch (packet_vision.id)
		{
		case 1:
			st.armor_id = ARMOR_HERO;
			break;
		case 2:
			st.armor_id = ARMOR_ENGINEER;
			break;
		case 3:
			st.armor_id = ARMOR_INFANTRY3;
			break;
		case 4:
			st.armor_id = ARMOR_INFANTRY4;
			break;
		case 5:
			st.armor_id = ARMOR_INFANTRY5;
			break;
		case 6:
			st.armor_id = ARMOR_GUARD;
			break;
		case 7:
			st.armor_id = ARMOR_BASE;
			break;
		default:
			st.armor_id = ARMOR_OUTPOST;
			break;
		}
		switch (packet_vision.armors_num)
		{
		case 2:
			st.armor_num = ARMOR_NUM_BALANCE;
			break;
		case 3:
			st.armor_num = ARMOR_NUM_OUTPOST;
			break;
		case 4:
			st.armor_num = ARMOR_NUM_NORMAL;
			break;
		default:
			break;
		}
		st.xw = -packet_vision.x;//packet_vision.x;
		st.yw = -packet_vision.y; // packet_vision.y;
		st.zw = packet_vision.z;
		st.tar_yaw = packet_vision.yaw + PI;
		st.vxw = packet_vision.vx;
		st.vyw = packet_vision.vy;
		st.vzw = packet_vision.vz;
		st.v_yaw = packet_vision.v_yaw;
		st.r1 = packet_vision.r1;
		st.r2 = packet_vision.r2;
		st.dz = packet_vision.dz;

		st.current_pitch = INS_num[1];
		st.current_yaw = INS_num[0];
		// st.xw = 3.0f;
		// st.yw = 0.0f;
		// st.zw = 0.2f;
		// st.tar_yaw = 0.0f;
		// st.vxw = 0.0f;
		// st.vyw = 0.0f;
		// st.vzw = 0.0f;
		// st.v_yaw = 0.0f;
		// st.r1 = packet_vision.r1;
		// st.r2 = packet_vision.r2;
		// st.dz = packet_vision.dz;
	}
	index = 0;
	// return 0;
	/*������
	//unit8_t data_length = BUFLEGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);	//128��ʣ��ռ� �õ��Ѿ����ܵ�����
	data_length = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
//	if(data_length	== RXDATALENGTH)
	{
//		if(verify_CRC16_check_sum(sbus_rx_buffer,16))
		{
					if(sbus_rx_buffer[0] == 's')
					{
						HAL_UART_Transmit_DMA(&huart1,sbus_tx_buffer,TXDATALENGTH);
						get_data.pitch_data_.c[1] = sbus_rx_buffer[3];
						get_data.pitch_data_.c[0] = sbus_rx_buffer[4];
//						get_data.pitch_data_.c[2] = sbus_rx_buffer[3];
//						get_data.pitch_data_.c[3] = sbus_rx_buffer[4];
						get_data.yaw_data_.c[1] = sbus_rx_buffer[1];
						get_data.yaw_data_.c[0] = sbus_rx_buffer[2];

//						get_data.yaw_data_.c[2] = sbus_rx_buffer[8];
//						get_data.yaw_data_.c[3] = sbus_rx_buffer[9];

						get_data.distance = sbus_rx_buffer[12];
//						//�жϱ�־λ��0x00λ���ţ�0x01Ϊ���ţ���������Ϊ�Ӿ��Ǳ����������Ŵ���
//						if (sbus_rx_buffer[5] == 0x000)
//						{
//								get_data.pitch_data_.f = -fabs(get_data.pitch_data_.f);
//						}
//						else
//						{
//							get_data.pitch_data_.f = fabs(get_data.pitch_data_.f);
//						}
//						if(sbus_rx_buffer[10] ==0x00)
//						{
//							get_data.yaw_data_.f = -(fabs(get_data.yaw_data_.f));
//						}
//						else
//						{
//							get_data.yaw_data_.f = (fabs(get_data.yaw_data_.f));
//						}
						//�ж��Ƿ���Ŀ�꣬�Ƿ����ӵ�
						if(sbus_rx_buffer[11] == 0x00)
						{
							get_data.IsHaveArmor = 0;
						}
						else
						{
							get_data.IsHaveArmor = 1;
						}
						if(sbus_rx_buffer[13] == 0x00)
						{
							get_data.IsShootBuff = 0;
						}
						else
						{
							get_data.IsShootBuff = 1;
						}
					}
			}
	}������*/
	
	memset(sbus_rx_buffer, 0, BUFLENGTH);
	// HAL_UART_Transmit_DMA(&huart1,sbus_tx_buffer,TXDATALENGTH);
	HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, BUFLENGTH);
}


GetData *get_AUTO_control_point(void)
{
	return &get_data;
}

// ��PC����
void PC_vosion_task(void const *argument)
{
	vTaskDelay(AUTO_INIT_TIME);

	while (1)
	{
			INS_num = get_INS_angle_point();
			PackToVisionUnion.PackToVision.header = 0x5A;
			PackToVisionUnion.PackToVision.detect_color = 0; // 0-red 1-blue
			PackToVisionUnion.PackToVision.task_mode = 2;
			PackToVisionUnion.PackToVision.reserved = 5;
			
			PackToVisionUnion.PackToVision.roll  = 	INS_num[2];
			PackToVisionUnion.PackToVision.pitch =	INS_num[1];
			PackToVisionUnion.PackToVision.yaw 	 =  INS_num[0];
			

//			PackToVisionUnion.PackToVision.aim_x = aim_x1;
//			PackToVisionUnion.PackToVision.aim_y = aim_y1;
//			PackToVisionUnion.PackToVision.aim_z = aim_z1;
			PackToVisionUnion.PackToVision.aim_x = aim_x1;
			PackToVisionUnion.PackToVision.aim_y = aim_y1;
			PackToVisionUnion.PackToVision.aim_z = aim_z1;
			Append_CRC16_Check_Sum(PackToVisionUnion.UsartData, sizeof(PackToVisionUnion.PackToVision));

			/* 20240314 cac: 添加临界代码段 */
			taskENTER_CRITICAL();
			CDC_Transmit_FS(PackToVisionUnion.UsartData, sizeof(PackToVisionUnion.PackToVision));
			taskEXIT_CRITICAL();
			vTaskDelay(AUTO_TIME);
	
	}
	
}