/**
 * @file Service_USB.cpp
 * @author cac
 * @brief
 * @version 0.1
 * @date 2024-03-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Service_USB.h"
#include "AutoGimbal.h"

//packetdef packet_vision;
TaskHandle_t task_usb_handle;
QueueHandle_t usb_cdc_queue_handle;
uint8_t usb_cdc_queue_buffer[USB_CDC_QUEUE_BUF_LEN];
uint8_t *p_cdc_queue_now = usb_cdc_queue_buffer;
float write_hex_to_float(uint8_t number1, uint8_t number2, uint8_t number3, uint8_t number4);

static void unpack_vision(uint8_t *buffer, uint8_t packet_length);

//void Service_USB_Init(void)
//{
//    xTaskCreate(Task_USB, "Task.USB", Huge_Stack_Size, NULL, PriorityHigh, &task_usb_handle);
//    usb_cdc_queue_handle = xQueueCreate(USB_CDC_QUEUE_BUF_LEN, sizeof(uint8_t));
//}

/**
 * @brief USB CDC 接收处理任务
 *
 * @param args
 */
void Task_USB(void *args)
{
    UNUSED(args);
    MX_USB_DEVICE_Init();
    for (;;)
    {
        while (1)
        {
            if (xQueueReceive(usb_cdc_queue_handle, p_cdc_queue_now, portMAX_DELAY) == pdPASS)
            {
                if (p_cdc_queue_now == usb_cdc_queue_buffer)
                {
                    if (*p_cdc_queue_now == 0xA5)
                        p_cdc_queue_now++;
                    else
                        p_cdc_queue_now = usb_cdc_queue_buffer;
                }
                else if (p_cdc_queue_now >= usb_cdc_queue_buffer + PACKET_SIZE - 1)
                {
                    p_cdc_queue_now = usb_cdc_queue_buffer;
                    break;
                }
                else
                {
                    p_cdc_queue_now++;
                }
            }
        }
        taskENTER_CRITICAL();
        // todo: packet length check
        // if(content_length == ) {
				if (Verify_CRC16_Check_Sum(usb_cdc_queue_buffer, PACKET_SIZE))
        unpack_vision(usb_cdc_queue_buffer, PACKET_SIZE);

        taskEXIT_CRITICAL();
    }
    vTaskDelete(NULL);
}

/**
 * @brief PCvision解包函数，更新st
 *
 * @param buffer 要解包的数据缓冲区(全数据包)
 * @param packet_length 数据长度
 */
void unpack_vision(uint8_t *buffer, uint8_t packet_length)
{
    static uint8_t index = 0;
	
//	Verify_CRC16_Check_Sum(buffer, sizeof(buffer) - data_length_version);
	if (buffer[0] == 0xA5)
	{
		packet_vision.header = buffer[index];
		index += 1;
		packet_vision.tracking = buffer[index] & 0x01;
		//packet_vision.tracking = 1;
		packet_vision.id = (buffer[index] & 0x0e) >> 1;
		packet_vision.armors_num = (buffer[index] & 0x70) >> 4;
		packet_vision.reserved = (buffer[index] & 0x80) >> 7;
		index += 1;
		packet_vision.x = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.y = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.z = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.yaw = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.vx = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.vy = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.vz = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.v_yaw = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.r1 = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.r2 = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
		index += 4;
		packet_vision.dz = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
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
		st.xw = packet_vision.x;//对面车的x坐标;
		st.yw = packet_vision.y; //对面车的y坐标;
		st.zw =	packet_vision.z;//对面车的z坐标;
		st.tar_yaw = packet_vision.yaw;
		st.vxw = -packet_vision.vy;
		st.vyw = packet_vision.vx;
		st.vzw = -packet_vision.vz;
		if(fabs(st.lastv_yaw-packet_vision.v_yaw) > 1){
			st.lastv_yaw=packet_vision.v_yaw;
			st.v_yaw = st.lastv_yaw;
		}
		else{
			st.v_yaw = 0;
		}
		//st.v_yaw = packet_vision.v_yaw;
		st.r1 = packet_vision.r1;
		st.r2 = packet_vision.r2;
		st.dz = packet_vision.dz;

		st.current_pitch = INS_num[2];
		st.current_yaw = INS_num[0];
		
		
//		st.xw = 1;//对面车的x坐标;//右负左正
//		st.yw = -1; //对面车的y坐标;//前负后正//右负左正
//		st.zw =	0.5;//对面车的z坐标;//上负下正//上正下负
//		st.tar_yaw = 0;
//		st.vxw = 0;
//		st.vyw = 0;
//		st.vzw = 0;
//		st.v_yaw = 0;
//		st.r1 = 0.26;
//		st.r2 = 0.26;
//		st.dz = 0.2;
//		st.current_pitch = INS_num[1];
//		st.current_yaw = INS_num[0];
		
	}
	index = 0;
	

}
/*
void unpack_vision(uint8_t *buffer, uint8_t packet_length)
{
    if (packet_length != PACKET_SIZE)
    {
        return;
    }
    static uint8_t index = 0;
    // packetdef packet_vision;
    // data_length_version = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
    // todo: CRC checksum
    // Verify_CRC16_Check_Sum(buffer, packet_length);
    if (buffer[0] == 0xA5)
    {
        packet_vision.header = buffer[index];
        index += 1;
        packet_vision.tracking = buffer[index] & 0x01;
        packet_vision.reserved = 0;
        index += 1;
        packet_vision.x = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
        index += 4;
        packet_vision.y = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
        index += 4;
        packet_vision.z = write_hex_to_float(buffer[index], buffer[index + 1], buffer[index + 2], buffer[index + 3]);
        index += 4;


        st.xw = packet_vision.x;
        st.yw = packet_vision.y;
        st.zw = packet_vision.z;
    }
    index = 0;
}*/
