/**
 * @file Service_USB.h
 * @author cac
 * @brief
 * @version 0.1
 * @date 2024-03-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef SERVICE_USB_H
#define SERVICE_USB_H


#include "usb_device.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "SolveTrajectory.h"
#include "AutoGimbal.h"
//#include "PCvision.h"
//#include "Service_SwerveChassis.h"
#include "math.h"

#define USB_CDC_QUEUE_BUF_LEN 128
#define USB_CDC_RX_BUF_LEN 128
#define PACKET_SIZE 48

extern TaskHandle_t task_usb_handle;
extern QueueHandle_t usb_cdc_queue_handle;
//extern packetdef packet_vision;
void Service_USB_Init(void);
void Task_USB(void *args);

#endif



