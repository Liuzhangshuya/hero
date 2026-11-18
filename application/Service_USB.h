#ifndef SERVICE_USB_H
#define SERVICE_USB_H

#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t task_usb_handle;
void Service_USB_Init(void);
void Task_USB(void *args);
void Service_USB_Receive(const uint8_t *data, uint32_t length);

#endif
