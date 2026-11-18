#include "Service_USB.h"
#include "AutoGimbal.h"

TaskHandle_t task_usb_handle;

void Service_USB_Init(void)
{
    /* CDC receives in its interrupt callback; this task owns device startup. */
    if (xTaskCreate(Task_USB, "USB.init", 128, NULL, tskIDLE_PRIORITY + 2,
                    &task_usb_handle) != pdPASS)
        Error_Handler();
}

void Task_USB(void *args)
{
    UNUSED(args);
    MX_USB_DEVICE_Init();
    task_usb_handle = NULL;
    vTaskDelete(NULL);
}

void Service_USB_Receive(const uint8_t *data, uint32_t length)
{
    AutoGimbal_ReceiveUSB(data, length);
}
