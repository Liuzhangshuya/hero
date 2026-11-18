#include "AutoGimbal.h"
#include "crc8_crc16.h"
#include "INS_task.h"
#include "SolveTrajectory.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t uart_rx_buffer[BUFLENGTH];
static uint16_t uart_rx_position;
static vision_receiver_t uart_receiver;
static vision_receiver_t usb_receiver;
static vision_command_t latest_command;
static uint32_t latest_received_ms;
static uint8_t has_command;

const fp32 *INS_num;
GetData get_data;
PackToVisionUnionDef PackToVisionUnion;
float aim_x1, aim_y1, aim_z1;
/* Retained for the legacy solver, which is no longer used by auto control. */
struct SolveTrajectoryParams st;

static void receive_bytes(vision_receiver_t *receiver, const uint8_t *data, size_t length)
{
    uint32_t now = HAL_GetTick();
    if (vision_receiver_feed(receiver, data, length, now) != 0U)
    {
        uint32_t irq_mask = __get_PRIMASK();
        __disable_irq();
        latest_command = receiver->command;
        latest_received_ms = receiver->received_ms;
        has_command = 1U;
        __set_PRIMASK(irq_mask);
    }
}

void AutoGimbal_ReceiveUSB(const uint8_t *data, uint32_t length)
{
    receive_bytes(&usb_receiver, data, length);
}

uint8_t AutoGimbal_GetTarget(vision_target_t *target)
{
    vision_command_t command;
    uint32_t received_ms, now;
    uint8_t valid;
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    command = latest_command;
    received_ms = latest_received_ms;
    valid = has_command;
    now = HAL_GetTick();
    __set_PRIMASK(irq_mask);
    memset(target, 0, sizeof(*target));
    if (!valid)
        return 0U;
    return vision_command_predict(&command, (uint32_t)(now - received_ms), target);
}

void AUTO_control_init(void)
{
    vision_receiver_init(&uart_receiver);
    vision_receiver_init(&usb_receiver);
    has_command = 0U;
    uart_rx_position = 0U;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, BUFLENGTH);
}

/* Circular RX DMA: consume on IDLE, half-full and full interrupts. This keeps
 * frame assembly independent of serial gaps and DMA wraparound. */
void Usart1Receive_IDLE(void)
{
    uint16_t position = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (position == BUFLENGTH)
        position = 0U;
    if (position == uart_rx_position)
        return;
    if (position < uart_rx_position)
    {
        receive_bytes(&uart_receiver, uart_rx_buffer + uart_rx_position,
                      BUFLENGTH - uart_rx_position);
        uart_rx_position = 0U;
    }
    if (position > uart_rx_position)
        receive_bytes(&uart_receiver, uart_rx_buffer + uart_rx_position,
                      position - uart_rx_position);
    uart_rx_position = position;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
        Usart1Receive_IDLE();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
        Usart1Receive_IDLE();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        HAL_UART_AbortReceive(uart);
        vision_receiver_init(&uart_receiver);
        uart_rx_position = 0U;
        HAL_UART_Receive_DMA(uart, uart_rx_buffer, BUFLENGTH);
    }
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
			append_CRC16_check_sum(PackToVisionUnion.UsartData, sizeof(PackToVisionUnion.PackToVision));

			/* 20240314 cac: 添加临界代码段 */
			taskENTER_CRITICAL();
			CDC_Transmit_FS(PackToVisionUnion.UsartData, sizeof(PackToVisionUnion.PackToVision));
			taskEXIT_CRITICAL();
			vTaskDelay(AUTO_TIME);
	
	}
	
}
