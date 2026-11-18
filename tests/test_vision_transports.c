#include <assert.h>
#include <stdio.h>
#include "AutoGimbal.h"
#include "vision_test_packet.h"

/* CMSIS is already included. Replace only the hardware interrupt-mask calls
 * for this host translation unit, leaving production receive code intact. */
static uint32_t irq_mask;
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask = 1U)
#define __set_PRIMASK(mask) (irq_mask = (mask))
#include "../application/AutoGimbal.c"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
static USART_TypeDef uart_registers;
static DMA_Stream_TypeDef dma_registers;
static uint32_t now_ms;
static unsigned write_position;

uint32_t HAL_GetTick(void) { return now_ms; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(uart == &huart1 && data == uart_rx_buffer && length == BUFLENGTH);
    dma_registers.NDTR = length;
    write_position = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    return HAL_OK;
}

static void serial_write(const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; ++i)
    {
        uart_rx_buffer[write_position++] = data[i];
        if (write_position == BUFLENGTH) write_position = 0;
        dma_registers.NDTR = BUFLENGTH - write_position;
        if (write_position == BUFLENGTH / 2)
            HAL_UART_RxHalfCpltCallback(&huart1);
        if (write_position == 0)
            HAL_UART_RxCpltCallback(&huart1);
    }
}

int main(void)
{
    vision_target_t target;
    uint8_t noise[BUFLENGTH], bad[VISION_PACKET_SIZE];
    memset(noise, 0xff, sizeof(noise));
    huart1.Instance = &uart_registers;
    hdma_usart1_rx.Instance = &dma_registers;
    now_ms = 1000;

    /* Every possible DMA start offset, including half/full wrap boundaries. */
    for (size_t offset = 0; offset < BUFLENGTH; ++offset)
    {
        AUTO_control_init();
        assert(!AutoGimbal_GetTarget(&target));
        serial_write(noise, offset);
        Usart1Receive_IDLE();
        serial_write(vision_golden_packet, VISION_PACKET_SIZE);
        Usart1Receive_IDLE();
        assert(AutoGimbal_GetTarget(&target));
        assert(target.mode == VISION_MODE_FIRE && target.pitch == -0.5f);
        assert(irq_mask == 0);
    }

    /* A continuous stream needs no IDLE interrupt to publish complete frames. */
    AUTO_control_init();
    for (unsigned i = 0; i < 1000; ++i)
        serial_write(vision_golden_packet, VISION_PACKET_SIZE);
    Usart1Receive_IDLE();
    assert(AutoGimbal_GetTarget(&target) && uart_receiver.length == 0);

    AUTO_control_init();
    AutoGimbal_ReceiveUSB(vision_golden_packet, 13);
    serial_write(vision_golden_packet + 13, 16);
    Usart1Receive_IDLE();
    assert(!AutoGimbal_GetTarget(&target));
    AutoGimbal_ReceiveUSB(vision_golden_packet + 13, 16);
    assert(AutoGimbal_GetTarget(&target));
    assert(target.yaw == 1.25f && target.pitch == -0.5f);

    memcpy(bad, vision_golden_packet, sizeof(bad));
    bad[28] ^= 1;
    now_ms += 99;
    AutoGimbal_ReceiveUSB(bad, sizeof(bad));
    assert(AutoGimbal_GetTarget(&target));
    ++now_ms;
    assert(!AutoGimbal_GetTarget(&target));
    assert(target.mode == VISION_MODE_DISABLED && target.yaw_vel == 0);

    /* Receive timestamp arithmetic must survive a 32-bit HAL tick rollover. */
    now_ms = UINT32_MAX - 20U;
    AutoGimbal_ReceiveUSB(vision_golden_packet, VISION_PACKET_SIZE);
    now_ms = 10;
    assert(AutoGimbal_GetTarget(&target));
    now_ms = 79;
    assert(!AutoGimbal_GetTarget(&target));

    /* Recover a damaged serial link without completing its old partial frame. */
    AUTO_control_init();
    serial_write(vision_golden_packet, 13);
    Usart1Receive_IDLE();
    HAL_UART_ErrorCallback(&huart1);
    assert(uart_rx_position == 0 && uart_receiver.length == 0);
    serial_write(vision_golden_packet, VISION_PACKET_SIZE);
    Usart1Receive_IDLE();
    assert(AutoGimbal_GetTarget(&target));
    puts("Production USB/UART RX: DMA offsets/wrap, continuous stream, isolation, CRC, timeout and recovery passed.");
    return 0;
}
