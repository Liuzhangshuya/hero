#include "vision_protocol.h"

#include <math.h>
#include <string.h>

#define VISION_PI 3.14159265358979323846f

uint16_t vision_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    unsigned bit;
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0x8408U : 0U);
    }
    return crc;
}

static float read_float_le(const uint8_t *data)
{
    uint32_t bits = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint8_t decode(const uint8_t *data, vision_command_t *command)
{
    uint16_t crc = (uint16_t)data[27] | ((uint16_t)data[28] << 8);
    if (data[0] != 'S' || data[1] != 'P' || data[2] > VISION_MODE_FIRE ||
        crc != vision_crc16(data, VISION_CRC_OFFSET))
        return 0U;

    memset(command, 0, sizeof(*command));
    command->mode = data[2];
    command->yaw = read_float_le(data + 3);
    command->yaw_vel = read_float_le(data + 7);
    command->yaw_acc = read_float_le(data + 11);
    command->pitch = read_float_le(data + 15);
    command->pitch_vel = read_float_le(data + 19);
    command->pitch_acc = read_float_le(data + 23);
    command->crc16 = crc;
    return isfinite(command->yaw) && isfinite(command->yaw_vel) &&
           isfinite(command->yaw_acc) && isfinite(command->pitch) &&
           isfinite(command->pitch_vel) && isfinite(command->pitch_acc);
}

void vision_receiver_init(vision_receiver_t *receiver)
{
    memset(receiver, 0, sizeof(*receiver));
}

size_t vision_receiver_feed(vision_receiver_t *receiver, const uint8_t *data,
                            size_t length, uint32_t received_ms)
{
    size_t i, accepted = 0U;
    if (receiver == NULL || data == NULL)
        return 0U;
    if (length == 0U)
        return 0U;
    if ((uint32_t)(received_ms - receiver->last_data_ms) >= VISION_COMMAND_TIMEOUT_MS)
        receiver->length = 0U;
    receiver->last_data_ms = received_ms;

    for (i = 0; i < length; ++i)
    {
        vision_command_t command;
        receiver->buffer[receiver->length++] = data[i];
        /* Slide one byte after a bad candidate, preserving an embedded header
         * and partial frames across USB packets and DMA boundaries. */
        while (receiver->length > 0U)
        {
            if (receiver->buffer[0] == 'S' &&
                (receiver->length == 1U || receiver->buffer[1] == 'P'))
            {
                if (receiver->length < VISION_PACKET_SIZE)
                    break;
                if (decode(receiver->buffer, &command))
                {
                    receiver->command = command;
                    receiver->received_ms = received_ms;
                    receiver->has_command = 1U;
                    receiver->length = 0U;
                    ++accepted;
                    break;
                }
            }
            --receiver->length;
            memmove(receiver->buffer, receiver->buffer + 1, receiver->length);
        }
    }
    return accepted;
}

static float wrap_angle(float angle)
{
    float wrapped = fmodf(angle, 2.0f * VISION_PI);
    if (wrapped > VISION_PI)
        wrapped -= 2.0f * VISION_PI;
    else if (wrapped < -VISION_PI)
        wrapped += 2.0f * VISION_PI;
    return wrapped;
}

uint8_t vision_command_predict(const vision_command_t *command, uint32_t age_ms,
                               vision_target_t *target)
{
    float dt;
    memset(target, 0, sizeof(*target));
    if (age_ms >= VISION_COMMAND_TIMEOUT_MS ||
        (command->mode != VISION_MODE_TRACK && command->mode != VISION_MODE_FIRE))
        return 0U;

    dt = age_ms * 0.001f;
    target->yaw = command->yaw + command->yaw_vel * dt + 0.5f * command->yaw_acc * dt * dt;
    target->pitch = command->pitch + command->pitch_vel * dt + 0.5f * command->pitch_acc * dt * dt;
    target->yaw_vel = command->yaw_vel + command->yaw_acc * dt;
    target->pitch_vel = command->pitch_vel + command->pitch_acc * dt;
    if (!isfinite(target->yaw) || !isfinite(target->pitch) ||
        !isfinite(target->yaw_vel) || !isfinite(target->pitch_vel))
    {
        memset(target, 0, sizeof(*target));
        return 0U;
    }
    target->yaw = wrap_angle(target->yaw);
    target->pitch = wrap_angle(target->pitch);
    target->mode = command->mode;
    return 1U;
}
