#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define VISION_PACKET_SIZE 29U
#define VISION_CRC_OFFSET 27U
#define VISION_COMMAND_TIMEOUT_MS 100U

enum vision_mode
{
    VISION_MODE_DISABLED = 0,
    VISION_MODE_TRACK = 1,
    VISION_MODE_FIRE = 2
};

/* Wire offsets are decoded explicitly; this structure needs no packing.
 * All angles use radians, with negative pitch pointing upwards. */
typedef struct
{
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} vision_command_t;

typedef struct
{
    uint8_t buffer[VISION_PACKET_SIZE];
    uint8_t length;
    uint8_t has_command;
    vision_command_t command;
    uint32_t received_ms;
    uint32_t last_data_ms;
} vision_receiver_t;

typedef struct
{
    uint8_t mode;
    float yaw;
    float pitch;
    float yaw_vel;
    float pitch_vel;
} vision_target_t;

uint16_t vision_crc16(const uint8_t *data, size_t length);
void vision_receiver_init(vision_receiver_t *receiver);
/* Separate receiver instances are required for independent byte streams. */
size_t vision_receiver_feed(vision_receiver_t *receiver, const uint8_t *data,
                            size_t length, uint32_t received_ms);
uint8_t vision_command_predict(const vision_command_t *command, uint32_t age_ms,
                               vision_target_t *target);

#endif
