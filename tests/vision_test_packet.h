#ifndef VISION_TEST_PACKET_H
#define VISION_TEST_PACKET_H

#include <stdint.h>
#include <string.h>
#include "vision_protocol.h"

/* Independently generated with Python struct.pack('<2sB6f', ...) and
 * bit-reflected binascii.crc_hqx(..., 0xffff). CRC = 0x898b. */
static const uint8_t vision_golden_packet[VISION_PACKET_SIZE] = {
    0x53, 0x50, 0x02, 0x00, 0x00, 0xa0, 0x3f, 0x00, 0x00, 0x20,
    0xc0, 0x00, 0x00, 0x70, 0x40, 0x00, 0x00, 0x00, 0xbf, 0x00,
    0x00, 0x00, 0x3e, 0x00, 0x00, 0x80, 0xbe, 0x8b, 0x89
};

static inline void vision_test_checksum(uint8_t *packet)
{
    uint16_t crc = vision_crc16(packet, VISION_CRC_OFFSET);
    packet[27] = (uint8_t)crc;
    packet[28] = (uint8_t)(crc >> 8);
}

static inline void vision_test_float(uint8_t *packet, unsigned offset, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    packet[offset] = (uint8_t)bits;
    packet[offset + 1] = (uint8_t)(bits >> 8);
    packet[offset + 2] = (uint8_t)(bits >> 16);
    packet[offset + 3] = (uint8_t)(bits >> 24);
}

#endif
