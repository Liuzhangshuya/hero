#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "vision_test_packet.h"

static void close_to(float a, float b) { assert(fabsf(a - b) < 0.00001f); }

int main(void)
{
    vision_receiver_t rx, other;
    vision_target_t target;
    uint8_t packet[VISION_PACKET_SIZE], stream[2 * VISION_PACKET_SIZE + 5];
    size_t split, i, bit;
    unsigned mode;

    assert(vision_crc16((const uint8_t *)"123456789", 9) == 0x6f91U);
    assert(vision_crc16(vision_golden_packet, 27) == 0x898bU);
    for (split = 0; split <= VISION_PACKET_SIZE; ++split)
    {
        vision_receiver_init(&rx);
        size_t accepted = vision_receiver_feed(&rx, vision_golden_packet, split, 1000);
        accepted += vision_receiver_feed(&rx, vision_golden_packet + split,
                                         VISION_PACKET_SIZE - split, 1001);
        assert(accepted == 1 && rx.has_command && rx.length == 0);
        assert(rx.command.mode == VISION_MODE_FIRE && rx.command.crc16 == 0x898b);
        close_to(rx.command.yaw, 1.25f);
        close_to(rx.command.yaw_vel, -2.5f);
        close_to(rx.command.yaw_acc, 3.75f);
        close_to(rx.command.pitch, -0.5f);
        close_to(rx.command.pitch_vel, 0.125f);
        close_to(rx.command.pitch_acc, -0.25f);
    }

    /* Every single-bit corruption must leave the last valid command and its
     * freshness timestamp unchanged, then recover on the next good frame. */
    for (i = 0; i < VISION_PACKET_SIZE; ++i)
        for (bit = 0; bit < 8; ++bit)
        {
            vision_receiver_init(&rx);
            vision_receiver_feed(&rx, vision_golden_packet, VISION_PACKET_SIZE, 0);
            memcpy(packet, vision_golden_packet, sizeof(packet));
            packet[i] ^= (uint8_t)(1U << bit);
            assert(vision_receiver_feed(&rx, packet, sizeof(packet), 50) == 0);
            assert(rx.received_ms == 0 && rx.command.mode == VISION_MODE_FIRE);
            assert(vision_receiver_feed(&rx, vision_golden_packet, VISION_PACKET_SIZE, 51) == 1);
        }

    memset(stream, 0xff, sizeof(stream));
    stream[0] = 'S'; stream[1] = 'P'; /* A false candidate before two real frames. */
    memcpy(stream + 5, vision_golden_packet, VISION_PACKET_SIZE);
    memcpy(stream + 5 + VISION_PACKET_SIZE, vision_golden_packet, VISION_PACKET_SIZE);
    vision_receiver_init(&rx);
    assert(vision_receiver_feed(&rx, stream, sizeof(stream), 0) == 2);

    /* Missing bytes, including either header byte and either CRC byte. */
    for (i = 0; i < VISION_PACKET_SIZE; ++i)
    {
        memcpy(stream, vision_golden_packet, i);
        memcpy(stream + i, vision_golden_packet + i + 1, VISION_PACKET_SIZE - i - 1);
        memcpy(stream + VISION_PACKET_SIZE - 1, vision_golden_packet, VISION_PACKET_SIZE);
        vision_receiver_init(&rx);
        assert(vision_receiver_feed(&rx, stream, 2 * VISION_PACKET_SIZE - 1, 0) == 1);
    }

    /* A payload float may contain SP without beginning a new frame. */
    memcpy(packet, vision_golden_packet, sizeof(packet));
    packet[7] = 'S'; packet[8] = 'P';
    vision_test_checksum(packet);
    vision_receiver_init(&rx);
    assert(vision_receiver_feed(&rx, packet, sizeof(packet), 0) == 1);

    for (mode = 0; mode < 4; ++mode)
    {
        memcpy(packet, vision_golden_packet, sizeof(packet));
        packet[2] = (uint8_t)mode;
        vision_test_checksum(packet);
        vision_receiver_init(&rx);
        assert(vision_receiver_feed(&rx, packet, sizeof(packet), 0) == (mode <= 2));
        if (mode <= 2)
        {
            assert(vision_command_predict(&rx.command, 0, &target) == (mode != 0));
            assert(target.mode == mode);
        }
    }

    /* Valid-CRC NaN and +/-infinity must be rejected in each float field. */
    for (i = 3; i < VISION_CRC_OFFSET; i += 4)
    {
        const float invalid[] = {NAN, INFINITY, -INFINITY};
        for (bit = 0; bit < 3; ++bit)
        {
            memcpy(packet, vision_golden_packet, sizeof(packet));
            vision_test_float(packet, (unsigned)i, invalid[bit]);
            vision_test_checksum(packet);
            vision_receiver_init(&rx);
            assert(vision_receiver_feed(&rx, packet, sizeof(packet), 0) == 0);
        }
    }

    vision_receiver_init(&rx);
    vision_receiver_feed(&rx, vision_golden_packet, VISION_PACKET_SIZE, 0);
    assert(vision_command_predict(&rx.command, 50, &target));
    close_to(target.yaw, 1.1296875f);
    close_to(target.pitch, -0.4940625f);
    close_to(target.yaw_vel, -2.3125f);
    close_to(target.pitch_vel, 0.1125f);
    assert(vision_command_predict(&rx.command, 99, &target));
    assert(!vision_command_predict(&rx.command, 100, &target));
    assert(target.mode == 0 && target.yaw_vel == 0 && target.pitch_vel == 0);
    assert(!vision_command_predict(&rx.command, UINT32_MAX, &target));
    rx.command.yaw = FLT_MAX;
    rx.command.yaw_vel = FLT_MAX;
    assert(!vision_command_predict(&rx.command, 99, &target));

    /* Independent transports cannot finish one another's partial frame. */
    vision_receiver_init(&rx);
    vision_receiver_init(&other);
    assert(!vision_receiver_feed(&rx, vision_golden_packet, 13, 1000));
    assert(!vision_receiver_feed(&other, vision_golden_packet + 13, 16, 1001));
    assert(vision_receiver_feed(&rx, vision_golden_packet + 13, 16, 1002) == 1);

    /* A long inter-chunk gap discards an unfinished frame, even on tick wrap. */
    vision_receiver_init(&rx);
    vision_receiver_feed(&rx, vision_golden_packet, 13, UINT32_MAX - 20U);
    assert(vision_receiver_feed(&rx, vision_golden_packet + 13, 16, 10) == 1);
    assert(vision_command_predict(&rx.command, (uint32_t)(20U - rx.received_ms), &target));
    vision_receiver_init(&rx);
    vision_receiver_feed(&rx, vision_golden_packet, 13, 1000);
    assert(!vision_receiver_feed(&rx, vision_golden_packet + 13, 16, 1100));
    assert(!rx.has_command);
    assert(vision_receiver_feed(&rx, vision_golden_packet, VISION_PACKET_SIZE, 1101) == 1);

    /* Model a continuous stream split arbitrarily at USB and DMA boundaries. */
    vision_receiver_init(&rx);
    uint8_t many[29000];
    size_t accepted = 0;
    for (i = 0; i < sizeof(many); i += VISION_PACKET_SIZE)
        memcpy(many + i, vision_golden_packet, VISION_PACKET_SIZE);
    for (i = 0; i < sizeof(many); )
    {
        size_t chunk = (i * 17U + 1U) % 73U + 1U;
        if (chunk > sizeof(many) - i) chunk = sizeof(many) - i;
        accepted += vision_receiver_feed(&rx, many + i, chunk, 2000 + (uint32_t)(i / 100));
        i += chunk;
    }
    assert(accepted == 1000 && rx.length == 0);
    puts("SP protocol: golden CRC, all splits, corruption, resync, modes, prediction and timeout passed.");
    return 0;
}
