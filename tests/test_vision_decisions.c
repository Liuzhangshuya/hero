/* Exercise the production setpoint functions, including static functions,
 * against simulated input. Hardware and clock access are the only stubs. */
#include <assert.h>
#include <stdio.h>
#include "vision_test_packet.h"
#include "../application/gimbal_task.c"

static RC_ctrl_t remote;
static motor_measure_t trigger, friction_r, friction_l, feeder;
static vision_receiver_t receiver;
static uint32_t now_ms;
static uint16_t heat, heat_limit = 100;
static uint8_t referee_offline;

const RC_ctrl_t *get_remote_control_point(void) { return &remote; }
const motor_measure_t *get_trigger_motor_measure_point(void) { return &trigger; }
const motor_measure_t *get_R_3508_measure_point(void) { return &friction_r; }
const motor_measure_t *get_L_3508_measure_point(void) { return &friction_l; }
const motor_measure_t *get_T_2006_measure_point(void) { return &feeder; }
bool_t toe_is_error(uint8_t toe) { return toe == REFEREE_TOE && referee_offline; }
void get_shoot_heat0_limit_and_heat0(uint16_t *limit, uint16_t *value)
{
    *limit = heat_limit;
    *value = heat;
}
void laser_on(void) {}
void laser_off(void) {}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return GPIO_PIN_SET;
}
uint8_t AutoGimbal_GetTarget(vision_target_t *target)
{
    memset(target, 0, sizeof(*target));
    return receiver.has_command &&
        vision_command_predict(&receiver.command, now_ms - receiver.received_ms, target);
}

static void close_to(float a, float b) { assert(fabsf(a - b) < 0.00001f); }

static void deliver(uint8_t mode, float yaw, float pitch, float pitch_vel)
{
    uint8_t packet[VISION_PACKET_SIZE];
    memcpy(packet, vision_golden_packet, sizeof(packet));
    packet[2] = mode;
    vision_test_float(packet, 3, yaw);
    vision_test_float(packet, 15, pitch);
    vision_test_float(packet, 19, pitch_vel);
    vision_test_checksum(packet);
    assert(vision_receiver_feed(&receiver, packet, sizeof(packet), now_ms) == 1);
}

static void cycle(void)
{
    GIMBAL_Set_Contorl(&gimbal_control);
    GIMBAL_control_loop(&gimbal_control);
    shoot_control_loop();
}

static void no_feeding(void)
{
    assert(shoot_control.shoot_mode == SHOOT_READY || shoot_control.shoot_mode == SHOOT_STOP);
    assert(!shoot_control.move_flag && shoot_control.speed_set == 0.0f);
    assert(shoot_control.set_total_ecd == shoot_control.total_ecd);
}

int main(void)
{
    const float rate_pid[] = {1.0f, 0.0f, 0.0f};
    gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;
    memset(&gimbal_control, 0, sizeof(gimbal_control));
    vision_receiver_init(&receiver);
    remote.rc.s[0] = RC_SW_UP;
    remote.rc.s[1] = RC_SW_MID;
    gimbal_control.gimbal_rc_ctrl = &remote;
    gimbal_control.last_gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
    gimbal_control.gimbal_motor_mode = GIMBAL_AUTO;
    pitch->min_relative_angle = -0.3f;
    pitch->max_relative_angle = 0.3f;
    gimbal_PID_init(&yaw->gimbal_motor_auto_angle_pid, 10, 0, 0, 0, 0);
    gimbal_PID_init(&pitch->gimbal_motor_auto_angle_pid, 10, 0, 0, 0, 0);
    PID_init(&yaw->gimbal_motor_AUTO_gyro_pid, PID_POSITION, rate_pid, 100, 0);
    PID_init(&pitch->gimbal_motor_AUTO_gyro_pid, PID_POSITION, rate_pid, 100, 0);
    gimbal_mode_change_control_transit(&gimbal_control);
    shoot_init();
    assert(get_gimbal_control_point() == &gimbal_control);

    cycle(); /* No command must neither acquire a target nor fire. */
    assert(!gimbal_control.vision_control_active);
    no_feeding();

    deliver(VISION_MODE_TRACK, 1.25f, -0.2f, 0.125f);
    cycle();
    close_to(yaw->absolute_angle_set, 1.25f);
    close_to(pitch->absolute_angle_set, -0.2f); /* Up stays negative. */
    close_to(yaw->motor_gyro_set, -2.5f); /* Actual rate feedforward, rad/s. */
    close_to(pitch->motor_gyro_set, 0.125f);
    no_feeding();

    /* A fire request cannot turn on previously disarmed friction wheels. */
    deliver(VISION_MODE_FIRE, 1.25f, -0.2f, 0.125f);
    cycle();
    assert(shoot_control.shoot_mode == SHOOT_STOP);
    no_feeding();

    /* Existing manual arming permits a mode=2 index. */
    remote.rc.s[1] = RC_SW_UP;
    cycle();
    assert(shoot_control.shoot_mode == SHOOT_BULLET && shoot_control.move_flag);
    int64_t first_index = shoot_control.set_total_ecd;
    assert(first_index > shoot_control.total_ecd);

    /* Finish one physical index; sustained mode=2 starts the next index. */
    trigger.ecd = (uint16_t)(first_index % 8192);
    trigger.last_ecd = trigger.ecd;
    shoot_control.round_cnt = (int16_t)(first_index / 8192);
    cycle();
    assert(!shoot_control.move_flag);
    cycle();
    assert(shoot_control.move_flag && shoot_control.set_total_ecd > first_index);

    /* Revoking fire cancels an unfinished index and overrides manual long-press. */
    deliver(VISION_MODE_TRACK, 1.25f, -0.2f, 0.125f);
    remote.mouse.press_l = remote.mouse.press_r = 1;
    remote.rc.s[1] = RC_SW_DOWN;
    shoot_control.press_l_time = PRESS_LONG_TIME;
    shoot_control.press_r_time = PRESS_LONG_TIME;
    shoot_control.rc_s_time = RC_S_LONG_TIME;
    cycle();
    no_feeding();
    cycle();
    no_feeding();

    /* Heat protection overrides mode=2, without entering trigger preload. */
    remote.rc.s[1] = RC_SW_MID;
    deliver(VISION_MODE_FIRE, 1.25f, -0.2f, 0.125f);
    heat = heat_limit + 1;
    cycle();
    no_feeding();
    heat = 0;
    cycle();
    assert(shoot_control.move_flag);

    /* At 100 ms stale input stops an in-progress shot and latches this pose. */
    now_ms += 99;
    cycle();
    assert(gimbal_control.vision_control_active && shoot_control.move_flag);
    yaw->absolute_angle = 0.4f;
    pitch->absolute_angle = -0.1f;
    ++now_ms;
    cycle();
    no_feeding();
    assert(!gimbal_control.vision_control_active);
    close_to(yaw->absolute_angle_set, 0.4f);
    close_to(pitch->absolute_angle_set, -0.1f);
    close_to(yaw->auto_rate_feedforward, 0);
    close_to(pitch->auto_rate_feedforward, 0);
    yaw->absolute_angle = 0.5f;
    pitch->absolute_angle = -0.15f;
    cycle();
    close_to(yaw->absolute_angle_set, 0.4f); /* Hold; do not chase current feedback. */
    close_to(pitch->absolute_angle_set, -0.1f);

    /* mode=0 immediately revokes a fresh mode=2 request too. */
    deliver(VISION_MODE_FIRE, 0.2f, -0.2f, -0.125f);
    cycle();
    assert(shoot_control.move_flag);
    deliver(VISION_MODE_DISABLED, 0.2f, -0.2f, -0.125f);
    cycle();
    no_feeding();
    assert(!gimbal_control.vision_control_active);

    /* Target position and velocity must both respect the pitch stop. */
    pitch->absolute_angle = 0.0f;
    pitch->relative_angle = -0.3f;
    deliver(VISION_MODE_TRACK, 0.2f, -0.5f, -1.0f);
    cycle();
    close_to(pitch->absolute_angle_set, 0);
    close_to(pitch->auto_rate_feedforward, 0);
    no_feeding();

    yaw->absolute_angle = 3.1f;
    pitch->relative_angle = 0;
    deliver(VISION_MODE_TRACK, -3.1f, -0.2f, 0);
    cycle();
    close_to(yaw->auto_add_angle, 2.0f * PI - 6.2f);
    close_to(yaw->absolute_angle_set, -3.1f);

    /* Leaving AUTO cancels its index. A recent PC fire packet does not create
     * a manual-mode fire request; normal manual trigger input still works. */
    deliver(VISION_MODE_FIRE, 0.2f, -0.2f, 0);
    cycle();
    assert(shoot_control.move_flag);
    remote.mouse.press_l = remote.mouse.press_r = 0;
    shoot_control.press_l_time = shoot_control.press_r_time = shoot_control.rc_s_time = 0;
    shoot_control.press_l = shoot_control.press_r = 0;
    gimbal_control.gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
    shoot_control_loop();
    no_feeding();
    shoot_control.press_l = 1;
    shoot_control.last_press_l = 0;
    shoot_control_loop();
    assert(shoot_control.move_flag);

    gimbal_control.gimbal_motor_mode = GIMBAL_ZERO_FORCE;
    shoot_control_loop();
    no_feeding();
    assert(shoot_control.shoot_mode == SHOOT_STOP);

    puts("Production gimbal/shooter: modes, arming, heat, timeout, stops, feedforward and manual override passed.");
    return 0;
}
