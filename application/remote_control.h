/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;
        __packed union
        {
                uint16_t key_code;
                __packed struct
                {
                        uint16_t W : 1;
                        uint16_t S : 1;
                        uint16_t A : 1;
                        uint16_t D : 1;
                        uint16_t SHIFT : 1;
                        uint16_t CTRL : 1;
                        uint16_t Q : 1;
                        uint16_t E : 1;
                        uint16_t R : 1;
                        uint16_t F : 1;
                        uint16_t G : 1;
                        uint16_t Z : 1;
                        uint16_t X : 1;
                        uint16_t C : 1;
                        uint16_t V : 1;
                        uint16_t B : 1;
                } bit;
        } kb;
} RC_ctrl_t;
typedef struct
{
        uint16_t rc_keyQ_time;     // 按键Q的时间
        uint16_t rc_keyE_time;     // 按键E的时间
        uint16_t rc_keyR_time;     // 按键R的时间
        uint16_t rc_keyF_time;     // 按键F的时间
        uint16_t rc_keyG_time;     // 按键G的时间
        uint16_t rc_keyZ_time;     // 按键Z的时间
        uint16_t rc_keyX_time;     // 按键X的时间
        uint16_t rc_keyC_time;     // 按键C的时间
        uint16_t rc_keyV_time;     // 按键V的时间
        uint16_t rc_keyB_time;     // 按键B的时间
        uint16_t rc_keyShift_time; // 按键Shift的时间
        uint16_t rc_keyCtrl_time;  // 按键Ctrl的时间
        uint16_t mouse_left_time;
        uint16_t mouse_right_time;

        uint16_t rc_keyQ_E_time;     // 同时按QE的时间
        uint16_t rc_keyShift_E_time; // 同时按Shift和E的时间
} RC_ctrl_time_t;
/* -----------------------  按键 ----------------------------------- */

typedef enum
{
        SW_MID = 0,
        SW_UP,
        SW_DOWM,
} RC_SW_Change_t;

/* 检测键盘按键状态 */
#define KEY_UP 0x00
#define KEY_DOWN 0x01

/*按键状态枚举*/
typedef enum
{
        UP = 0,     // 0
        PRESS,      // 0->1
        SHORT_DOWN, // 1短按
        DOWN,       // 1长按
        RELAX,      // 1->0
} KEY_State_t;

/*单独按键信息*/
typedef struct
{
        int state_cnt;
        KEY_State_t State;
        KEY_State_t prev_State;
        uint8_t prev_KEY_PRESS;
        uint8_t KEY_PRESS;
} KEY_Info_t;

/*按键时间 长短按的判断*/
#define CHANGE_TIM 5                // ms
#define LONG_CHANGE_TIM_W 1000      // ms
#define LONG_CHANGE_TIM_S 1000      // ms
#define LONG_CHANGE_TIM_A 1000      // ms
#define LONG_CHANGE_TIM_D 1000      // ms
#define LONG_CHANGE_TIM_Q 500       // ms
#define LONG_CHANGE_TIM_E 500       // ms
#define LONG_CHANGE_TIM_R 500       // ms
#define LONG_CHANGE_TIM_F 500       // ms
#define LONG_CHANGE_TIM_G 500       // ms
#define LONG_CHANGE_TIM_Z 500       // ms
#define LONG_CHANGE_TIM_X 500       // ms
#define LONG_CHANGE_TIM_C 500       // ms
#define LONG_CHANGE_TIM_V 500       // ms
#define LONG_CHANGE_TIM_B 300       // ms
#define LONG_CHANGE_TIM_CTRL 1000   // ms
#define LONG_CHANGE_TIM_SHIFT 500   // ms
#define LONG_CHANGE_TIM_MOUSE_L 350 // ms
#define LONG_CHANGE_TIM_MOUSE_R 500 // ms

/*遥控器消抖判断时间*/
#define RC_S_TIM 50 // ms

/*总体键盘按键信息*/
typedef struct
{
        KEY_Info_t W;
        KEY_Info_t S;
        KEY_Info_t A;
        KEY_Info_t D;
        KEY_Info_t Q; // 左转90度
        KEY_Info_t E; // 右转90度
        KEY_Info_t R; // 打开弹舱
        KEY_Info_t F; // 小陀螺
        KEY_Info_t G; // 快速抬头
        KEY_Info_t Z;
        KEY_Info_t X;
        KEY_Info_t C;     // 视觉模式切换
        KEY_Info_t V;     // 从右后转180度
        KEY_Info_t B;     // 对位模式？
        KEY_Info_t CTRL;  // 机械模式切换
        KEY_Info_t SHIFT; // 极速模式切换
        KEY_Info_t Mouse_L;
        KEY_Info_t Mouse_R;
        //  KEY_MoveInfo_t MoveInfo;
        //  Mouse_SF_t MouseSF;
} Keyboard_Info_t;
extern Keyboard_Info_t Keyboard;

#define RC_SW rc_ctrl.rc.ch[4]
#define RC_SW_VALUE_MIN ((int16_t)-660)
#define RC_SW_VALUE_OFFSET ((int16_t)0)
#define RC_SW_VALUE_MAX ((int16_t)660)

#define KEY_W rc_ctrl.kb.bit.W
#define KEY_S rc_ctrl.kb.bit.S
#define KEY_A rc_ctrl.kb.bit.A
#define KEY_D rc_ctrl.kb.bit.D
#define KEY_SHIFT rc_ctrl.kb.bit.SHIFT
#define KEY_CTRL rc_ctrl.kb.bit.CTRL
#define KEY_Q rc_ctrl.kb.bit.Q
#define KEY_E rc_ctrl.kb.bit.E
#define KEY_R rc_ctrl.kb.bit.R
#define KEY_F rc_ctrl.kb.bit.F
#define KEY_G rc_ctrl.kb.bit.G
#define KEY_Z rc_ctrl.kb.bit.Z
#define KEY_X rc_ctrl.kb.bit.X
#define KEY_C rc_ctrl.kb.bit.C
#define KEY_V rc_ctrl.kb.bit.V
#define KEY_B rc_ctrl.kb.bit.B

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);

// 按键判断函数
RC_SW_Change_t RC_Judge_sw_Change(void);

void RC_Function_Ctrl(void);

void Key_SHIFT(void);
void SuperCap_KEY_ON(void);
void SuperCap_KEY_OFF(void);
void KEY_State_Judge(KEY_Info_t *str, uint8_t KEY_PRESS, int change_tim, int long_change_tim);
#endif
