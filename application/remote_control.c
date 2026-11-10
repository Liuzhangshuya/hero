/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */



#include "bsp_laser.h"
#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"

extern int8_t Laser_cmd = 0; // 

extern int8_t tem_power = 0;
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

Keyboard_Info_t Keyboard;
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
static void Set_Laser(void);

void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
                detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[0]);
//							  KEY_Function_Ctrl();
//                RC_Function_Ctrl();
//                Set_Laser();
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                //记录数据接收时间
                detect_hook(DBUS_TOE);
                // sbus_to_usart1(sbus_rx_buf[1]);
//                RC_Function_Ctrl();
//                Set_Laser();
            }
        }
    }

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->kb.key_code   =  rc_ctrl->key.v;
}


/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}
/**
 * @brief 关闭超电
 * @param
 */
void Laser_KEY_OFF(void) { Laser_cmd = 1; }

/**
 * @brief 开启超电
 * @param
 */
void Laser_KEY_ON(void) { Laser_cmd = 0; }


/**
 * @brief 遥控器功能切换控制（遥控器操作的各种功能）
 * @param
 */
void RC_Function_Ctrl(void)
{
    switch (RC_Judge_sw_Change())
    {
    case SW_UP:
        Laser_KEY_ON();
        // SuperCap_KEY_ON();
        tem_power++;
        //    SPIN_ON();
        break;
    case SW_DOWM:

        Laser_KEY_OFF();
        // SuperCap_KEY_OFF();
        tem_power--;
        //    SPIN_OFF();
        //      if(!CHAS_SPIN_Report())//遥控状态下如果没有启动陀螺则拨轮往下为切换防御模式
        //        System.Defense_Mode =
        //        SPIN_Defense_Mode_Switch(System.Defense_Mode);
        break;
    case SW_MID:
        //    SPIN_CLOSING_Judge();
        break;
    }
}

/**
 * @brief 拨轮的判断
 * @param
 */
RC_SW_Change_t RC_Judge_sw_Change(void)
{
    static int func_change_cnt = 0;
    RC_SW_Change_t res;
    switch (RC_SW)
    {
    case RC_SW_VALUE_MIN:
        func_change_cnt++;
        if (func_change_cnt > RC_S_TIM)
            res = SW_UP;
        break;
    case RC_SW_VALUE_MAX:
        func_change_cnt++;
        if (func_change_cnt > RC_S_TIM)
            res = SW_DOWM;
        break;
    default:
        func_change_cnt = 0;
        res = SW_MID;
        break;
    }
    return res;
}


/**
 * @brief 判断按键是否按下
 * @note
 * @param
 */
void FirstGetInto_KEY_PRESS(KEY_Info_t *str)
{
    if (str->prev_KEY_PRESS != str->KEY_PRESS)
    {
        str->state_cnt = 0;
        str->prev_KEY_PRESS = str->KEY_PRESS;
    }
}
/**
 * @brief 判断按键
 * @note
 * @param
 */

void KEY_State_Judge(KEY_Info_t *str, uint8_t KEY_PRESS, int change_tim, int long_change_tim)
{
    str->KEY_PRESS = KEY_PRESS;
    FirstGetInto_KEY_PRESS(str);
    switch (KEY_PRESS)
    {
    case KEY_UP:
    {
        if (str->prev_State != UP)
        {
            str->state_cnt++;
            if (str->state_cnt >= change_tim)
            {
                str->State = RELAX;
                str->prev_State = RELAX;
                if (str->state_cnt >= change_tim + 1) // 抬起不分长短抬
                {
                    str->State = UP;
                    str->prev_State = UP;
                }
            }
        }
        else
        {
            str->state_cnt = 0;
        }
    }
    break;

    case KEY_DOWN:
    {
        if (str->prev_State != DOWN)
        {
            str->state_cnt++;
            if (str->state_cnt >= change_tim)
            {
                str->State = PRESS;
                str->prev_State = PRESS;
                if (str->state_cnt >= change_tim + 1)
                {
                    str->State = SHORT_DOWN;
                    str->prev_State = SHORT_DOWN;
                    if (str->state_cnt >= long_change_tim)
                    {
                        str->State = DOWN;
                        str->prev_State = DOWN;
                    }
                }
            }
        }
        else
        {
            str->state_cnt = 0;
        }
    }
    break;
    }
}

void Key_SHIFT(void)
{
    KEY_State_Judge(&Keyboard.SHIFT, KEY_SHIFT, CHANGE_TIM, LONG_CHANGE_TIM_SHIFT);
    switch (Keyboard.SHIFT.State)
    {
    case UP:
//        SuperCap_KEY_OFF();
        break;
    case PRESS:
        //      if(KEY_CTRL)
        //        IF_CarHalfMove_Switch();//斜向模式的跳变开关
//        SuperCap_KEY_ON();
        break;
    case SHORT_DOWN:
        break;    
    case DOWN:
        break;
    case RELAX:
        break;
    }
}
void Set_Laser(void)
{
    if (Laser_cmd == 0)
    {
        laser_on();
    }
    if (Laser_cmd == 1)
    {
        laser_off();
    }
}