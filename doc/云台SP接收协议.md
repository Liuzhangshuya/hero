# 云台 SP 下行协议

上位机发送最终瞄准方向，C 板负责跟随和执行发射请求，不再调用板端弹道解算。
USB CDC 与 USART1（115200、8N1）使用相同协议，分别保存拆包状态；两路同时发送时，以最近收到的有效完整帧为准。

## 字节布局

固定 29 字节，无填充；float 为 IEEE 754 binary32，float 和 CRC 均为小端。

| 偏移 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0～1 | head | 2 字节 | `53 50`，即 `'S', 'P'` |
| 2 | mode | uint8_t | 0：不控制；1：跟随但不发射；2：跟随并请求发射 |
| 3～6 | yaw | float | 目标 yaw，rad |
| 7～10 | yaw_vel | float | yaw 角速度，rad/s |
| 11～14 | yaw_acc | float | yaw 角加速度，rad/s² |
| 15～18 | pitch | float | 目标 pitch，rad，向上为负 |
| 19～22 | pitch_vel | float | pitch 角速度，rad/s |
| 23～26 | pitch_acc | float | pitch 角加速度，rad/s² |
| 27～28 | crc16 | uint16_t | 对前 27 字节计算，低字节在前 |

CRC 沿用本工程原来的 CRC16 算法：初值 `0xFFFF`，反射多项式 `0x8408`（对应 `0x1021`），无末尾异或。
`"123456789"` 的校验结果是 `0x6F91`。解析时按字节偏移读取，不依赖 C 结构体对齐。

独立参考帧：`mode=2, yaw=1.25, yaw_vel=-2.5, yaw_acc=3.75, pitch=-0.5, pitch_vel=0.125, pitch_acc=-0.25`：

```text
53 50 02 00 00 A0 3F 00 00 20 C0 00 00 70 40
00 00 00 BF 00 00 00 3E 00 00 80 BE 8B 89
```

## 控制行为

遥控右拨杆上档或原有 Q 键自瞄入口选择 `GIMBAL_AUTO`。右拨杆中档继续使用原来的手动控制，下档继续零力并停止发射。

| 自瞄状态下的输入 | 云台 | 拨弹 |
| --- | --- | --- |
| 无有效指令 | 保持进入自瞄时的姿态 | 禁止 |
| mode=0 | 退出目标跟随，在退出时的姿态保持 | 取消未完成拨弹，禁止新拨弹 |
| mode=1 | 跟随上位机方向 | 禁止，包括鼠标长按、遥控长按和自动预拨弹 |
| mode=2 | 跟随上位机方向 | 摩擦轮已经手动开启时，执行原有单发定角拨弹；持续收到 mode=2 时逐发重复 |
| 最后有效帧已满 100 ms | 退出跟随，保持当前姿态，清除速度前馈 | 取消未完成拨弹，禁止新拨弹 |

模式选择、摩擦轮开关及原有裁判系统热量判断继续生效；初始化和云台校准期间禁止发射。
切出自瞄时取消正在执行的自动拨弹，手动发射恢复原有输入逻辑。

两帧之间按接收后的时间 `dt`（秒）计算：

```text
angle_target = angle + velocity * dt + 0.5 * acceleration * dt²
rate_feedforward = velocity + acceleration * dt
rate_target = angle_PID(angle_target) + rate_feedforward
```

角速度目标仍使用原有控制器输出限幅。Pitch 位置目标和速度前馈都受现有编码器软限位约束。
角度直接使用 rad，不作角度制换算，不再次翻转 pitch，也不再次添加弹道补偿。

接收端支持任意拆包、粘包、帧头跨接收块、连续串口数据和 DMA 环回。
帧头不符、CRC 错误、非法 mode、NaN 或无穷值都不会更新当前指令及其有效时间。
USART1 RX 已设为循环 DMA，IDLE、半满、全满中断共同消费数据；相应 CubeMX 配置也已同步。
USB 由 `Service_USB_Init()` 启动，接收回调在重新使能端点前完成解析。

## 验证

```sh
python3 tests/run_vision_tests.py
cc -std=c99 -Wall -Wextra -Werror -Iapplication tests/test_gimbal_pitch_control.c -lm -o /tmp/test_pitch
/tmp/test_pitch
make -f STM32Make.make DEBUG=1 -j4
```

主机测试覆盖协议参考帧、每个拆分点、逐位损坏、失步恢复、非法数值、超时与 tick 回绕，
并通过模拟硬件运行实际云台设置、拨弹控制和 USB/USART 接收代码。
这些检查不代表已经完成实机闭环或收发联调。

本次仅修改下行接收与控制决策。原来的 C 板上传协议和上传任务开关保留现状。
