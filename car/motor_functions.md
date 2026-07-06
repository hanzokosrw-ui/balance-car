# 电机驱动函数说明

本文档说明 `car/Core/Src/motor.c` 和 `car/Core/Inc/motor.h` 中电机驱动函数的用途和使用方法。

## 基本约定

- 左电机使用 `PA6/TIM3_CH1` 和 `PA7/TIM3_CH2`。
- 右电机使用 `PB0/TIM3_CH3` 和 `PB1/TIM3_CH4`。
- PWM 频率由 `TIM3` 决定，目前 `Period = 3599`，定时器时钟 72 MHz 时 PWM 频率为 20 kHz。
- 函数参数中的占空比单位是百分比，范围是 `-100` 到 `100`。
- 正数表示小车前进方向，负数表示小车后退方向。
- 右轮方向已经在驱动内部反相处理，所以 `Motor_SetDuty(60, 60)` 表示左右轮一起向前转。

## Motor_Init

功能：启动 TIM3 的 4 路 PWM 输出通道，并把电机输出清零。

使用位置：在 `MX_TIM3_Init()` 之后调用一次。

返回值：

- `HAL_OK`：4 路 PWM 启动成功。
- 其他 HAL 状态：某一路 PWM 启动失败。

典型用途：程序初始化阶段调用。如果返回值不是 `HAL_OK`，说明 PWM 没有正常启动，应进入错误处理。

示例：

```c
if (Motor_Init() != HAL_OK) //by codex
{ //by codex
  Error_Handler(); //by codex
} //by codex
```

## Motor_Stop

功能：让电机滑行停止。

它会把四个 PWM 比较值都设为 0：

- 左轮：`IN1 = 0`，`IN2 = 0`
- 右轮：`IN1 = 0`，`IN2 = 0`

驱动芯片处于滑行状态，电机不会被主动短接刹车。

典型用途：

- 上电初始化后保持电机不动。
- 开环测试结束后停止电机。
- 异常状态下先关闭动力输出。

示例：

```c
Motor_Stop(); //by codex
```

## Motor_Brake

功能：主动刹车。

它会把四个 PWM 通道都设为最大占空比，相当于：

- 左轮：`IN1 = 1`，`IN2 = 1`
- 右轮：`IN1 = 1`，`IN2 = 1`

根据 AT8236 真值表，这属于刹车状态。和 `Motor_Stop()` 不同，`Motor_Brake()` 会让电机两端被驱动芯片主动短接，减速更快。

典型用途：

- 需要快速停车时使用。
- 调试时短时间制动。

注意：不要长时间高频切换刹车和大占空比驱动，避免电机和驱动芯片发热。

示例：

```c
Motor_Brake(); //by codex
```

## Motor_SetDuty

功能：设置左右电机占空比。

函数原型：

```c
void Motor_SetDuty(int16_t left_percent, int16_t right_percent); //by codex
```

参数：

- `left_percent`：左电机占空比，范围 `-100` 到 `100`。
- `right_percent`：右电机占空比，范围 `-100` 到 `100`。

方向含义：

- `Motor_SetDuty(60, 60)`：左右轮向前。
- `Motor_SetDuty(-60, -60)`：左右轮向后。
- `Motor_SetDuty(60, -60)`：原地向右或向左转，具体方向取决于车体坐标定义。
- `Motor_SetDuty(0, 0)`：等价于 `Motor_Stop()`。

如果传入值超过范围，驱动会自动限幅：

- 大于 `100` 会按 `100` 处理。
- 小于 `-100` 会按 `-100` 处理。

典型用途：开环控制、转向测试、后续 PID 输出直接驱动电机。

示例：

```c
Motor_SetDuty(60, 60); //by codex
HAL_Delay(3000); //by codex
Motor_Stop(); //by codex
```

## Motor_SetDutyWithDeadZone

功能：带死区补偿的占空比设置。

函数原型：

```c
void Motor_SetDutyWithDeadZone(int16_t left_percent, int16_t right_percent, uint8_t dead_zone_percent); //by codex
```

参数：

- `left_percent`：左电机目标占空比，范围 `-100` 到 `100`。
- `right_percent`：右电机目标占空比，范围 `-100` 到 `100`。
- `dead_zone_percent`：死区补偿值，比如你当前测到约 `56`。

补偿逻辑：

- 如果目标为 `0`，输出仍然是 `0`。
- 如果目标为正，但小于死区值，则输出死区值。
- 如果目标为负，但绝对值小于死区值，则输出负死区值。
- 如果目标绝对值已经大于死区值，则保持原值。

例子：

- `Motor_SetDutyWithDeadZone(20, 20, 56)` 实际输出约等于 `56, 56`。
- `Motor_SetDutyWithDeadZone(70, 70, 56)` 实际输出仍是 `70, 70`。
- `Motor_SetDutyWithDeadZone(-20, -20, 56)` 实际输出约等于 `-56, -56`。
- `Motor_SetDutyWithDeadZone(0, 0, 56)` 实际输出仍是 `0, 0`。

典型用途：

- 电机刚启动阶段。
- 开环实验中避免低占空比完全不动。
- 后续 PID 输出较小时，避免控制量落在电机死区内。

示例：

```c
Motor_SetDutyWithDeadZone(60, 60, MOTOR_DEFAULT_DEAD_ZONE_PERCENT); //by codex
```

## 推荐初始化顺序

在 `main.c` 中，推荐顺序如下：

```c
MX_GPIO_Init(); //by codex
MX_TIM3_Init(); //by codex
if (Motor_Init() != HAL_OK) //by codex
{ //by codex
  Error_Handler(); //by codex
} //by codex
Motor_SetDutyWithDeadZone(60, 60, MOTOR_DEFAULT_DEAD_ZONE_PERCENT); //by codex
```

## 开环实验建议

你当前测得死区大约在 `56%` 附近，所以开环测试建议从 `56%` 或 `60%` 开始，再逐步增加：

- `56%`
- `60%`
- `65%`
- `70%`
- `80%`

每组占空比保持固定时间，记录左右电机转速。后续接入编码器后，可以用编码器计数值替代人工观察。

## 注意事项

- 电机测试时先把小车架空，避免突然冲出。
- `Motor_Init()` 必须在 `MX_TIM3_Init()` 后调用。
- 如果重新用 CubeMX 生成代码，要确认 `TIM3` 仍是 `PWM Generation CH1~CH4`，`Period` 仍是 `3599`。
- `Motor_SetDuty()` 不做死区补偿，适合你想观察真实占空比响应时使用。
- `Motor_SetDutyWithDeadZone()` 会自动抬高小占空比，适合实际控制时使用。
