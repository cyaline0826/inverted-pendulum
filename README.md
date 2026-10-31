# FPGA 旋转倒立摆控制系统

本工程面向旋转倒立摆，使用 FPGA 完成摆杆起摆、直立保持和旋转臂位置调节。工程器件配置为 Gowin `GW2A-LV18PG256C8/I7`，RTL 使用 Verilog 编写。

## 功能

- 通过 I2C 读取 ADS1115 的摆杆角度通道，并读取 A0 电位器作为位置目标输入。
- 起摆状态机根据有效角度样本、摆动方向和转折信息控制电机；进入直立区域后经过捕获与稳摆确认，再切换到平衡控制。
- 角度 PID 内环保持摆杆直立，位置 PD 外环根据编码器位置误差调整角度目标。
- 位置目标可由 A0 电位器、KEY2 触发的半圈位置轨迹或 UART 输入的有符号整数设置。
- OLED 显示运行状态和位置；UART 输出位置目标与实际相对位置。
- 使用正交编码器反馈旋转臂位置，通过 PWM 和方向信号驱动电机。

## 控制节拍

顶层默认时钟为 50 MHz。角度控制周期为 2 ms（500 Hz），位置控制周期为 10 ms（100 Hz），编码器速度采样周期为 1 ms（1 kHz）。具体参数可在 `src/daolibai.v` 顶层参数中查看。

## 工程结构

```text
daolibai.gprj          Gowin FPGA 工程文件
src/
  daolibai.v           顶层控制与模块连接
  swing_up.v           起摆、捕获和稳摆状态管理
  pid_angle.v          摆杆角度内环
  pid_position.v       旋转臂位置外环
  angle_velocity_timed.v  角度变化率估算
  adc.v, i2c.v         ADS1115 采样与 I2C 通信
  encoder.v            AB 相正交编码器解码
  pwm.v                电机 PWM 输出
  knob_position.v      电位器位置目标处理
  position_half_turn.v 半圈位置轨迹规划
  uart.v, uart_position.v  UART 收发与目标解析
  oled.v, oled_number.v OLED 状态显示
  key.v                按键处理
  daolibai.cst         引脚约束
  daolibai.sdc         时序约束
```

## 打开与运行

1. 安装 Gowin EDA，并在工程管理器中打开 `daolibai.gprj`。
2. 使用与当前开发板硬件连接相符的 `src/daolibai.cst`，确认 ADC、编码器、电机、按键和 OLED 引脚后再综合。
3. 综合、布局布线并生成配置文件后，将 FPGA 配置到开发板。首次运行前应确认电机驱动和摆杆机械结构处于安全状态。

工程参数包含传感器量程、角度目标、编码器计数和 PID 增益，换用硬件或机械结构时需要重新核对和调试。该仓库描述 RTL 与工程配置，不代表已在所有板卡上完成验证。
