# simple_foc_sh

ESP32 + ESP-IDF 5.5 + FreeRTOS 的三相 FOC 电机控制实验工程。

## 当前硬件映射

- M1 PWM：U/V/W = GPIO26 / GPIO27 / GPIO14
- 电流采样：U/V = GPIO36 / GPIO39（ADC1_CH0 / ADC1_CH3）
- W 相电流：`iw = -(iu + iv)`
- AS5600：SDA=GPIO21，SCL=GPIO19，地址=0x36
- 直流母线：实验使用 12 V

## 当前控制框架

- PWM：20 kHz，MCPWM 上下计数模式。
- 电流环：由 MCPWM 空事件通知 FreeRTOS 任务，每 12 个 PWM 周期执行一次，控制周期固定为 600 us。
- ADC：ADC continuous DMA，采样频率 200 kHz，40 字节一帧。
- ADC 帧：经典 ESP32 的一个 DMA 结果是 4 字节，因此 40 字节是 10 个结果，约 50 us 的采样窗口。
- 电流值：DMA 任务完成一帧平均后发布，电流环只读取已发布的最新结果，不在实时任务中做 ADC 阻塞读取。
- 角度：AS5600 独立任务每 1 ms 读取一次；电流环根据角度时间戳预测到电流采样帧的中心时刻。
- 速度环：当前关闭，正在单独验证电流环。

## 最近一次实测结论（2026-09-03）

最新日志中：

- `current_age_us` 约 73~198 us，说明 ADC DMA 数据新鲜度正常。
- `missed=0`、`overrun=0`，说明 PWM 通知、FreeRTOS 调度和电流环没有丢周期。
- `max_loop_us=501`，小于固定控制周期 600 us，当前任务仍有约 99 us 余量。
- 电机速度约 156~159 rad/s，说明固定 `iq_ref=100 mA` 时电机已经实际运行。
- `id` 仍有约 -65~113 mA 波动，`iq` 约 12~224 mA 波动。
- `vd` 长期约 -4.1 V，但没有达到 ±5 V 限幅；PI 没有继续失控累加。

因此，当前已经排除“ADC DMA 太慢、`vTaskDelay` 造成 PI 周期错误、FreeRTOS 丢调度”这几类问题。剩余重点是电角度零点/角度预测精度，以及 1.67 kHz 电流环更新频率带来的电流和转矩波动。固定 `iq` 是转矩测试，不是速度闭环；速度环关闭时电机自然会加速。

## 当前实验参数

- `M1_ENABLE_SPEED_LOOP = 0`
- `M1_CURRENT_LOOP_TEST_IQ_REF_A = 0.10 A`
- 电流 PI 输出限幅：-5 V ~ +5 V
- 电流环 PI：`Kp=1.5`，`Ki=8.0`
- 电角度预测额外延时：0 us

## 验证方式

```powershell
python -m unittest discover -s test -p 'test_*contract.py'
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run -t upload --upload-port COM7
```

最近一次提交前验证：契约测试 41/41 通过，固件编译成功，并已下载到 COM7。