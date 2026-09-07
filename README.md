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
- 电流环：由 MCPWM 空事件通知 FreeRTOS 任务，每 4 个 PWM 周期执行一次，控制周期固定为 200 us（5 kHz）。
- ADC：ADC continuous DMA，配置采样频率 200 kHz，32 字节一帧；运行时按DMA事件时间戳对齐。
- ADC 帧：经典 ESP32 的一个 DMA 结果是 4 字节，因此32字节是8个结果，配置窗口约40 us；实际窗口以相邻DMA完成时间为准。
- 电流值：DMA 任务完成一帧平均后发布，电流环只读取已发布的最新结果，不在实时任务中做 ADC 阻塞读取。
- 角度：AS5600 独立任务由500 us esp_timer通知；电流环根据角度时间戳外推到电流采样帧中心。
- 速度环：当前关闭，正在单独验证电流环。

## 最近一次实测结论（2026-09-03）

最新日志中：

- `current_age_us` 约 73~198 us，说明 ADC DMA 数据新鲜度正常。
- `missed=0`、`overrun=0`，说明 PWM 通知、FreeRTOS 调度和电流环没有丢周期。
- 历史 600 us 版本曾记录 max_loop_us=501；当前已切换为 5 kHz/200 us，实机以新的 COM7 遥测为准。
- 电机速度约 156~159 rad/s，说明固定 `iq_ref=100 mA` 时电机已经实际运行。
- `id` 仍有约 -65~113 mA 波动，`iq` 约 12~224 mA 波动。
- `vd` 长期约 -4.1 V，但没有达到 ±5 V 限幅；PI 没有继续失控累加。

因此，当前已经排除“ADC DMA 太慢、`vTaskDelay` 造成 PI 周期错误、FreeRTOS 丢调度”这几类问题。剩余重点是电角度零点/角度预测精度，以及 5 kHz 电流环更新频率带来的电流和转矩波动。固定 `iq` 是转矩测试，不是速度闭环；速度环关闭时电机自然会加速。

## 当前实验参数

- `M1_ENABLE_SPEED_LOOP = 0`
- `M1_CURRENT_LOOP_TEST_IQ_REF_A = 0.20 A`
- 电流 PI 输出限幅：-5 V ~ +5 V
- 电流环 PI：`Kp=1.5`，`Ki=8.0`
- 电角度预测额外延时：0 us

## 验证方式

```powershell
python -m unittest discover -s test -p 'test_*contract.py'
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run -t upload --upload-port COM7
```

最近一次提交前验证：契约测试 39/39 通过，固件编译成功，并已下载到 COM7。
## 本次清理记录（2026-09-04）

当前工程的电机运行入口保留闭环电流控制链路，LED/WS2812 功能也保留供后续使用。已删除：旧开环任务及其文件、固定 PWM 测试分支、M2 引脚配置、重复的电流 GPIO 宏、PWM tick 查询接口、旧的阻塞式电流读取接口，以及对应的无效测试脚本和断言。

保留的宏开关都有实际用途：`M1_ENABLE_SPEED_LOOP` 用于后续接入速度环，`M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION` 控制上电校准，`M1_ENABLE_TIMING_LOG` 和 `M1_ENABLE_CURRENT_TRACE` 用于现场时序/电流调试。它们不是未使用配置。

本次清理后契约测试为 39/39，通过后再进行完整固件编译。
## 2026-09-04：高速电流环 dq 解耦

- 根据 COM7 抓包中约 12 V、iq_ref=200 mA、约 200 rad/s 的稳态工作点，加入初始电机模型参数：Ld=Lq=19 mH、磁链 0.53 mWb；参数集中在 src/motor/motor_config.h，后续可用堵转/阶跃测试修正。
- FOC 控制器加入标准 PMSM dq 解耦前馈：vd_ff=Rs*Id-ωe*Lq*Iq、vq_ff=Rs*Iq+ωe*(Ld*Id+磁链)。
- 解耦后的 vd/vq 总矢量在逆 Park 前限幅到 12 V 母线、5%~95% 占空比对应的线性 SVPWM 范围。
- 遥测新增 vd_dec_mV、vq_dec_mV、v_limit_mV，用于确认高速时主要电压是否由模型前馈承担。
- 本次保留：电流环 5 kHz、ADC DMA、PWM GPIO26/27/14、ADC GPIO36/39、速度环关闭、LED/WS2812。
- 固件已编译并下载到 COM7；下载后的串口复测因 COM7 被已有监视器占用，待监视器释放后继续采集。
## 2026-09-04 实时修正

- PI 积分使用固定 `FOC_CURRENT_TS_S=200 us`（由 5 kHz PWM 事件节拍计算）；任务唤醒间隔只用于统计调度抖动，不再进入 PI。
- FOC 电流任务固定在 CPU1，ADC DMA 任务固定在 CPU1；电流环仍通过 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 阻塞等待 PWM 通知，不使用 `vTaskDelay`。
- ADC DMA 任务完成一次校准换算后，把安培值写入带时间戳的历史帧；FOC 实时路径只复制历史帧，避免在控制周期内调用 ADC 校准函数。
- Park 使用电流采样时刻的电角度；逆 Park 使用“电流帧年龄 + 约50 us PWM更新窗口”的预测角度，补偿实际执行延迟。
- 当前 ESP32 ADC continuous 公共接口仍是自由运行 DMA，代码实现的是带时间戳的软件对齐，不是 MCPWM 直接触发 ADC 的硬件同步。若后续必须做到硬件触发采样，需要改用支持触发链路的 ADC/MCU 或底层外设方案。
- 最新一次旧版本日志中 `speed_mrad_s≈166000`、`id≈-150~255 mA`、`vd≈-4.5~-5.0 V`，说明高速时角度/执行延迟比 PI 参数更先需要处理。

## 2026-09-04：高速电流环 dq 解耦

- 根据 COM7 抓包中约 12 V、iq_ref=200 mA、约 200 rad/s 的稳态工作点，加入初始电机模型参数：Ld=Lq=19 mH、磁链 0.53 mWb；参数集中在 src/motor/motor_config.h，后续可用堵转/阶跃测试修正。
- FOC 控制器加入标准 PMSM dq 解耦前馈：vd_ff=Rs*Id-ωe*Lq*Iq、vq_ff=Rs*Iq+ωe*(Ld*Id+磁链)。
- 解耦后的 vd/vq 总矢量在逆 Park 前限幅到 12 V 母线、5%~95% 占空比对应的线性 SVPWM 范围。
- 遥测新增 vd_dec_mV、vq_dec_mV、v_limit_mV，用于确认高速时主要电压是否由模型前馈承担。
- 本次保留：电流环 5 kHz、ADC DMA、PWM GPIO26/27/14、ADC GPIO36/39、速度环关闭、LED/WS2812。
- 固件已编译并下载到 COM7；下载后的串口复测因 COM7 被已有监视器占用，待监视器释放后继续采集。
## 2026-09-04 低延迟实测

- 将DMA帧从40字节改为32字节后，current_age_us实测约100~200 us，且5 kHz电流环无新增丢周期。iq_ref=200 mA时平均iq接近目标，但高速瞬时纹波仍受AS5600异步采样和电机反电势影响。
- 尝试10 kHz（每2个PWM中心事件、100 us）时，实测current_age_us约3.2 ms、missed持续增加并触发CPU1看门狗，已回退到稳定的5 kHz。
- 当前下载到COM7的版本：速度环关闭、PWM引脚仍为26/27/14、ADC仍为36/39、32字节DMA帧、5 kHz固定PI周期。
