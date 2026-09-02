# FOC Real-Time Current Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 在 ESP32 + ESP-IDF/FreeRTOS 上建立可验证的 10 kHz FOC 电流环：PWM 事件提供节拍，ADC DMA 提供同步采样，PI 使用固定 Ts，I2C 和日志不阻塞控制路径。电流环稳定后再恢复速度环。

**Architecture:** 保留 M1 的三相 PWM GPIO26/27/14 和 U/V 电流 GPIO36/39。MCPWM 20 kHz 中心对齐运行，每两个半周期产生一次 10 kHz 控制通知。ADC continuous DMA 持续采样，由 DMA frame 完成回调唤醒/标记最新采样帧。当前控制任务只等待 PWM 通知，读取最近的完整采样帧，使用固定 100 us 运行 PI，并在 PWM 安全更新点更新占空比。AS5600 移到独立低速任务，当前环只读取角度快照或插值结果。

**Tech Stack:** PlatformIO, ESP-IDF 5.5, FreeRTOS task notifications, MCPWM, ADC continuous DMA, AS5600 I2C, C unit-contract tests.

**Spec:** 具体硬件、时序、日志字段和阶段验收标准同步记录在 `readme.txt`；本文件是实施顺序和变更边界。

## Global Constraints

- M1 PWM 映射固定为 U/V/W = GPIO26/27/14；任何阶段不得改动。
- 电流采样固定从 GPIO36/39 读取，W 相继续由 `iw = -(iu + iv)` 重建，除非实测证明硬件定义不同。
- 电流环控制路径禁止 I2C、printf/ESP_LOG、Wi-Fi、动态内存、长时间 busy-wait 和 `vTaskDelay`。
- PI 使用固定 `FOC_CURRENT_TS_S = 0.0001f`，不使用日志周期或任务唤醒间隔作为控制 Ts。
- 速度环保持关闭，直到电流给定阶跃可重复、无 ADC DMA 溢出、无控制超时、无 WDT、三相电流闭合误差合格。
- 每个阶段都要运行测试或构建并记录结果；不把“串口看起来有输出”当作时序证明。
- 不 force-push，不覆盖用户未提交的改动；每个可独立验证的阶段单独提交。

---

## Task 1: 建立当前基线和时序契约

**Files:**
- Create: `test/test_foc_timing_contract.py`
- Update: `readme.txt`

- [ ] 记录当前工程、分支、M1 引脚、PWM 频率、ADC DMA 配置和当前控制周期。
- [ ] 添加静态契约测试，检查源码中仍存在 `FOC_CURRENT_LOOP_PERIOD_MS 1U`、`vTaskDelayUntil`、可变 `control_dt_s` 和 current task 内角度 I2C 调用，作为当前问题的可重复基线。
- [ ] 运行 Python 测试，确认基线测试能明确指出这些待修复项，而不是依赖人工猜测。
- [ ] 运行 PlatformIO clean build，确认改动前工程可编译。
- [ ] Commit: `test: establish realtime current-loop baseline`

## Task 2: 用 MCPWM 事件建立 10 kHz 控制节拍

**Files:**
- Update: `src/motor/motor_pwm.c`
- Update: `src/motor/motor_pwm.h`
- Update: `src/main.c`
- Update: `test/test_foc_timing_contract.py`

- [ ] 注册 MCPWM timer 的 ISR 回调，使用 TEZ/TEP 或 full/empty 事件计数，每两个 20 kHz 半周期通知一次控制任务。
- [ ] 暴露 `motor_pwm_register_control_task(TaskHandle_t)` 和节拍计数/丢节拍诊断接口。
- [ ] ISR 只做计数和 `vTaskNotifyGiveFromISR`，不读 I2C、不做 PI、不打印。
- [ ] 保持 GPIO26/27/14、20 kHz 中心对齐、现有互补输出和死区配置不变。
- [ ] 测试通知接口和配置约束，编译确认 ESP-IDF 5.5 API 类型正确。
- [ ] Commit: `feat: derive current-loop tick from MCPWM`

## Task 3: 把 ADC continuous DMA 变成完整采样帧

**Files:**
- Update: `src/sensor/current_sense.c`
- Update: `src/sensor/current_sense.h`
- Update: `test/test_foc_timing_contract.py`

- [ ] 注册 `on_conv_done` 和 `on_pool_ovf` 回调，记录 frame sequence、完成计数和 DMA overflow。
- [ ] 明确 ADC pattern 的 U/V 通道顺序、结果字节宽度和每个控制周期所需的样本数；不再通过控制任务临时读一两个结果推断电流。
- [ ] 提供无阻塞的 `current_sense_read_latest_frame()`，只返回最近一个完整帧；没有新帧时返回明确状态。
- [ ] 保留 U/V 原始值、零点校准、比例换算和 W 相闭合误差诊断。
- [ ] 删除或隔离 current task 内按 PWM 计数器 busy-wait 的采样窗口逻辑；采样同步由 DMA/PWM 设计负责。
- [ ] Commit: `feat: publish adc dma current frames`

## Task 4: 把 AS5600 从电流环移到角度缓存

**Files:**
- Update: `src/sensor/as5600.c`
- Update: `src/sensor/as5600.h`
- Update: `src/main.c`
- Update: `test/test_foc_timing_contract.py`

- [ ] 创建 1--2 kHz 的角度采样任务，I2C 读取只发生在该任务中。
- [ ] 提供带 sequence/timestamp 的角度快照，当前环不直接调用 AS5600 I2C。
- [ ] 按需要使用最近角度和速度进行短时插值，避免 10 kHz 当前环看到阶梯角度。
- [ ] 保留机械角、电角零点偏移和方向定义，先验证符号，不盲目改 GPIO 或相序。
- [ ] Commit: `refactor: cache angle outside current loop`

## Task 5: 切换为固定 Ts 的实时电流 PI

**Files:**
- Update: `src/main.c`
- Update: `src/motor/pi_controller.c` or current PI implementation
- Update: `test/test_foc_timing_contract.py`

- [ ] 将 current task 改为 `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` 等待 MCPWM 控制节拍，不再使用 `vTaskDelayUntil`。
- [ ] 每次控制严格使用 `FOC_CURRENT_TS_S = 0.0001f`；实测执行时间和节拍计数仅用于诊断。
- [ ] 增加 overrun、missed_tick、ADC_frame_age、ADC_overflow、三相闭合误差和 PI 饱和计数。
- [ ] 把遥测降频、批量化并放到非实时任务；current task 不直接打印。
- [ ] 确认 PI 积分限幅、输出限幅、饱和时 anti-windup 以及 `vd/vq` 符号和电角度方向。
- [ ] Commit: `feat: run fixed-step realtime current pi`

## Task 6: 分级上电验证和 PI 调参

**Files:**
- Update: `readme.txt`
- Optional: `src/main.c` debug telemetry fields

- [ ] 无功率或低风险条件下验证：10 kHz tick、执行时间、无 missed tick、无 DMA overflow、无 WDT。
- [ ] 上电但 `iq_ref=0`：确认三相电流接近零，`iu+iv+iw` 接近零，占空比不会异常饱和。
- [ ] 小电流阶跃：从 0.05 A 开始，先只调电流环 Kp，再逐步增加 Ki；观察 id/iq 跟踪、噪声和啸叫。
- [ ] 改变电角度方向或电流符号前，先保存日志并一次只改一个变量。
- [ ] 只有电流环稳定后才打开速度环；速度环使用更低频任务通知，并输出 `iq_ref` 限幅。
- [ ] Commit: `docs: record realtime current-loop validation`

## Verification Commands

```powershell
python -m unittest discover -s test -p "test_*.py"
& "C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe" run
git status --short
git log --oneline -5
```

## Final Review Checklist

- [ ] M1 GPIO26/27/14 未改变。
- [ ] current task 没有 I2C、日志、`vTaskDelay` 或 PWM-counter busy-wait。
- [ ] PI Ts 是固定 100 us，不来自 `dt_us` 或 1 Hz 遥测。
- [ ] ADC 采样以完整 DMA frame 为单位，能报告 frame age 和 overflow。
- [ ] MCPWM 事件、控制通知和 PWM 更新点有明确关系。
- [ ] 先有测试/build/日志证据，再说“电流环调好了”。
