# Synchronized FOC Current Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 M1 电流环改成由 PWM 硬件事件驱动、ADC 数据带有确定采样时序、FOC 计算不使用 `vTaskDelay` 的标准实时结构。

**Architecture:** 保留 GPIO26/27/14、GPIO36/39 和 AS5600 接口。当前 ESP32 ADC continuous DMA 不提供由 MCPWM 直接触发 ADC 的现成同步接口，因此第一阶段采用 MCPWM TEZ 作为唯一控制节拍、ADC DMA 连续采样并记录转换帧事件；控制任务只消费最新完整帧。若实测仍受 PWM 开关噪声影响，再将采样硬件/ADC 触发方案单独升级，不在本阶段混入 PI 调参。

**Tech Stack:** PlatformIO, ESP-IDF 5.5, FreeRTOS task notifications, MCPWM, ADC continuous DMA, native C, unittest contract tests.

**Spec:** 本计划实现 `docs/superpowers/plans/2026-09-04-synchronized-foc-current-loop.md` 与当前 simple_foc 硬件约束。

## Global Constraints

- M1 PWM 必须保持 U/V/W = GPIO26/GPIO27/GPIO14。
- 电流输入必须保持 U/V = GPIO36/GPIO39，W 由 `-(iu+iv)` 重构。
- `M1_ENABLE_SPEED_LOOP` 保持 0，先只验证电流环。
- 电流环禁止使用 `vTaskDelay` 作为控制周期。
- LED/WS2812 初始化和任务必须保留。
- 每个实现步骤必须先有失败测试，再实现，再运行契约测试和 PlatformIO 编译。

### Task 1: PWM event contract

**Files:**
- Modify: `test/test_foc_timing_contract.py`
- Inspect: `src/motor/motor_pwm.c`, `src/motor/motor_pwm.h`, `src/main.c`

- [ ] **Step 1: Write failing tests**

  Assert that the control notification is generated from a named PWM timer event, the current loop consumes a task notification, and no `vTaskDelay` appears in the current-loop function.

- [ ] **Step 2: Run the focused test and verify RED**

  Run `python -m unittest test.test_foc_timing_contract -v`.
  Expected: the new event/ownership assertion fails against the current implementation.

- [ ] **Step 3: Implement the smallest PWM event interface change**

  Keep MCPWM TEZ comparator buffering and GPIO mapping unchanged. Expose only the control-task registration/notification boundary required by the current loop; do not change frequency or PI gains.

- [ ] **Step 4: Run the focused test and verify GREEN**

  Run the same unittest command and expect all timing contract tests to pass.

### Task 2: ADC frame timing contract

**Files:**
- Modify: `test/test_current_sense_contract.py`
- Modify: `src/sensor/current_sense.c`
- Modify: `src/sensor/current_sense.h` only if the frame metadata interface needs a new field

- [ ] **Step 1: Write failing tests**

  Assert that each published frame retains its sequence and conversion-event timestamp and that the consumer can reject stale or repeated frames.

- [ ] **Step 2: Run the focused test and verify RED**

  Run `python -m unittest test.test_current_sense_contract -v`.
  Expected: the new per-frame timing assertion fails before implementation.

- [ ] **Step 3: Implement frame metadata without logging in the DMA task**

  Keep ADC1_CH0/CH3 and 200 kHz configuration. Publish a coherent latest frame with sequence and timestamp under the existing critical section. Keep the ISR short and IRAM-safe.

- [ ] **Step 4: Run tests and build**

  Run all contract tests, then `pio run`.
  Expected: tests and firmware build pass.

### Task 3: Fixed-latency FOC consumption

**Files:**
- Modify: `src/main.c`
- Modify: `src/control/foc_controller.h`
- Modify: `src/control/foc_controller.c`
- Modify: corresponding contract tests

- [ ] **Step 1: Write failing test for separate measurement/output angle**

  Assert that the controller input can carry the angle used for Park current measurement and the angle used for inverse-Park voltage output, while preserving the existing single-angle behavior through an explicit assignment.

- [ ] **Step 2: Run test and verify RED**

  Run the focused unittest and expect the new field/usage assertion to fail.

- [ ] **Step 3: Implement separate angles**

  Use the sampled current timestamp for Park. Predict the electrical angle to the next buffered PWM update for inverse Park. Keep PI `dt_s` based on measured notification-to-notification period.

- [ ] **Step 4: Run all tests and build**

  Expected: all contract tests pass and PlatformIO build succeeds.

### Task 4: Hardware verification

**Files:** no source changes unless telemetry proves a defect.

- [ ] **Step 1: Download to COM7**

  Run `pio run --target upload --upload-port COM7`.

- [ ] **Step 2: Capture low-current telemetry**

  Verify `dt_us` near the target, `missed=0`, current frame sequence increasing, `current_age_us` bounded, and no watchdog reset.

- [ ] **Step 3: Capture 100/200/500 mA steps**

  Keep speed loop disabled. Compare mean and peak-to-peak `id/iq`, voltage saturation, and electrical-angle timing.

- [ ] **Step 4: Decide next change from evidence**

  If timing is stable but current ripple remains, isolate PWM-switching noise and ADC sampling phase; do not blindly increase PI gains.

## Self-review

- GPIO, ADC channels, speed-loop state, and LED behavior are explicitly preserved.
- Every code task has a failing test, a focused implementation, and a build/test gate.
- No task assumes direct MCPWM-to-ADC triggering is available in ESP-IDF 5.5 continuous ADC; the plan treats that as a separate hardware/driver capability question.