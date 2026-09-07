# Mature FOC Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a predictable AS5600 angle estimator and robust current-loop protection to the ESP32 FOC firmware.

**Architecture:** Keep AS5600 I2C in a slow task, publish a validated angle/velocity estimator state, and let the current loop predict electrical angle at the ADC sample and PWM output timestamps. Keep the current PI time base tied to PWM notifications and add vector anti-windup and stale-angle shutdown.

**Tech Stack:** ESP-IDF 5.5, FreeRTOS, MCPWM prelude driver, ADC continuous DMA, C, Python `unittest`, PlatformIO.

**Spec:** `docs/superpowers/specs/2026-09-07-mature-foc-control-design.md`

## Global Constraints

- Preserve PWM U/V/W GPIO26/GPIO27/GPIO14.
- Preserve current U/V GPIO36/GPIO39 and W reconstruction.
- Preserve AS5600 SDA21/SCL19/address 0x36.
- Preserve LED/WS2812 tasks and initialization.
- Keep `M1_ENABLE_SPEED_LOOP=0` during current-loop work.
- Do not call blocking I2C, logging, or `vTaskDelay` in the fast current-control path.

### Task 1: Add estimator and protection contract tests

**Files:**
- Create: `src/control/foc_angle_estimator.h`
- Create: `src/control/foc_angle_estimator.c`
- Test: `test/test_foc_angle_estimator_contract.py`
- Test: `test/test_foc_timing_contract.py`
- Test: `test/test_foc_pi_integration_contract.py`

- [ ] Write tests that require a validated estimator state, wrap-safe angle prediction, stale/error invalidation, and controller vector anti-windup.
- [ ] Run the focused tests and verify they fail because the estimator API and protection behavior do not exist yet.

### Task 2: Implement the AS5600 estimator

**Files:**
- Modify: `src/control/foc_angle_estimator.h`
- Modify: `src/control/foc_angle_estimator.c`
- Modify: `src/main.c`
- Modify: `src/CMakeLists.txt`

- [ ] Implement wrap-safe sample updates, filtered velocity, timestamped validity, and prediction at an arbitrary timestamp.
- [ ] Replace direct angle snapshot extrapolation in `foc_current_task` with estimator prediction for the current-sample and PWM-output timestamps.
- [ ] Mark the estimator invalid after an AS5600 read failure and recover only after a new valid sample.
- [ ] Run focused estimator/timing tests.

### Task 3: Implement voltage-vector anti-windup and safe stale-angle handling

**Files:**
- Modify: `src/control/foc_controller.c`
- Modify: `src/main.c`
- Modify: `src/motor/motor_config.h`

- [ ] Add vector saturation back-calculation after feed-forward terms.
- [ ] Add explicit maximum angle age and safe center-PWM/PI-reset behavior.
- [ ] Disable dq decoupling by default until measured motor parameters are available, while retaining the code path for later validation.
- [ ] Run the full Python contract suite and native firmware build.

### Task 4: Verify and document

**Files:**
- Modify: `README.md`

- [ ] Document the actual fast-path timing and the AS5600 estimator boundary.
- [ ] Run `python -m unittest discover -s test -p 'test_*contract.py'`.
- [ ] Run PlatformIO build and inspect Git diff/check output.
- [ ] Download to COM7 only after build verification and record the resulting telemetry.
