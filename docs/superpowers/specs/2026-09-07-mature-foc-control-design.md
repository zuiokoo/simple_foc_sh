# Mature FOC Control Architecture Design

**Date:** 2026-09-07

## Goal

Make the existing ESP32 native ESP-IDF/FreeRTOS FOC control path behave like a mature sensored FOC implementation while preserving the board pin contract and LED/WS2812 functions.

## Current constraints

- PWM U/V/W remain GPIO26/GPIO27/GPIO14.
- Current U/V remain GPIO36/GPIO39; W remains reconstructed as `-(U+V)`.
- AS5600 remains on SDA21/SCL19 at address `0x36`.
- Speed loop remains disabled during current-loop bring-up.
- Current-loop PI time base remains the PWM-derived fixed period.
- No blocking I2C or logging is allowed in the fast current-control path.

## Target data flow

```text
AS5600 I2C task (slow absolute samples)
    -> angle unwrap + velocity estimate + phase prediction
PWM event
    -> wake the dedicated current-control path
ADC DMA completed frames
    -> timestamped current sample
angle predictor at current-sample and PWM-output timestamps
    -> Clarke/Park -> current PI -> voltage limit/anti-windup -> inverse Park/SVPWM
```

## Design decisions

1. AS5600 is an absolute-angle source, not a blocking dependency of the current PI. The fast path consumes a compact estimator state and predicts angle from the most recent valid sample.
2. A failed or stale AS5600 sample invalidates the fast path. The controller must center PWM and reset PI state instead of continuing indefinitely with stale angle.
3. The current PI uses a fixed PWM-derived `dt`; scheduler latency is telemetry only.
4. The controller must apply vector voltage limiting with anti-windup after adding feed-forward terms.
5. dq decoupling is disabled by default until motor resistance, inductance, flux linkage, and electrical-angle sign are measured. It can be enabled later without changing the real-time architecture.
6. This first implementation keeps the ESP32/IDF-supported task boundary and makes its timing explicit. A later hardware-triggered ADC variant is a separate change and must be verified against the exact ESP32 peripheral capability before being claimed.

## Verification

- Contract tests cover estimator prediction, wrap-around, stale/error invalidation, fixed PI period, and preservation of pin/LED contracts.
- Native test suite must pass.
- PlatformIO firmware build must pass.
- COM7 download and bench verification are separate after code verification; logs must show bounded angle age, no stale-angle operation, no repeated voltage saturation at low current reference, and stable current tracking before increasing current.

## Reference implementations consulted

- ESP-IDF ADC continuous mode and MCPWM documentation.
- VESC `bldc` ADC/PWM-triggered FOC implementation.
- ESP32 `espFoC` architecture using a slower AS5600 task and a PWM-rate encoder estimator.
- SimpleFOC AS5600 fast-mode driver.
