# Current Loop Tuner Design

**Date:** 2026-09-09

## Goal

Provide a Windows Python desktop tool for the `D:\platformio_workspace\simple_foc_sh` firmware that can connect to the existing 115200-baud console, send a bounded Iq target, display current-loop telemetry in real time, and save a CSV trace.

## Current constraints

The firmware currently emits human-readable `FOC_TUNE` lines from `src/main.c`, but it has no command receiver. The existing FOC current loop runs at 5 kHz, the motor is currently in the fixed-Iq/free-rotation bench mode, and the safety limit in `src/main.c` is 0.50 A per phase. The PC tool must not remove or bypass those protections.

## Scope

The first version includes:

- A Python GUI using Tkinter and Matplotlib.
- COM-port and baud-rate selection, connect/disconnect, and connection status.
- A signed Iq target entry in mA, with a visible firmware limit of ±500 mA.
- Apply, stop/zero, and telemetry-rate controls.
- Live plots for `iq_ref`, `iq`, `id`, `vq`, and mechanical speed.
- A bounded in-memory history and CSV recording.
- Parsing of fragmented serial lines and the current `FOC_TUNE` log format.
- A small ASCII command protocol on the existing console UART.

It does not include a speed loop, automatic PI tuning, motor-parameter identification, automatic flashing, or a claim that mA is an exact N·m torque value. The GUI labels the command as Iq until the motor torque constant is measured.

## Serial protocol

Commands are newline terminated:

```text
FOC IQ <signed_mA>
FOC STOP
FOC RATE <hz>
FOC STATUS
```

Responses are short ASCII lines:

```text
FOC OK IQ=100
FOC OK STOP
FOC OK RATE=50
FOC STATUS IQ=100 RATE=50 FAULT=0
FOC ERR RANGE
FOC ERR FORMAT
```

The firmware accepts `-500` through `500` mA, stores the target behind a critical-section protected state object, and starts with a target of zero. The current-loop task continues to apply its configured reference ramp, so a newly requested value does not step the PI reference abruptly. `FOC STOP` requests zero and clears the command-side enable state. The firmware keeps angle-stale, current-stale, invalid-current, and latched-fault handling unchanged.

Telemetry keeps the existing human-readable `FOC_TUNE` output at its current one-second diagnostic interval. A second compact line is emitted at a configurable rate, default 20 Hz and limited to 1..100 Hz:

```text
FOC_DATA,t_us,iq_target_mA,iq_ref_mA,iq_mA,id_mA,vq_mV,speed_mrad_s,current_age_us,angle_age_us,fault
```

This rate is sufficient for a GUI waveform while avoiding 5 kHz log flooding over 115200 baud. The GUI requests 50 Hz after connecting.

## Architecture

The firmware protocol state is isolated in `src/control/foc_tune_protocol.c/.h`. It owns command parsing, input task lifetime, the bounded Iq command, and telemetry-rate state. `src/main.c` reads the command state from the current-loop task and publishes the compact telemetry line from the existing snapshot. The protocol module uses the already configured console UART through standard input/output; it does not install a second UART driver over UART0.

The Python application is split into pure protocol/telemetry parsing, a serial worker, a bounded trace model, and the Tkinter view. The parser never touches Tkinter and the serial worker never updates widgets directly; the GUI polls a thread-safe queue on its normal event loop.

## Safety behavior

- GUI startup target is zero and no command is sent until the user presses Apply.
- Apply validates finite integer mA and displays the ±500 mA limit.
- Stop sends `FOC STOP` and then waits for an acknowledgement; disconnect also sends a best-effort stop before closing.
- A stale-data indicator turns red after 500 ms without `FOC_DATA`.
- GUI communication errors do not cause a new target to be sent.
- Firmware rejects malformed and out-of-range commands and clamps no accepted value silently.
- Existing firmware protection and PWM shutdown paths remain authoritative.

## Runtime alignment behavior

The GUI exposes an explicit alignment button. It sends `FOC ALIGN`; the protocol task only records a request and zeros both runtime targets. The current-loop task consumes that request at a PWM-loop boundary, resets both PI controllers, stops PWM, runs the existing low-voltage alignment routine, applies the measured electrical-zero offset plus the configured trim, restarts PWM, and leaves both targets at zero. It then prints either `FOC ALIGN DONE offset=<rad>` or `FOC ALIGN ERR CALIBRATION`.

The GUI also exposes Id and Iq target fields because Id is useful for the current-loop bench test while Iq is the torque-producing command. Both are bounded to ±500 mA and ramped by firmware. A normal fixed-torque test uses Id=0 and a nonzero Iq.
## Acceptance checks

A headless Python test run must pass parser, command-format, fragmented-line, bounded-history, and CSV tests. A local GUI launch must succeed when its Python dependencies are installed. `platformio run` must pass for the firmware. Bench validation is manual: with the motor mechanically safe, connect, observe zero target, apply 50/100 mA, confirm `iq_ref` ramps toward the target, press Stop, and confirm target and reference return toward zero.