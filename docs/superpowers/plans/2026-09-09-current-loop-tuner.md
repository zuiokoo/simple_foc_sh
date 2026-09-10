# Current Loop Tuner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a safe Python desktop tuner for the fixed-Iq FOC current loop and add the minimum firmware protocol needed to change Iq and stream waveform data.

**Architecture:** Add a small `foc_tune_protocol` firmware module that owns a critical-section protected runtime target and a console-UART command task. Extend the existing snapshot telemetry in `src/main.c` with a compact configurable-rate `FOC_DATA` line. Build a Python application from pure parsers, a serial worker, a bounded trace model, and a Tkinter/Matplotlib GUI.

**Tech Stack:** ESP-IDF/FreeRTOS C, existing PlatformIO environment, Python 3, Tkinter, pyserial, Matplotlib, unittest/pytest-compatible tests.

**Spec:** `docs/superpowers/specs/2026-09-09-current-loop-tuner-design.md`

## Global Constraints

- The firmware command target is signed Iq in mA and is accepted only in the inclusive range -500..500 mA.
- Firmware boots with Iq target 0 A and keeps the current reference ramp and all existing current/angle/fault protections.
- The GUI never sends a target on startup; Apply is required.
- The GUI treats Iq as a current command, not an exact N·m torque measurement.
- Compact telemetry is limited to 1..100 Hz; the GUI requests 50 Hz after connecting.
- Do not reset, checkout, or overwrite the existing dirty worktree. Do not upload or flash firmware.

---

### Task 1: Define the Python protocol and trace contracts

**Files:**
- Create: `tools/current_loop_tuner/current_loop_tuner/__init__.py`
- Create: `tools/current_loop_tuner/current_loop_tuner/protocol.py`
- Create: `tools/current_loop_tuner/current_loop_tuner/telemetry.py`
- Create: `tools/current_loop_tuner/current_loop_tuner/trace.py`
- Create: `tools/current_loop_tuner/tests/test_protocol.py`
- Create: `tools/current_loop_tuner/tests/test_telemetry.py`
- Create: `tools/current_loop_tuner/tests/test_trace.py`

**Interfaces:**
- `protocol.format_set_iq(mA: int) -> bytes` returns `b"FOC IQ <mA>\\n"` after validating -500..500.
- `protocol.format_stop() -> bytes` returns `b"FOC STOP\\n"`.
- `protocol.format_rate(hz: int) -> bytes` validates 1..100 and returns `b"FOC RATE <hz>\\n"`.
- `protocol.parse_response(line: str) -> Response` returns a dataclass with `kind`, `message`, and optional integer `value`.
- `telemetry.parse_line(line: str) -> TelemetrySample | None` parses `FOC_DATA` and current `FOC_TUNE` lines, normalizing values to SI units where the model needs them and preserving the raw line.
- `trace.TraceBuffer(maxlen: int = 5000)` exposes `append(sample)`, `clear()`, `snapshot() -> list[TelemetrySample]`, and `write_csv(path)`.

- [ ] **Step 1: Write the failing tests for command validation and formatting**

```python
from current_loop_tuner.protocol import format_set_iq, format_rate
import pytest

def test_set_iq_formats_signed_milliamps():
    assert format_set_iq(-100) == b"FOC IQ -100\\n"

def test_set_iq_rejects_values_above_firmware_limit():
    with pytest.raises(ValueError, match="-500..500"):
        format_set_iq(501)

def test_rate_formats_and_validates():
    assert format_rate(50) == b"FOC RATE 50\\n"
    with pytest.raises(ValueError, match="1..100"):
        format_rate(0)
```

- [ ] **Step 2: Run the tests and verify the RED state**

Run from `D:\platformio_workspace\simple_foc_sh\tools\current_loop_tuner`:

```powershell
python -m pytest tests/test_protocol.py -q
```

Expected: FAIL because the package and formatter functions do not exist yet.

- [ ] **Step 3: Implement the minimal protocol module**

Use integer parsing and explicit bounds checks. Keep the command formatters pure and return bytes so the serial worker can call `write()` directly. Define `Response` as a dataclass and parse `FOC OK`, `FOC STATUS`, and `FOC ERR` without throwing on unrelated boot/log lines.

- [ ] **Step 4: Add telemetry and trace RED/GREEN tests**

```python
def test_parse_compact_data_line():
    sample = parse_line("FOC_DATA,123456,100,96,94,3,812,4200,120,80,0")
    assert sample.timestamp_us == 123456
    assert sample.iq_target_a == pytest.approx(0.100)
    assert sample.iq_a == pytest.approx(0.094)
    assert sample.fault == 0

def test_parse_current_tune_line():
    sample = parse_line(
        "E (2810) FOC_TUNE: dt_us=200 speed_mrad_s=4200 "
        "angle_age_us=80 current_age_us=120 iu_mA=1 iv_mA=-2 iw_mA=1 "
        "iq_ref_mA=100 iq_target_mA=100 id_mA=3 iq_mA=94 "
        "id_err_mA=-3 iq_err_mA=6 vd_mV=0 vq_mV=812"
    )
    assert sample.iq_a == pytest.approx(0.094)
    assert sample.vq_v == pytest.approx(0.812)

def test_trace_buffer_is_bounded_and_writes_csv(tmp_path):
    trace = TraceBuffer(maxlen=2)
    trace.append(sample_at(1))
    trace.append(sample_at(2))
    trace.append(sample_at(3))
    assert [s.timestamp_us for s in trace.snapshot()] == [2, 3]
    path = tmp_path / "trace.csv"
    trace.write_csv(path)
    assert "iq_target_a" in path.read_text()
```

Run:

```powershell
python -m pytest tests -q
```

Expected after implementation: PASS.

- [ ] **Step 5: Commit only the new Python contract files**

```powershell
git add tools/current_loop_tuner
 git commit -m "test: define current loop tuner data contracts"
```

If the dirty worktree prevents a clean commit, leave unrelated files untouched and report the exact status instead of staging them.

### Task 2: Add the firmware command state and compact telemetry

**Files:**
- Create: `src/control/foc_tune_protocol.h`
- Create: `src/control/foc_tune_protocol.c`
- Modify: `src/main.c:1-110, 520-540, 770-895, 910-942`
- Modify: `src/motor/motor_config.h:40-60`
- Create: `test/test_foc_tune_protocol_contract.py`

**Interfaces:**
- `esp_err_t foc_tune_protocol_start(void)` creates the console input task and initializes target 0 A and telemetry rate 20 Hz.
- `float foc_tune_get_iq_target_a(void)` returns the protected runtime target.
- `uint32_t foc_tune_get_telemetry_rate_hz(void)` returns the protected telemetry rate.
- `uint32_t foc_tune_get_command_sequence(void)` increments when a valid IQ/STOP command is accepted.
- `void foc_tune_protocol_report_status(uint32_t fault_code)` prints the current status response when requested.

- [ ] **Step 1: Write the failing static contract test before editing C**

```python
from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_protocol_has_bounded_iq_command_and_boot_zero():
    source = (ROOT / "src/control/foc_tune_protocol.c").read_text()
    assert "FOC_TUNE_MIN_IQ_MA" in source
    assert "FOC_TUNE_MAX_IQ_MA" in source
    assert "iq_target_a = 0.0f" in source


def test_main_reads_runtime_target_and_emits_compact_data():
    source = (ROOT / "src/main.c").read_text()
    assert "foc_tune_get_iq_target_a" in source
    assert "FOC_DATA," in source
    assert "foc_tune_get_telemetry_rate_hz" in source
```

- [ ] **Step 2: Run the contract test and verify RED**

```powershell
python -m pytest test/test_foc_tune_protocol_contract.py -q
```

Expected: FAIL because the protocol module and compact output do not exist.

- [ ] **Step 3: Implement `foc_tune_protocol.c/.h`**

Use a `portMUX_TYPE` lock around `float iq_target_a`, `uint32_t telemetry_rate_hz`, `uint32_t command_sequence`, and the pending status request. Start one FreeRTOS task that reads newline-terminated input from the already configured console stream at 115200 baud. Trim CR/LF, reject lines longer than 64 bytes, accept exactly `FOC IQ <signed integer>`, `FOC STOP`, `FOC RATE <integer>`, and `FOC STATUS`, and print the response strings from the design spec. Do not call `uart_driver_install`; UART0 is already the ESP-IDF console.

- [ ] **Step 4: Connect runtime Iq to the current-loop task**

In `src/main.c`, include the new header, replace the compile-time `FOC_TEST_IQ_REF_A` target assignment with `foc_tune_get_iq_target_a()`, and keep the existing `M1_CURRENT_REFERENCE_RAMP_A_PER_S` ramp. Initialize the protocol before creating the current task. Change the runtime default in `motor_config.h` to `M1_CURRENT_LOOP_TEST_IQ_REF_A 0.0f` only if that macro remains used as a fallback; the command-state initializer is the authoritative boot-zero behavior.

- [ ] **Step 5: Add compact telemetry without changing the existing one-second diagnostic line**

Add a rate accumulator or deadline to `foc_current_telemetry_task`, copy the existing protected snapshot once per iteration, and emit a single compact line when the configured period expires:

```c
ESP_LOGI("FOC_DATA", "FOC_DATA,%lld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu",
    (long long)esp_timer_get_time(),
    foc_trace_milli(snapshot.iq_target_a),
    foc_trace_milli(snapshot.iq_ref_a),
    foc_trace_milli(snapshot.iq_a),
    foc_trace_milli(snapshot.id_a),
    foc_trace_milli(snapshot.vq_v),
    foc_trace_milli(snapshot.mechanical_speed_rad_s),
    (unsigned long)snapshot.current_age_us,
    (unsigned long)snapshot.angle_age_us,
    (unsigned long)snapshot.fault_code);
```

Use the configured rate to select the delay and preserve the existing human-readable `FOC_TUNE` line at one second. Keep the output below 115200-baud capacity at 100 Hz.

- [ ] **Step 6: Run firmware build and static contracts**

```powershell
python -m pytest test/test_foc_tune_protocol_contract.py -q
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run
```

Expected: contract tests PASS and PlatformIO exits with code 0. Existing flash-size and unused-function warnings may remain; do not change unrelated settings in this task.

- [ ] **Step 7: Commit the firmware protocol task**

```powershell
git add src/control/foc_tune_protocol.c src/control/foc_tune_protocol.h src/main.c src/motor/motor_config.h test/test_foc_tune_protocol_contract.py
git commit -m "feat: add runtime iq tuning protocol"
```

Stage only these paths so the user's existing unrelated modifications remain untouched.

### Task 3: Implement the serial worker and GUI

**Files:**
- Create: `tools/current_loop_tuner/current_loop_tuner/serial_worker.py`
- Create: `tools/current_loop_tuner/current_loop_tuner/app.py`
- Create: `tools/current_loop_tuner/requirements.txt`
- Create: `tools/current_loop_tuner/README.md`
- Create: `tools/current_loop_tuner/run_tuner.ps1`
- Create: `tools/current_loop_tuner/tests/test_serial_worker.py`

**Interfaces:**
- `SerialWorker(port: str, baudrate: int, incoming: queue.Queue, outgoing: queue.Queue)` owns the serial thread and exposes `start()`, `send(data: bytes)`, and `stop() -> None`.
- The worker posts `("telemetry", TelemetrySample)`, `("response", Response)`, `("log", str)`, and `("error", str)` tuples to `incoming`.
- `CurrentLoopApp` creates a `SerialWorker`, polls `incoming` every 50 ms, maintains a `TraceBuffer(maxlen=5000)`, and schedules Matplotlib redraws every 100 ms.

- [ ] **Step 1: Write the failing worker test with a fake serial object**

```python
class FakeSerial:
    def __init__(self):
        self.writes = []
        self.closed = False
    def write(self, data):
        self.writes.append(data)
    def readline(self):
        return b""
    def close(self):
        self.closed = True


def test_worker_writes_commands_without_touching_gui():
    fake = FakeSerial()
    worker = SerialWorker.from_serial_for_test(fake, queue.Queue())
    worker.send(b"FOC IQ 100\\n")
    assert fake.writes == [b"FOC IQ 100\\n"]
```

- [ ] **Step 2: Run the worker test and verify RED**

```powershell
python -m pytest tests/test_serial_worker.py -q
```

Expected: FAIL because the worker does not exist.

- [ ] **Step 3: Implement the worker**

Use `serial.Serial(port, baudrate, timeout=0.1)`, a daemon reader thread, and a thread-safe outgoing queue. Decode UTF-8 with replacement, split on lines, feed each line to `parse_line` and `parse_response`, and route unknown lines as logs. `stop()` must set an event, join for at most one second, and close the serial object. The test-only constructor must inject a fake object without opening hardware.

- [ ] **Step 4: Implement the Tkinter/Matplotlib view**

Build a single-window layout with port scan, baud entry defaulting to 115200, Connect/Disconnect, Iq target mA entry defaulting to 0, Apply, STOP/Zero, rate entry defaulting to 50, status text, stale-data indicator, and a five-trace plot. On connect, clear the trace, start the worker, send `FOC RATE 50`, and do not send Iq. Apply sends `format_set_iq(value)`. STOP sends `format_stop()`. Close sends a best-effort STOP, stops the worker, then destroys the window. Use `after(50, poll_queue)` and never update widgets from the serial thread.

- [ ] **Step 5: Add dependency and usage documentation**

`requirements.txt` contains:

```text
pyserial>=3.5
matplotlib>=3.8
pytest>=8.0
```

`README.md` documents PowerShell setup and run commands:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m current_loop_tuner.app
```

The README states that the motor must be mechanically safe, target starts at zero, and no firmware upload is performed by the tool.

- [ ] **Step 6: Run Python tests and a GUI import smoke test**

```powershell
python -m pytest tests -q
python -c "import tkinter; import matplotlib; import serial; from current_loop_tuner.app import CurrentLoopApp; print('GUI IMPORT OK')"
```

Expected: all tests pass and the second command prints `GUI IMPORT OK` without opening a window.

- [ ] **Step 7: Commit only the upper-computer files**

```powershell
git add tools/current_loop_tuner
git commit -m "feat: add current loop tuner gui"
```

### Task 4: Verify the end-to-end contract and hand off

**Files:**
- Modify: `tools/current_loop_tuner/README.md`
- Create: `tools/current_loop_tuner/tests/test_end_to_end_contract.py`

**Interfaces:**
- The test fixture feeds a firmware-style `FOC_DATA` line and verifies that the GUI-side parser exposes the same Iq target, reference, measured Iq, Id, Vq, speed, ages, and fault fields.

- [ ] **Step 1: Write the failing end-to-end contract test**

```python
def test_firmware_data_line_round_trips_to_gui_fields():
    line = "FOC_DATA,1000,100,98,96,4,800,2500,120,80,0"
    sample = parse_line(line)
    assert sample.iq_target_a == pytest.approx(0.1)
    assert sample.iq_ref_a == pytest.approx(0.098)
    assert sample.iq_a == pytest.approx(0.096)
    assert sample.id_a == pytest.approx(0.004)
    assert sample.vq_v == pytest.approx(0.8)
    assert sample.fault == 0
```

- [ ] **Step 2: Run the test to confirm RED, then implement the smallest missing conversion**

```powershell
python -m pytest tests/test_end_to_end_contract.py -q
```

Expected first: FAIL if any field conversion is inconsistent; after the parser/model correction: PASS.

- [ ] **Step 3: Run the full local verification**

```powershell
python -m pytest tools/current_loop_tuner/tests test/test_foc_tune_protocol_contract.py -q
& 'C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe' run
```

Expected: Python tests pass and PlatformIO exits 0. Do not run `pio run -t upload`.

- [ ] **Step 4: Perform the safe bench handoff**

Start the GUI and verify that it connects while the firmware target remains zero. Apply 50 mA, then 100 mA only with the motor and power stage mechanically safe. Check that `iq_ref` ramps, measured `iq` follows, `id` remains observed rather than controlled, and pressing STOP drives the target back to zero. Stop the test if any current, angle age, fault, or unexpected speed indication becomes abnormal.

- [ ] **Step 5: Commit the final contract/documentation update**

```powershell
git add tools/current_loop_tuner/tests/test_end_to_end_contract.py tools/current_loop_tuner/README.md
git commit -m "test: verify tuner telemetry contract"
```

## Self-review checklist

- Spec coverage: command protocol, boot-zero, bounded Iq, rate-limited telemetry, GUI controls, plotting, CSV, stale-data state, no automatic flash, and existing firmware protections are covered by Tasks 1-4.
- Completeness scan: no task depends on an unspecified file, command, range, or function signature.
- Type consistency: `TelemetrySample`, `Response`, `TraceBuffer`, `SerialWorker`, and every formatter/parser signature are defined before consumers use them.
- Dirty-worktree safety: each commit stages only new tuner files or the explicitly listed firmware paths; unrelated deleted tests and existing FOC edits are not reset.
## Implemented scope additions

The approved implementation includes two-axis runtime commands and runtime alignment:

- `FOC ID <signed_mA>` controls the Id reference; `FOC IQ <signed_mA>` controls the Iq reference.
- `FOC ALIGN` is consumed by the PWM current-loop task, so the command task never manipulates PWM concurrently with the control task.
- Alignment completion returns `FOC ALIGN DONE offset=<rad>` or `FOC ALIGN ERR CALIBRATION`; both targets remain zero after alignment.
- Compact telemetry fields are `timestamp_us,id_target_mA,id_ref_mA,iq_target_mA,iq_ref_mA,id_mA,iq_mA,vd_mV,vq_mV,speed_mrad_s,current_age_us,angle_age_us,fault`.