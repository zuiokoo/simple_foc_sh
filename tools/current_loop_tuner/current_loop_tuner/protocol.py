from dataclasses import dataclass, field
import math
import re

MIN_TARGET_MA = -500
MAX_TARGET_MA = 500
MIN_RATE_HZ = 1
MAX_RATE_HZ = 100

PI_KP_MIN = 0.0
PI_KP_MAX = 10.0
PI_KI_MIN = 0.0
PI_KI_MAX = 50.0


@dataclass(frozen=True)
class Response:
    kind: str
    message: str
    value: int | None = None
    fields: dict[str, str] = field(default_factory=dict)


def _target_command(axis: str, value_ma: int) -> bytes:
    if not isinstance(value_ma, int) or not MIN_TARGET_MA <= value_ma <= MAX_TARGET_MA:
        raise ValueError("target must be in -500..500 mA")
    return f"FOC {axis} {value_ma}\n".encode("ascii")


def format_set_id(value_ma: int) -> bytes:
    return _target_command("ID", value_ma)


def format_set_iq(value_ma: int) -> bytes:
    return _target_command("IQ", value_ma)


def _validate_pi(value: float, minimum: float, maximum: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} PI must be a finite number")
    numeric = float(value)
    if not math.isfinite(numeric) or not minimum <= numeric <= maximum:
        raise ValueError(f"{name} PI must be in {minimum:g}..{maximum:g}")
    return numeric


def format_set_pi(id_kp: float, id_ki: float, iq_kp: float, iq_ki: float) -> bytes:
    values = (
        _validate_pi(id_kp, PI_KP_MIN, PI_KP_MAX, "Id Kp"),
        _validate_pi(id_ki, PI_KI_MIN, PI_KI_MAX, "Id Ki"),
        _validate_pi(iq_kp, PI_KP_MIN, PI_KP_MAX, "Iq Kp"),
        _validate_pi(iq_ki, PI_KI_MIN, PI_KI_MAX, "Iq Ki"),
    )
    return (
        f"FOC PI {values[0]:.6f} {values[1]:.6f} "
        f"{values[2]:.6f} {values[3]:.6f}\n"
    ).encode("ascii")

def format_stop() -> bytes:
    return b"FOC STOP\n"


def format_align() -> bytes:
    return b"FOC ALIGN\n"


def format_rate(rate_hz: int) -> bytes:
    if not isinstance(rate_hz, int) or not MIN_RATE_HZ <= rate_hz <= MAX_RATE_HZ:
        raise ValueError("rate must be in 1..100 Hz")
    return f"FOC RATE {rate_hz}\n".encode("ascii")


def format_status() -> bytes:
    return b"FOC STATUS\n"


def parse_response(line: str) -> Response | None:
    text = line.strip()
    if not text.startswith("FOC "):
        return None
    if text.startswith("FOC OK IQ="):
        return Response("ok_iq", text, int(text.removeprefix("FOC OK IQ=")))
    if text.startswith("FOC OK ID="):
        return Response("ok_id", text, int(text.removeprefix("FOC OK ID=")))
    if text.startswith("FOC OK PI "):
        fields = dict(re.findall(r"([A-Z_]+)=([-+]?\d+(?:\.\d+)?)", text))
        return Response("ok_pi", text, fields=fields)
    if text.startswith("FOC OK RATE="):
        return Response("ok_rate", text, int(text.removeprefix("FOC OK RATE=")))
    if text == "FOC OK STOP":
        return Response("ok_stop", text)
    if text == "FOC OK ALIGN":
        return Response("ok_align", text)
    if text.startswith("FOC ALIGN DONE"):
        match = re.search(r"offset=([-+]?\d+(?:\.\d+)?)", text)
        return Response("align_done", text, fields={"offset": match.group(1) if match else ""})
    if text.startswith("FOC ALIGN ERR"):
        return Response("align_error", text)
    if text.startswith("FOC STATUS"):
        fields = dict(re.findall(r"([A-Za-z_]+)=([^\s]+)", text))
        return Response("status", text, fields=fields)
    if text.startswith("FOC ERR"):
        return Response("error", text.removeprefix("FOC ERR "))
    return None