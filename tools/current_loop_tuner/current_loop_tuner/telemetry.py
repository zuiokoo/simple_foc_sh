from dataclasses import dataclass
import re


@dataclass(frozen=True)
class TelemetrySample:
    timestamp_us: int
    id_target_a: float
    iq_target_a: float
    id_a: float
    iq_a: float
    vd_v: float
    vq_v: float
    speed_rad_s: float
    current_age_us: int
    angle_age_us: int
    fault: int
    raw_line: str = ""


def _ma(value: str) -> float:
    return float(value) / 1000.0


def _parse_compact(line: str) -> TelemetrySample | None:
    payload = line.split("FOC_DATA,", 1)[1].strip()
    parts = [part.strip() for part in payload.split(",")]
    try:
        if len(parts) == 11:
            timestamp_us, id_target, iq_target, id_value, iq_value, vd, vq, speed, current_age, angle_age, fault = [
                int(value) for value in parts
            ]
        elif len(parts) == 13:
            # Keep old CSV/log captures readable; the intermediate references
            # are intentionally ignored.
            timestamp_us, id_target, _id_ref, iq_target, _iq_ref, id_value, iq_value, vd, vq, speed, current_age, angle_age, fault = [
                int(value) for value in parts
            ]
        else:
            return None
    except ValueError:
        return None
    return TelemetrySample(
        timestamp_us=timestamp_us,
        id_target_a=_ma(str(id_target)),
        iq_target_a=_ma(str(iq_target)),
        id_a=_ma(str(id_value)),
        iq_a=_ma(str(iq_value)),
        vd_v=_ma(str(vd)),
        vq_v=_ma(str(vq)),
        speed_rad_s=float(speed) / 1000.0,
        current_age_us=current_age,
        angle_age_us=angle_age,
        fault=fault,
        raw_line=line,
    )


def _parse_tune(line: str) -> TelemetrySample | None:
    if "FOC_TUNE:" not in line:
        return None
    values = dict(re.findall(r"([A-Za-z_]+)=([-+]?\d+(?:\.\d+)?)", line))
    required = ("speed_mrad_s", "angle_age_us", "current_age_us", "iq_target_mA", "id_mA", "iq_mA", "id_err_mA", "vd_mV", "vq_mV")
    if any(key not in values for key in required):
        return None
    try:
        uptime_match = re.search(r"\((\d+)\)", line)
        timestamp_us = int(uptime_match.group(1)) * 1000 if uptime_match else 0
        id_value = float(values["id_mA"])
        id_target = id_value + float(values["id_err_mA"])
        return TelemetrySample(
            timestamp_us=timestamp_us,
            id_target_a=id_target / 1000.0,
            iq_target_a=_ma(values["iq_target_mA"]),
            id_a=id_value / 1000.0,
            iq_a=_ma(values["iq_mA"]),
            vd_v=_ma(values["vd_mV"]),
            vq_v=_ma(values["vq_mV"]),
            speed_rad_s=float(values["speed_mrad_s"]) / 1000.0,
            current_age_us=int(values["current_age_us"]),
            angle_age_us=int(values["angle_age_us"]),
            fault=0,
            raw_line=line,
        )
    except (TypeError, ValueError):
        return None


def parse_line(line: str) -> TelemetrySample | None:
    if "FOC_DATA," in line:
        sample = _parse_compact(line)
        if sample is not None:
            return sample
    return _parse_tune(line)