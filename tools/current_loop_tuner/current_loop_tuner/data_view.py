from .telemetry import TelemetrySample


TABLE_COLUMNS = (
    "t (s)",
    "Id target (mA)",
    "Id measured (mA)",
    "Iq target (mA)",
    "Iq measured (mA)",
    "Vd (V)",
    "Vq (V)",
    "Speed (rad/s)",
    "Current age (us)",
    "Angle age (us)",
    "Fault",
)


def sample_to_table_row(sample: TelemetrySample, start_timestamp_us: int) -> tuple[str, ...]:
    return (
        f"{(sample.timestamp_us - start_timestamp_us) / 1_000_000.0:.3f}",
        f"{sample.id_target_a * 1000:.0f}",
        f"{sample.id_a * 1000:.0f}",
        f"{sample.iq_target_a * 1000:.0f}",
        f"{sample.iq_a * 1000:.0f}",
        f"{sample.vd_v:.3f}",
        f"{sample.vq_v:.3f}",
        f"{sample.speed_rad_s:.2f}",
        str(sample.current_age_us),
        str(sample.angle_age_us),
        str(sample.fault),
    )