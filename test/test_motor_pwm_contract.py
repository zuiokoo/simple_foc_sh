from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src" / "motor" / "motor_config.h"
SOURCE = ROOT / "src" / "motor" / "motor_pwm.c"


class MotorPwmContractTest(unittest.TestCase):
    def test_center_aligned_duty_uses_timer_peak_ticks(self):
        config = CONFIG.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn(
            "#define M1_PWM_COMPARE_MAX_TICKS (M1_PWM_PERIOD_TICKS / 2UL)",
            config,
        )
        self.assertIn("duty[phase] * M1_PWM_COMPARE_MAX_TICKS", source)
        self.assertNotIn("duty[phase] * M1_PWM_PERIOD_TICKS", source)


if __name__ == "__main__":
    unittest.main()
