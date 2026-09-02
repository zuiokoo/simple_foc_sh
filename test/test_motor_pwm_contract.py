from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src" / "motor" / "motor_config.h"
SOURCE = ROOT / "src" / "motor" / "motor_pwm.c"


class MotorPwmContractTest(unittest.TestCase):
    def test_unused_compare_phase_helper_is_not_part_of_pwm_api(self):
        header = (ROOT / "src" / "motor" / "motor_pwm.h").read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertNotIn("motor_pwm_max_compare_phase", header)
        self.assertNotIn("motor_pwm_max_compare_phase", source)


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
