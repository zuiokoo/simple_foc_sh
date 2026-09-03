from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
CONFIG = ROOT / "src" / "motor" / "motor_config.h"


class ElectricalAlignmentContractTest(unittest.TestCase):
    def test_alignment_is_explicitly_guarded(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION", config)
        self.assertIn("M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION", main)

    def test_alignment_sets_duty_starts_and_stops_pwm(self):
        main = MAIN.read_text(encoding="utf-8")

        duty_index = main.index("motor_pwm_set_duty(")
        start_index = main.index("motor_pwm_start()")
        stop_index = main.index("motor_pwm_stop()")

        self.assertLess(duty_index, start_index)
        self.assertLess(start_index, stop_index)

    def test_alignment_rejects_noise_sized_current_vector(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("M1_ALIGNMENT_MIN_CURRENT_A", config)
        self.assertIn("alignment_current_magnitude", main)
        self.assertIn("alignment_current_magnitude < M1_ALIGNMENT_MIN_CURRENT_A", main)

    def test_alignment_uses_commanded_vector_for_zero(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("electrical_zero_offset = mechanical_electrical_angle;", main)
        self.assertNotIn("mechanical_electrical_angle - alignment_electrical_angle", main)

    def test_alignment_logs_electrical_zero_offset(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("foc_mechanical_to_electrical_angle(", main)
        self.assertIn("ELECTRICAL_ZERO_OFFSET", main)


if __name__ == "__main__":
    unittest.main()
