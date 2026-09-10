import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]
CONFIG = (ROOT / "src" / "motor" / "motor_config.h").read_text(encoding="utf-8")
PROTOCOL = (ROOT / "src" / "control" / "foc_tune_protocol.c").read_text(encoding="utf-8")
APP = (ROOT / "tools" / "current_loop_tuner" / "current_loop_tuner" / "app.py").read_text(encoding="utf-8")
CONTROLLER = (ROOT / "src" / "control" / "foc_controller.h").read_text(encoding="utf-8")


class IdCurrentLoopConfigTest(unittest.TestCase):
    def test_id_loop_is_enabled_for_this_bench_test(self):
        self.assertIn("#define M1_ENABLE_ID_CURRENT_LOOP 1", CONFIG)

    def test_protocol_starts_from_id_50ma_default(self):
        self.assertIn('#include "motor/motor_config.h"', PROTOCOL)
        self.assertIn("foc_tune_id_target_a = M1_CURRENT_LOOP_TEST_ID_REF_A;", PROTOCOL)

    def test_controller_input_uses_target_names_without_ref_fields(self):
        self.assertIn("float id_target_a;", CONTROLLER)
        self.assertIn("float iq_target_a;", CONTROLLER)
        self.assertNotIn("id_ref_a", CONTROLLER)
        self.assertNotIn("iq_ref_a", CONTROLLER)

    def test_upper_computer_defaults_to_id_50ma_and_iq_zero(self):
        self.assertIn('self.id_var = tk.StringVar(value="50")', APP)
        self.assertIn('self.iq_var = tk.StringVar(value="0")', APP)

    def test_bench_reference_is_id_50ma_and_iq_zero(self):
        self.assertIn("#define M1_CURRENT_LOOP_TEST_ID_REF_A 0.05f", CONFIG)
        self.assertIn("#define M1_CURRENT_LOOP_TEST_IQ_REF_A 0.0f", CONFIG)
        self.assertIn("#define M1_ELECTRICAL_ZERO_TRIM_RAD 0.00f", CONFIG)


if __name__ == "__main__":
    unittest.main()
