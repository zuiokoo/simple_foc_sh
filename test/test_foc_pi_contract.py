from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "control" / "foc_pi.h"
SOURCE = ROOT / "src" / "control" / "foc_pi.c"


class FocPiContractTest(unittest.TestCase):
    def test_pi_module_exposes_state_and_basic_api(self):
        self.assertTrue(HEADER.exists(), "FOC PI header has not been created")
        self.assertTrue(SOURCE.exists(), "FOC PI source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_pi_controller_t", header)
        self.assertIn("float kp;", header)
        self.assertIn("float ki;", header)
        self.assertIn("float integral;", header)
        self.assertIn("float output_min;", header)
        self.assertIn("float output_max;", header)
        self.assertIn("foc_pi_init(", header)
        self.assertIn("foc_pi_update(", header)
        self.assertIn("foc_pi_reset(", header)
        self.assertIn("dt_s", header)

        self.assertIn("controller->integral", source)
        self.assertIn("controller->kp", source)
        self.assertIn("controller->ki", source)
        self.assertIn("controller->output_min", source)
        self.assertIn("controller->output_max", source)

    def test_pi_limits_output_and_rejects_invalid_dt(self):
        self.assertTrue(SOURCE.exists(), "FOC PI source has not been created")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("dt_s <= 0.0f", source)
        self.assertIn("output > controller->output_max", source)
        self.assertIn("output < controller->output_min", source)


if __name__ == "__main__":
    unittest.main()
