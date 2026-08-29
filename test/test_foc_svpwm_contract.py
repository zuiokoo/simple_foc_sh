from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "control" / "foc_svpwm.h"
SOURCE = ROOT / "src" / "control" / "foc_svpwm.c"


class FocSvpwmContractTest(unittest.TestCase):
    def test_svpwm_module_exists_and_exposes_voltage_to_duty_api(self):
        self.assertTrue(HEADER.exists(), "SVPWM header has not been created")
        self.assertTrue(SOURCE.exists(), "SVPWM source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_svpwm_calculate(", header)
        self.assertIn("bus_voltage_v", header)
        self.assertIn("float *duty_u", header)
        self.assertIn("float *duty_v", header)
        self.assertIn("float *duty_w", header)
        self.assertIn("ESP_ERR_INVALID_ARG", source)
        self.assertIn("bus_voltage_v <= 0.0f", source)
        self.assertIn("v_max", source)
        self.assertIn("v_min", source)
        self.assertIn("offset", source)
        self.assertIn("duty_u", source)
        self.assertIn("duty_v", source)
        self.assertIn("duty_w", source)

    def test_svpwm_centers_zero_voltage_at_half_duty_and_limits_outputs(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("0.5f", source)
        self.assertIn("bus_voltage_v", source)
        self.assertIn("< 0.0f", source)
        self.assertIn("> 1.0f", source)


if __name__ == "__main__":
    unittest.main()
