from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src" / "motor" / "motor_config.h"
MAIN = ROOT / "src" / "main.c"


class CurrentStartupContractTest(unittest.TestCase):
    def test_current_reference_has_a_startup_slew_limit(self):
        config = CONFIG.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("M1_CURRENT_REFERENCE_RAMP_A_PER_S", config)
        self.assertIn("current_reference_ramp", main)
        self.assertIn("control_dt_s", main)
        self.assertNotIn("iq_ref_a = FOC_TEST_IQ_REF_A;", main)


if __name__ == "__main__":
    unittest.main()