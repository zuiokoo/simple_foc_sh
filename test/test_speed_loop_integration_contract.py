from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
CONFIG = ROOT / "src" / "motor" / "motor_config.h"


class SpeedLoopIntegrationContractTest(unittest.TestCase):
    def test_speed_loop_feeds_iq_reference_into_current_loop(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn('#include "control/foc_speed_pi.h"', main)
        self.assertIn("foc_speed_pi_t", main)
        self.assertIn("M1_SPEED_REF_RAD_S", config)
        self.assertIn("M1_SPEED_PI_KP", config)
        self.assertIn("M1_SPEED_PI_KI", config)
        self.assertIn("foc_speed_pi_update(", main)
        self.assertIn("controller_input.iq_ref_a = iq_ref_a;", main)
        self.assertIn("as5600_measure_angle_velocity(", main)
        self.assertLess(
            main.index("foc_speed_pi_update("),
            main.index("foc_controller_step("),
        )

        self.assertIn("M1_ENABLE_SPEED_LOOP", config)

    def test_speed_loop_debug_configuration_is_explicit(self):
        config = CONFIG.read_text(encoding="utf-8")
        self.assertIn("#define M1_ENABLE_SPEED_LOOP 0", config)
        self.assertIn("#define M1_SPEED_REF_RAD_S 0.0f", config)
        self.assertIn("#define M1_SPEED_IQ_LIMIT_A 0.02f", config)
        self.assertIn("#define M1_CURRENT_LOOP_TEST_ID_REF_A 0.0f", config)
        self.assertIn("#define M1_CURRENT_LOOP_TEST_IQ_REF_A 0.20f", config)
        self.assertIn("#define M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION 1", config)


if __name__ == "__main__":
    unittest.main()
