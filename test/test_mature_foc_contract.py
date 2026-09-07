import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")
CONTROLLER = (ROOT / "src" / "control" / "foc_controller.c").read_text(encoding="utf-8")


class MatureFocContractTests(unittest.TestCase):
    def test_angle_estimator_is_integrated_into_main(self):
        self.assertIn('#include "control/foc_angle_estimator.h"', MAIN)
        self.assertIn("foc_angle_estimator_predict(", MAIN)
        self.assertIn("foc_angle_estimator_invalidate(", MAIN)

    def test_angle_estimator_has_wrap_safe_prediction_and_validity_state(self):
        source_path = ROOT / "src" / "control" / "foc_angle_estimator.c"
        header_path = ROOT / "src" / "control" / "foc_angle_estimator.h"
        source = source_path.read_text(encoding="utf-8")
        header = header_path.read_text(encoding="utf-8")
        self.assertIn("foc_angle_estimator_sample(", header)
        self.assertIn("foc_angle_estimator_predict(", header)
        self.assertIn("foc_angle_estimator_invalidate(", header)
        self.assertIn("delta_angle_rad > 3.14159265358979323846f", source)
        self.assertIn("sample_timestamp_us", source)
        self.assertIn("valid = false", source)

    def test_stale_angle_is_a_hard_stop_condition(self):
        self.assertIn("FOC_MAX_ANGLE_AGE_US", MAIN)
        self.assertIn("motor_pwm_stop", MAIN)
        self.assertIn("foc_controller_reset(&foc_controller)", MAIN)

    def test_vector_saturation_back_calculates_both_current_pis(self):
        self.assertIn("vd_rejected_v", CONTROLLER)
        self.assertIn("vq_rejected_v", CONTROLLER)
        self.assertIn("controller->id_pi.integral", CONTROLLER)
        self.assertIn("controller->iq_pi.integral", CONTROLLER)

    def test_current_loop_does_not_read_as5600_or_use_variable_pi_dt(self):
        match = re.search(
            r"static void foc_current_task\(void \*pvParameter\)(.*?)(?=\n/\*|\Z)",
            MAIN,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        current_task = match.group(1)
        self.assertNotIn("as5600_measure_angle_velocity", current_task)
        self.assertIn("float control_dt_s = FOC_CURRENT_TS_S;", current_task)


if __name__ == "__main__":
    unittest.main()
