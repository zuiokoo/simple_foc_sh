from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "control" / "foc_math.h"
SOURCE = ROOT / "src" / "control" / "foc_math.c"
CONTROLLER_SOURCE = ROOT / "src" / "control" / "foc_controller.c"
MAIN = ROOT / "src" / "main.c"


class FocMathContractTest(unittest.TestCase):
    def test_clarke_transform_module_exists_and_uses_two_current_inputs(self):
        self.assertTrue(HEADER.exists(), "FOC math header has not been created")
        self.assertTrue(SOURCE.exists(), "FOC math source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_clarke_transform(", header)
        self.assertIn("float *i_alpha_a", header)
        self.assertIn("float *i_beta_a", header)
        self.assertIn('#include <math.h>', source)
        self.assertIn("sqrtf(3.0f)", source)
        self.assertIn("*i_alpha_a = iu_a;", source)
        self.assertIn("*i_beta_a = (iu_a + 2.0f * iv_a)", source)
        self.assertIn(
            "*i_q_a = -i_alpha_a * sin_angle + i_beta_a * cos_angle;",
            source,
        )
        self.assertNotIn(
            "*i_q_a = -i_beta_a * sin_angle + i_beta_a * cos_angle;",
            source,
        )
        self.assertIn("ESP_ERR_INVALID_ARG", source)

    def test_main_reads_three_phase_current_and_runs_clarke_transform(self):
        main = MAIN.read_text(encoding="utf-8")
        controller = CONTROLLER_SOURCE.read_text(encoding="utf-8")

        self.assertIn('#include "control/foc_math.h"', main)
        self.assertIn("current_sense_read_three_phase(", main)
        self.assertIn("foc_clarke_transform(", controller)
        self.assertIn("foc_park_transform(", controller)

    def test_main_logs_measured_velocity(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn('"speed_ref: %.3f, speed: %.3f, iq_ref: %.3f, "', main)
        self.assertIn("mechanical_velocity_rad_s", main)

    def test_inverse_park_transform_is_declared_and_used_after_pi(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")
        controller = CONTROLLER_SOURCE.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("foc_inverse_park_transform(", header)
        self.assertIn("float v_d_v", header)
        self.assertIn("float v_q_v", header)
        self.assertIn("float *v_alpha_v", header)
        self.assertIn("float *v_beta_v", header)
        self.assertIn("*v_alpha_v = v_d_v * cos_angle - v_q_v * sin_angle;", source)
        self.assertIn("*v_beta_v = v_d_v * sin_angle + v_q_v * cos_angle;", source)
        self.assertIn("foc_inverse_park_transform(", controller)
        self.assertIn("v_alpha_v", controller)
        self.assertIn("v_beta_v", controller)
        self.assertLess(
            controller.index("foc_pi_update(&controller->id_pi"),
            controller.index("foc_inverse_park_transform("),
        )

    def test_inverse_clarke_transform_is_declared(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_inverse_clarke_transform(", header)
        self.assertIn("float v_alpha_v", header)
        self.assertIn("float v_beta_v", header)
        self.assertIn("float *u_voltage_v", header)
        self.assertIn("float *v_voltage_v", header)
        self.assertIn("float *w_voltage_v", header)
        self.assertIn("*u_voltage_v=v_alpha_v;", source)
        self.assertIn("*v_voltage_v=-0.5f*v_alpha_v", source)
        self.assertIn("*w_voltage_v=-0.5f*v_alpha_v", source)

    def test_main_uses_inverse_clarke_after_inverse_park(self):
        controller = CONTROLLER_SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_inverse_park_transform(", controller)
        self.assertIn("foc_inverse_clarke_transform(", controller)
        self.assertLess(
            controller.index("foc_inverse_park_transform("),
            controller.index("foc_inverse_clarke_transform("),
        )


if __name__ == "__main__":
    unittest.main()
