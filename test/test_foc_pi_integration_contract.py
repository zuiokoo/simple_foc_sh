from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
HEADER = ROOT / "src" / "control" / "foc_controller.h"
SOURCE = ROOT / "src" / "control" / "foc_controller.c"


class FocPiIntegrationContractTest(unittest.TestCase):
    def test_current_pi_uses_actual_twelve_volt_bus(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("FOC_TEST_PI_OUTPUT_MIN_V (-6.0f)", main)
        self.assertIn("FOC_TEST_PI_OUTPUT_MAX_V (6.0f)", main)
        self.assertIn("FOC_TEST_BUS_VOLTAGE_V 12.0f", main)

    def test_math_task_has_current_references_and_two_pi_controllers(self):
        main = MAIN.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn('#include "control/foc_controller.h"', main)
        self.assertIn("foc_pi_controller_t id_pi", header)
        self.assertIn("foc_pi_controller_t iq_pi", header)
        self.assertIn("id_ref_a", header)
        self.assertIn("iq_ref_a", header)
        self.assertIn("id_error_a", header)
        self.assertIn("iq_error_a", header)
        self.assertIn("foc_pi_update(&controller->id_pi", source)
        self.assertIn("foc_pi_update(&controller->iq_pi", source)
        self.assertIn("vd_v", source)
        self.assertIn("vq_v", source)

    def test_park_uses_measured_angle_and_inverse_park_uses_predicted_angle(self):
        main = MAIN.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("output_electrical_angle_rad", header)
        self.assertIn("foc_park_transform(output->i_alpha_a, output->i_beta_a, input->electrical_angle_rad", source)
        self.assertIn("foc_inverse_park_transform(output->vd_v, output->vq_v, input->output_electrical_angle_rad", source)
        self.assertIn("controller_input.output_electrical_angle_rad", main)
    def test_current_controller_has_speed_decoupling_and_vector_limit(self):
        main = MAIN.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")
        config = (ROOT / "src" / "motor" / "motor_config.h").read_text(encoding="utf-8")

        self.assertIn("electrical_velocity_rad_s", header)
        self.assertIn("motor_inductance_d_h", header)
        self.assertIn("motor_inductance_q_h", header)
        self.assertIn("motor_flux_linkage_wb", header)
        self.assertIn("motor_phase_resistance_ohm", header)
        self.assertIn("vd_decoupling_v", source)
        self.assertIn("vq_decoupling_v", source)
        self.assertIn("motor_inductance_q_h * output->i_q_a", source)
        self.assertIn("motor_inductance_d_h * output->i_d_a", source)
        self.assertIn("voltage_vector_limit_v", source)
        self.assertIn("M1_MOTOR_INDUCTANCE_D_H", config)
        self.assertIn("M1_MOTOR_INDUCTANCE_Q_H", config)
        self.assertIn("M1_MOTOR_FLUX_LINKAGE_WB", config)
        self.assertIn("M1_MOTOR_PHASE_RESISTANCE_OHM", config)
        self.assertIn("controller_input.electrical_velocity_rad_s", main)


if __name__ == "__main__":
    unittest.main()
