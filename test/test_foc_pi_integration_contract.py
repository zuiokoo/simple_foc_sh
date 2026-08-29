from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
HEADER = ROOT / "src" / "control" / "foc_controller.h"
SOURCE = ROOT / "src" / "control" / "foc_controller.c"


class FocPiIntegrationContractTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
