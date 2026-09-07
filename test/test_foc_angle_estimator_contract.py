import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "control" / "foc_angle_estimator.h"
SOURCE = ROOT / "src" / "control" / "foc_angle_estimator.c"
MAIN = ROOT / "src" / "main.c"


class FocAngleEstimatorContractTests(unittest.TestCase):
    def test_estimator_module_exposes_sample_predict_and_invalidate_api(self):
        header = HEADER.read_text(encoding="utf-8")
        self.assertIn("foc_angle_estimator_t", header)
        self.assertIn("foc_angle_estimator_sample(", header)
        self.assertIn("foc_angle_estimator_predict(", header)
        self.assertIn("foc_angle_estimator_invalidate(", header)

    def test_estimator_is_wrap_safe_and_predicts_from_timestamped_sample(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("delta_angle_rad > 3.14159265358979323846f", source)
        self.assertIn("sample_timestamp_us", source)
        self.assertIn("velocity_rad_s *", source)
        self.assertIn("valid = false", source)

    def test_main_uses_estimator_for_current_and_output_angle(self):
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn('#include "control/foc_angle_estimator.h"', main)
        self.assertIn("foc_angle_estimator_predict(", main)
        self.assertIn("foc_angle_estimator_invalidate(", main)


if __name__ == "__main__":
    unittest.main()
