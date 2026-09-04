from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "sensor" / "as5600.c"


class As5600VelocityContractTest(unittest.TestCase):
    def test_velocity_is_updated_from_each_sensor_sample_with_a_low_pass(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("AS5600_VELOCITY_FILTER_TAU_S", source)
        self.assertIn("delta_angle_rad / delta_time_s", source)
        self.assertIn("as5600_velocity_estimate_rad_s +=", source)
        self.assertNotIn("AS5600_VELOCITY_WINDOW_US", source)


if __name__ == "__main__":
    unittest.main()
