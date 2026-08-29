from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "sensor" / "as5600.c").read_text(encoding="utf-8")


class TestAs5600VelocityContract(unittest.TestCase):
    def test_velocity_uses_longer_measurement_window(self):
        self.assertIn("AS5600_VELOCITY_WINDOW_US", SOURCE)
        self.assertRegex(SOURCE, r"delta_time_us\s*>=\s*AS5600_VELOCITY_WINDOW_US")

    def test_velocity_is_not_single_1ms_difference(self):
        self.assertNotIn("*velocity_rad_s =\n\t\tdelta_angle_rad / delta_time_s;", SOURCE)


if __name__ == "__main__":
    unittest.main()
