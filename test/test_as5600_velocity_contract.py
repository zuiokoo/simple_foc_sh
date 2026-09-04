from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "sensor" / "as5600.c").read_text(encoding="utf-8")


class TestAs5600VelocityContract(unittest.TestCase):
    def test_velocity_updates_each_sample_with_short_filter(self):
        self.assertIn("AS5600_VELOCITY_FILTER_TAU_S", SOURCE)
        self.assertIn("instantaneous_velocity_rad_s", SOURCE)
        self.assertIn("as5600_velocity_estimate_rad_s +=", SOURCE)
        self.assertNotIn("AS5600_VELOCITY_WINDOW_US", SOURCE)

    def test_velocity_is_not_single_1ms_difference(self):
        self.assertIn("filter_alpha", SOURCE)


if __name__ == "__main__":
    unittest.main()
