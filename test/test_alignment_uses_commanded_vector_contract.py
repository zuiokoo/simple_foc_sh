import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")


class AlignmentUsesCommandedVectorContractTest(unittest.TestCase):
    def test_alignment_does_not_derive_zero_from_measured_current_angle(self):
        self.assertNotIn("alignment_current_angle", MAIN)
        self.assertIn("alignment_vector_angle_rad = atan2f(", MAIN)
        self.assertIn("M1_ALIGNMENT_DUTY_U - alignment_duty_average", MAIN)


if __name__ == "__main__":
    unittest.main()
