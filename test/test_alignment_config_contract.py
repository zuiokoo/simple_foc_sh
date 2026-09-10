from pathlib import Path
import re
import unittest


CONFIG = Path(__file__).resolve().parents[1] / "src" / "motor" / "motor_config.h"


class AlignmentConfigContractTest(unittest.TestCase):
    def test_alignment_max_current_is_one_ampere_for_bench_test(self):
        text = CONFIG.read_text(encoding="utf-8")
        match = re.search(r"#define\s+M1_ALIGNMENT_MAX_CURRENT_A\s+([0-9.]+)f", text)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(float(match.group(1)), 1.0)


if __name__ == "__main__":
    unittest.main()
