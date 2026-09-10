import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")
PROTO = (ROOT / "src" / "control" / "foc_tune_protocol.c").read_text(encoding="utf-8") if (ROOT / "src" / "control" / "foc_tune_protocol.c").exists() else ""


class FocTuneProtocolContractTest(unittest.TestCase):
    def test_alignment_command_is_owned_by_protocol(self):
        self.assertIn("FOC ALIGN", PROTO)
        self.assertIn("foc_tune_take_alignment_request", PROTO)

    def test_main_has_runtime_targets_and_compact_data(self):
        self.assertIn("foc_tune_get_id_target_a", MAIN)
        self.assertIn("foc_tune_get_iq_target_a", MAIN)
        self.assertIn("FOC_DATA,", MAIN)

    def test_status_reports_real_fault_state(self):
        self.assertNotIn("FAULT=0", PROTO)
        self.assertIn("foc_tune_get_fault_code", PROTO)
        self.assertIn("foc_tune_set_fault_code", MAIN)

    def test_snapshot_writer_uses_the_snapshot_lock(self):
        target = MAIN.index("foc_current_snapshot.id_target_a = id_target_a;")
        enter = MAIN.rfind("portENTER_CRITICAL(&foc_current_snapshot_lock);", 0, target)
        integral = MAIN.index("foc_current_snapshot.iq_pi_integral_v", target)
        exit = MAIN.index("portEXIT_CRITICAL(&foc_current_snapshot_lock);", integral)
        self.assertLess(enter, target)
        self.assertGreater(exit, integral)


if __name__ == "__main__":
    unittest.main()