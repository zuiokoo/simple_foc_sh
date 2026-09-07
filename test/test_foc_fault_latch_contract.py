import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")


class FocFaultLatchContractTests(unittest.TestCase):
    def test_runtime_fault_is_latched_and_reported_without_deleting_control_task(self):
        self.assertIn("static void foc_current_latch_fault(uint32_t fault_code)", MAIN)
        self.assertIn("foc_current_latch_fault(1U);", MAIN)
        self.assertIn("foc_current_latch_fault(2U);", MAIN)
        self.assertIn("foc_current_snapshot.fault_code = fault_code;", MAIN)
        self.assertIn("foc_current_snapshot.current_valid = 0U;", MAIN)
        self.assertNotIn("vTaskDelete(NULL);\n\t\t\treturn;", MAIN[MAIN.find("Over-current or invalid current"):])

    def test_latched_fault_keeps_telemetry_task_alive_until_reset(self):
        latch_start = MAIN.index("static void foc_current_latch_fault")
        latch_end = MAIN.index("static void foc_current_task", latch_start)
        latch = MAIN[latch_start:latch_end]
        self.assertIn("motor_pwm_stop();", latch)
        self.assertIn("while (true)", latch)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(1000));", latch)
        self.assertIn('current_valid=%lu angle_stale=%lu current_stale=%lu fault=%lu current_sequence=%lu', MAIN)


if __name__ == "__main__":
    unittest.main()
