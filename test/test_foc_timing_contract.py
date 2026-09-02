from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
PWM_SOURCE = ROOT / "src" / "motor" / "motor_pwm.c"
PWM_HEADER = ROOT / "src" / "motor" / "motor_pwm.h"


class FocTimingContractTest(unittest.TestCase):
    def test_pwm_exposes_a_control_task_notification_tick(self):
        source = PWM_SOURCE.read_text(encoding="utf-8")
        header = PWM_HEADER.read_text(encoding="utf-8")

        self.assertIn("mcpwm_timer_register_event_callbacks(", source)
        self.assertIn("vTaskNotifyGiveFromISR", source)
        self.assertIn("motor_pwm_register_control_task", header)
        self.assertIn("motor_pwm_control_tick_count", header)
        self.assertIn("on_empty", source)

    def test_current_loop_uses_pwm_notification_and_fixed_sample_time(self):
        main = MAIN.read_text(encoding="utf-8")
        match = re.search(
            r"static void foc_current_task\(void \*pvParameter\)(.*?)(?=\n/\* 参数化电压|\Z)",
            main,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match, "current-loop task body was not found")
        current_task = match.group(1)

        self.assertIn("#define FOC_CURRENT_TS_S 0.0001f", main)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", current_task)
        self.assertIn("controller_input.dt_s = FOC_CURRENT_TS_S;", current_task)
        self.assertNotIn("vTaskDelayUntil", current_task)
        self.assertNotIn("as5600_measure_angle_velocity", current_task)


if __name__ == "__main__":
    unittest.main()
