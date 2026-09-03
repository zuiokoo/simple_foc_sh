from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
PWM_SOURCE = ROOT / "src" / "motor" / "motor_pwm.c"
PWM_HEADER = ROOT / "src" / "motor" / "motor_pwm.h"
CONFIG = ROOT / "src" / "motor" / "motor_config.h"


class FocTimingContractTest(unittest.TestCase):
    def test_pwm_exposes_a_control_task_notification_tick(self):
        source = PWM_SOURCE.read_text(encoding="utf-8")
        header = PWM_HEADER.read_text(encoding="utf-8")

        self.assertIn("mcpwm_timer_register_event_callbacks(", source)
        self.assertIn("vTaskNotifyGiveFromISR", source)
        self.assertIn("motor_pwm_register_control_task", header)
        self.assertIn("motor_pwm_control_tick_count", header)
        self.assertIn("on_empty", source)

    def test_telemetry_exposes_pi_and_angle_diagnostics(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("id_pi_integral_v", main)
        self.assertIn("iq_pi_integral_v", main)
        self.assertIn("angle_age_us", main)
        self.assertIn("angle_velocity_mrad_s", main)

    def test_current_angle_prediction_has_no_unmeasured_extra_lead(self):
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("#define M1_ANGLE_PREDICTION_EXTRA_US 0U", config)
    def test_angle_prediction_does_not_add_unmeasured_delay(self):
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("#define M1_ANGLE_PREDICTION_EXTRA_US 0U", config)

    def test_angle_sensor_task_uses_fixed_period_wakeup(self):
        main = MAIN.read_text(encoding="utf-8")
        match = re.search(
            r"static void foc_angle_sensor_task\(void \*pvParameter\)(.*?)(?=\nstatic void foc_current_task)",
            main,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match, "angle-sensor task body was not found")
        angle_task = match.group(1)

        self.assertIn("vTaskDelayUntil", angle_task)
        self.assertNotIn("vTaskDelay(1)", angle_task)

    def test_current_loop_period_has_execution_headroom(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("#define FOC_CURRENT_TS_S 0.0006f", main)
        self.assertIn("#define M1_CURRENT_LOOP_PWM_PERIODS 12U", config)

    def test_current_loop_uses_pwm_notification_and_fixed_sample_time(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        match = re.search(
            r"static void foc_current_task\(void \*pvParameter\)(.*?)(?=\n/\* 参数化电压|\Z)",
            main,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match, "current-loop task body was not found")
        current_task = match.group(1)

        self.assertIn("#define FOC_CURRENT_TS_S 0.0006f", main)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", current_task)
        self.assertIn("ulTaskNotifyTake(pdTRUE, 0)", current_task)
        self.assertIn("current_loop_missed_ticks", current_task)
        self.assertIn("current_loop_overrun_count", current_task)
        self.assertIn("foc_latest_angle_timestamp_us", main)
        self.assertIn("controller_input.dt_s = FOC_CURRENT_TS_S;", current_task)
        self.assertNotIn("vTaskDelayUntil", current_task)
        self.assertNotIn("as5600_measure_angle_velocity", current_task)
        self.assertIn("#define M1_CURRENT_LOOP_PWM_PERIODS 12U", config)
        self.assertIn("xTaskCreate(foc_current_task, \"foc_current_task\", 4096, NULL, 9, NULL);", main)


    def test_current_loop_aligns_angle_to_current_frame_timestamp(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("current_sense_read_three_phase_with_timestamp(", main)
        self.assertIn("current_sample_timestamp_us", main)
        self.assertIn("current_sample_timestamp_us", main)
        self.assertIn("foc_latest_angle_timestamp_us", main)
        self.assertIn("angle_to_current_sample_us", main)
        self.assertIn("current_age_us", main)

if __name__ == "__main__":
    unittest.main()