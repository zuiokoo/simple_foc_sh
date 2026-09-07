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
        self.assertIn("on_full", source)

    def test_telemetry_exposes_pi_and_angle_diagnostics(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("id_pi_integral_v", main)
        self.assertIn("iq_pi_integral_v", main)
        self.assertIn("angle_age_us", main)
        self.assertIn("angle_velocity_mrad_s", main)
        self.assertIn("actual_period_us", main)
        self.assertIn("period_min_us", main)
        self.assertIn("period_max_us", main)
        self.assertIn("period_jitter_us", main)
        self.assertIn("iteration_start_us - previous_iteration_start_us", main)
        self.assertIn("current_sequence", main)
        self.assertIn("angle_to_current_sample_us", main)
        self.assertIn("electrical_angle_mrad", main)

    def test_angle_snapshot_is_read_and_written_coherently(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("portMUX_TYPE foc_angle_lock", main)
        self.assertIn("portENTER_CRITICAL(&foc_angle_lock)", main)
        self.assertIn("portEXIT_CRITICAL(&foc_angle_lock)", main)
        self.assertIn("angle_snapshot_valid", main)
        self.assertIn("angle_snapshot_timestamp_us", main)
    def test_angle_sensor_task_uses_fixed_period_wakeup(self):
        main = MAIN.read_text(encoding="utf-8")
        match = re.search(
            r"static void foc_angle_sensor_task\(void \*pvParameter\)(.*?)(?=\nstatic void foc_current_task)",
            main,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match, "angle-sensor task body was not found")
        angle_task = match.group(1)

        self.assertIn("esp_timer_start_periodic", angle_task)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", angle_task)
        self.assertNotIn("vTaskDelayUntil", angle_task)

    def test_current_loop_period_has_execution_headroom(self):
        main = MAIN.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("#define FOC_CURRENT_TS_S ((float)M1_CURRENT_LOOP_PWM_PERIODS / (float)M1_PWM_FREQUENCY_HZ)", main)
        self.assertIn("#define M1_CURRENT_LOOP_PWM_PERIODS 4U", config)

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

        self.assertIn("#define FOC_CURRENT_TS_S ((float)M1_CURRENT_LOOP_PWM_PERIODS / (float)M1_PWM_FREQUENCY_HZ)", main)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", current_task)
        self.assertIn("ulTaskNotifyTake(pdTRUE, 0)", current_task)
        self.assertIn("current_loop_missed_ticks", current_task)
        self.assertIn("current_loop_overrun_count", current_task)
        self.assertIn("foc_latest_angle_timestamp_us", main)
        self.assertNotIn("control_dt_s = actual_period_us > 0U", current_task)
        self.assertIn("controller_input.dt_s = control_dt_s;", current_task)
        self.assertIn("float control_dt_s = FOC_CURRENT_TS_S;", current_task)
        self.assertNotIn("control_dt_s = actual_period_us > 0U", current_task)
        self.assertNotIn("vTaskDelayUntil", current_task)
        self.assertNotIn("as5600_measure_angle_velocity", current_task)
        self.assertIn("#define M1_CURRENT_LOOP_PWM_PERIODS 4U", config)
        self.assertIn("xTaskCreatePinnedToCore(foc_current_task, \"foc_current_task\", 4096, NULL, 9, NULL, 1);", main)

    def test_inverse_park_prediction_uses_measured_current_age(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("current_age_us", main)
        self.assertIn("estimated_output_delay_us", main)
        self.assertIn("controller_input.output_electrical_angle_rad", main)
    def test_realtime_current_loop_does_not_print_when_running(self):
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("#define M1_ENABLE_TIMING_LOG 0", config)
        self.assertIn("#define M1_ENABLE_CURRENT_TRACE 0", config)
    def test_current_loop_is_pinned_away_from_main_cpu(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn(
            "xTaskCreatePinnedToCore(foc_current_task, \"foc_current_task\", 4096, NULL, 9, NULL, 1);",
            main,
        )


    def test_current_loop_aligns_angle_to_current_frame_timestamp(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("current_sense_read_three_phase_at_or_before_timestamp(", main)
        self.assertIn("current_sample_timestamp_us", main)
        self.assertIn("current_sample_timestamp_us", main)
        self.assertIn("foc_latest_angle_timestamp_us", main)
        self.assertIn("angle_to_current_sample_us", main)
        self.assertIn("current_age_us", main)

    def test_control_tick_uses_pwm_center_event_timestamp(self):
        source = PWM_SOURCE.read_text(encoding="utf-8")
        header = PWM_HEADER.read_text(encoding="utf-8")

        self.assertIn("MCPWM_TIMER_EVENT_FULL", source)
        self.assertIn(".on_full = motor_pwm_on_full", source)
        self.assertIn("motor_pwm_get_control_tick_timestamp_us", header)

    def test_current_loop_anchors_sample_selection_to_pwm_event(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn("motor_pwm_get_control_tick_timestamp_us()", main)
        self.assertIn("current_target_timestamp_us = iteration_start_us;", main)
        self.assertIn("current_target_timestamp_us,", main)
if __name__ == "__main__":
    unittest.main()
