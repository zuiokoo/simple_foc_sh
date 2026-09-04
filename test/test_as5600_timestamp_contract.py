from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / 'src' / 'main.c'
SOURCE = ROOT / 'src' / 'sensor' / 'as5600.c'

class As5600TimestampContractTest(unittest.TestCase):
    def test_angle_failure_invalidates_feedback_and_backs_off(self):
        main = (ROOT / "src" / "main.c").read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("foc_latest_angle_valid = false", main)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(2))", main)
        self.assertIn("AS5600_I2C_TIMEOUT_MS 2", source)
        self.assertIn("as5600_velocity_initialized = 0", source)


    def test_angle_snapshot_timestamp_uses_i2c_transaction_midpoint(self):
        text = MAIN.read_text(encoding='utf-8')
        self.assertIn('angle_read_start_us', text)
        self.assertIn('angle_read_end_us', text)
        self.assertIn('(angle_read_start_us + angle_read_end_us) / 2', text)
    def test_angle_task_is_driven_by_sub_millisecond_timer_notification(self):
        text = MAIN.read_text(encoding='utf-8')
        self.assertIn('esp_timer_start_periodic', text)
        self.assertIn('ulTaskNotifyTake(pdTRUE, portMAX_DELAY)', text)
        self.assertNotIn('vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1))', text)

if __name__ == '__main__':
    unittest.main()