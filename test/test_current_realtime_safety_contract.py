import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CURRENT = (ROOT / "src" / "sensor" / "current_sense.c").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")


class CurrentRealtimeSafetyContractTests(unittest.TestCase):
    def test_adc_timestamp_keeps_full_esp_timer_precision(self):
        self.assertIn(
            "static volatile int64_t current_adc_last_conv_done_timestamp_us",
            CURRENT,
        )
        self.assertIn(
            "current_adc_last_conv_done_timestamp_us = esp_timer_get_time()",
            CURRENT,
        )
        self.assertNotIn(
            "current_adc_last_conv_done_timestamp_us = (uint32_t)esp_timer_get_time()",
            CURRENT,
        )

    def test_dma_consumer_isolated_from_cpu1_current_loop(self):
        self.assertIn("&current_adc_task_handle, 0)", CURRENT)

    def test_stale_current_cannot_drive_pwm(self):
        self.assertIn("FOC_MAX_CURRENT_AGE_US", MAIN)
        self.assertIn("current_age_us > FOC_MAX_CURRENT_AGE_US", MAIN)
        self.assertIn("current_valid = 0U", MAIN)


if __name__ == "__main__":
    unittest.main()
