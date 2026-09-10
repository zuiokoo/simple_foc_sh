import unittest
from pathlib import Path


ROOT = Path(__file__).parents[1]
SOURCE = (ROOT / "src" / "sensor" / "current_sense.c").read_text(encoding="utf-8")
DMA_TASK = SOURCE[
    SOURCE.index("static void current_sense_dma_task"):
    SOURCE.index("esp_err_t current_sense_init")
]


class CurrentSenseDmaWatchdogContractTest(unittest.TestCase):
    def test_dma_task_consumes_one_notification_per_adc_frame(self):
        self.assertIn("ulTaskNotifyTake(pdFALSE, portMAX_DELAY)", DMA_TASK)
        self.assertNotIn(
            "while (1)\\n        {\\n            uint32_t bytes_read",
            DMA_TASK,
        )


if __name__ == "__main__":
    unittest.main()
