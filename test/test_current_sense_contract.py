from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "sensor" / "current_sense.h"
SOURCE = ROOT / "src" / "sensor" / "current_sense.c"
MAIN = ROOT / "src" / "main.c"


class CurrentSenseContractTest(unittest.TestCase):
    def test_adc_raw_reading_module_uses_adc1_channels(self):
        self.assertTrue(HEADER.exists(), "current sense header has not been created")
        self.assertTrue(SOURCE.exists(), "current sense source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("esp_err_t current_sense_init(void);", header)
        self.assertIn("esp_err_t current_sense_calibrate(void);", header)
        self.assertIn(
            "esp_err_t current_sense_read(int *iu_raw, int *iv_raw);",
            header,
        )
        self.assertIn(
            "esp_err_t current_sense_read_amperes(float *iu_a, float *iv_a);",
            header,
        )
        self.assertIn("esp_err_t current_sense_read_three_phase(", header)
        self.assertIn("float *iw_a);", header)
        self.assertNotIn("current_sense_read_corrected", header)
        self.assertIn('#include "esp_adc/adc_continuous.h"', source)
        self.assertIn('#include "esp_adc/adc_cali_scheme.h"', source)
        self.assertIn("adc_continuous_new_handle(", source)
        self.assertIn("adc_continuous_config(", source)
        self.assertIn("adc_continuous_start(", source)
        self.assertIn("adc_continuous_read(", source)
        self.assertIn("adc_cali_create_scheme_line_fitting(", source)
        self.assertIn("adc_cali_raw_to_voltage(", source)
        self.assertIn("ADC_UNIT_1", source)
        self.assertIn("ADC_CHANNEL_0", source)
        self.assertIn("ADC_CHANNEL_3", source)
        self.assertIn("current_sense_calibrate(void)", source)
        self.assertIn("current_sense_read_amperes(", source)
        self.assertIn("current_sense_read_three_phase(", source)
        self.assertIn("float sample_iw_a = -(sample_iu_a + sample_iv_a);", source)
        self.assertNotIn("current_sense_read_corrected", source)
        self.assertIn("CURRENT_SENSE_CALIBRATION_SAMPLES", source)
        self.assertIn("CURRENT_SENSE_MILLIVOLTS_PER_AMP", source)
        self.assertIn("current_sense_raw_to_voltage", source)
        self.assertIn("current_u_zero_voltage_mv", source)
        self.assertIn("current_v_zero_voltage_mv", source)
        self.assertIn("current_sense_calibrated", source)
        self.assertIn("motor_pwm_wait_lowside_window", source)
        self.assertNotIn("ADC_UNIT_2", source)

    def test_main_initializes_and_reads_current_sense(self):
        main = MAIN.read_text(encoding="utf-8")

        self.assertIn('#include "sensor/current_sense.h"', main)
        self.assertIn("current_sense_init();", main)
        self.assertIn("current_sense_calibrate();", main)
        self.assertIn("current_sense_read_three_phase(", main)
        self.assertIn(
            '"iu: %.3f, iv: %.3f, iw: %.3f, "',
            main,
        )
        self.assertIn('"ialpha: %.3f, ibeta: %.3f, "', main)
        self.assertIn("FOC_TEST_CURRENT_LIMIT_A", main)
        self.assertIn("foc_controller_reset(&foc_controller);", main)
        self.assertIn('#include "esp_timer.h"', main)
        self.assertIn("esp_timer_get_time()", main)
        self.assertIn("FOC_CURRENT_LOOP_PERIOD_MS", main)
        self.assertIn("controller_input.dt_s = control_dt_s;", main)
        self.assertIn("pdMS_TO_TICKS(FOC_CURRENT_LOOP_PERIOD_MS)", main)
        self.assertNotIn("current_sense_read_corrected", main)


if __name__ == "__main__":
    unittest.main()
