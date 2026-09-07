from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "sensor" / "current_sense.h"
SOURCE = ROOT / "src" / "sensor" / "current_sense.c"
MAIN = ROOT / "src" / "main.c"


class CurrentSenseContractTest(unittest.TestCase):
    def test_adc_dma_publishes_complete_frame_without_pwm_polling(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("typedef struct", header)
        self.assertIn("current_sense_frame_t", header)
        self.assertIn("current_sense_read_latest_frame", header)
        self.assertIn("adc_continuous_register_event_callbacks(", source)
        self.assertIn("on_conv_done", source)
        self.assertIn("on_pool_ovf", source)
        self.assertIn("#define CURRENT_ADC_FRAME_SIZE_BYTES 32U", source)
        self.assertIn("sizeof(adc_digi_output_data_t)", source)
        self.assertIn("xTaskCreatePinnedToCore(", source)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", source)
        self.assertNotIn("motor_pwm_wait_lowside_window", source)
        self.assertNotIn("motor_pwm_wait_count_rising", source)
        self.assertNotIn("current_sense_phase_sweep", source)


    def test_adc_raw_reading_module_uses_adc1_channels(self):
        self.assertTrue(HEADER.exists(), "current sense header has not been created")
        self.assertTrue(SOURCE.exists(), "current sense source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("esp_err_t current_sense_init(void);", header)
        self.assertIn("esp_err_t current_sense_calibrate(void);", header)
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
        self.assertIn("current_sense_read_three_phase(", source)
        self.assertIn("*iw_a = -(*iu_a + *iv_a);", source)
        self.assertNotIn("current_sense_read_corrected", source)
        self.assertIn("#define CURRENT_SENSE_CALIBRATION_SAMPLES 100", source)
        self.assertIn("CURRENT_SENSE_MILLIVOLTS_PER_AMP", source)
        self.assertIn("current_sense_raw_to_voltage", source)
        self.assertIn("current_u_zero_voltage_mv", source)
        self.assertIn("current_v_zero_voltage_mv", source)
        self.assertIn("current_sense_calibrated", source)
        self.assertNotIn("motor_pwm_wait_lowside_window", source)
        self.assertNotIn("ADC_UNIT_2", source)

    def test_realtime_current_read_uses_dma_published_amperes(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("current_latest_iu_a", source)
        self.assertIn("current_latest_iv_a", source)
        self.assertIn("current_sense_raw_to_voltage(frame.iu_raw", source)
        read_body = source.split("esp_err_t current_sense_read_three_phase_with_timestamp_and_sequence(", 1)[1]
        self.assertIn("current_latest_iu_a", read_body)
        self.assertIn("current_latest_iv_a", read_body)
        self.assertNotIn("adc_continuous_read(", read_body)
    def test_classic_esp32_frame_is_4_results_for_200khz(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("#define CURRENT_ADC_SAMPLE_FREQ_HZ 200000U", source)
        self.assertIn("#define CURRENT_ADC_FRAME_SIZE_BYTES 32U", source)
        self.assertIn("#define CURRENT_ADC_MAX_STORE_BUF_SIZE_BYTES 4096U", source)
        self.assertNotIn("static const uint32_t current_adc_frame_size_bytes", source)


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
        self.assertIn("FOC_CURRENT_TS_S", main)
        self.assertIn("controller_input.dt_s = control_dt_s;", main)
        self.assertIn("ulTaskNotifyTake(pdTRUE, portMAX_DELAY)", main)
        self.assertNotIn("current_sense_read_corrected", main)


    def test_current_frame_timestamp_represents_sample_center(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("frame_end_timestamp_us", source)
        self.assertIn("frame_duration_us", source)
        self.assertIn("frame_duration_us / 2", source)
        self.assertNotIn("20 results", source)
    def test_frame_center_uses_measured_dma_cadence_when_available(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("current_adc_previous_frame_end_timestamp_us", source)
        self.assertIn("measured_frame_duration_us", source)
        self.assertIn("frame_end_timestamp_us - current_adc_previous_frame_end_timestamp_us", source)
    def test_current_frame_timestamp_is_returned_with_currents(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("current_sense_read_three_phase_with_timestamp(", header)
        self.assertIn("current_sense_read_three_phase_with_timestamp(", source)
        self.assertIn("current_latest_frame.timestamp_us", source)

    def test_dma_frame_timestamp_comes_from_conversion_event(self):
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("current_adc_last_conv_done_timestamp_us", source)
        self.assertIn(
            "current_adc_last_conv_done_timestamp_us = esp_timer_get_time()",
            source,
        )
        self.assertIn("current_sense_take_frame_timestamp()", source)
        self.assertIn("int64_t frame_end_timestamp_us)", source)
    def test_realtime_current_read_can_return_frame_sequence(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn(
            "current_sense_read_three_phase_with_timestamp_and_sequence(",
            header,
        )
        self.assertIn(
            "current_sense_read_three_phase_with_timestamp_and_sequence(",
            source,
        )
        self.assertIn("*sequence = current_latest_frame.sequence", source)

    def test_current_consumer_can_select_frame_before_control_event(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn(
            "current_sense_read_three_phase_at_or_before_timestamp(",
            header,
        )
        self.assertIn(
            "current_sense_read_three_phase_at_or_before_timestamp(",
            source,
        )
        self.assertIn("current_frame_history", source)
        self.assertIn("timestamp_us <= target_timestamp_us", source)
        self.assertIn("selected_frame.iu_a", source)
        self.assertIn("selected_frame.iv_a", source)
        self.assertIn("xTaskCreatePinnedToCore", source)
if __name__ == "__main__":
    unittest.main()
