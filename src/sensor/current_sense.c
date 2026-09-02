#include "current_sense.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

/* M1 current-sense hardware mapping. */
#define CURRENT_SENSE_ADC_UNIT ADC_UNIT_1
#define CURRENT_SENSE_U_CHANNEL ADC_CHANNEL_0
#define CURRENT_SENSE_V_CHANNEL ADC_CHANNEL_3

/*
 * ADC1 runs at 200 kHz.  On the classic ESP32 one logical DMA result is
 * sizeof(adc_digi_output_data_t) bytes: 12 bits of data plus 4 bits of
 * channel information.  A 40-byte frame contains 20 results (about 10 U
 * samples and 10 V samples), giving about one complete frame every 100 us.
 */
#define CURRENT_SENSE_CALIBRATION_SAMPLES 100
#define CURRENT_ADC_SAMPLE_FREQ_HZ 200000U
#define CURRENT_ADC_FRAME_SIZE_BYTES 40U
#define CURRENT_ADC_MAX_STORE_BUF_SIZE_BYTES 4096U
#define CURRENT_ADC_TASK_STACK_WORDS 3072U
#define CURRENT_ADC_TASK_PRIORITY 8U

/* 5 mOhm shunt, gain 50: 0.005 * 50 * 1000 = 250 mV/A. */
#define CURRENT_SENSE_MILLIVOLTS_PER_AMP 250.0f

static const char *TAG = "CURRENT_SENSE";

static adc_continuous_handle_t current_adc_handle = NULL;
static adc_cali_handle_t current_adc_cali_handle = NULL;
static TaskHandle_t current_adc_task_handle = NULL;

static portMUX_TYPE current_frame_lock = portMUX_INITIALIZER_UNLOCKED;
static current_sense_frame_t current_latest_frame = {0};
static bool current_frame_valid = false;
static uint32_t current_frame_sequence = 0;
static volatile uint32_t current_dma_frame_count = 0;
static volatile uint32_t current_dma_overflow_count = 0;

static int current_u_zero_voltage_mv = 0;
static int current_v_zero_voltage_mv = 0;
static bool current_sense_calibrated = false;

static esp_err_t current_sense_raw_to_voltage(int raw, int *voltage_mv)
{
	if (voltage_mv == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (current_adc_cali_handle == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}

	return adc_cali_raw_to_voltage(current_adc_cali_handle, raw, voltage_mv);
}

static bool IRAM_ATTR current_sense_on_conv_done(
	adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t *edata,
	void *user_data)
{
	(void)handle;
	(void)edata;
	(void)user_data;

	if (current_adc_task_handle == NULL)
	{
		return false;
	}

	BaseType_t higher_priority_task_woken = pdFALSE;
	vTaskNotifyGiveFromISR(current_adc_task_handle, &higher_priority_task_woken);
	return higher_priority_task_woken == pdTRUE;
}

static bool IRAM_ATTR current_sense_on_pool_overflow(
	adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t *edata,
	void *user_data)
{
	(void)handle;
	(void)edata;
	(void)user_data;

	current_dma_overflow_count++;

	if (current_adc_task_handle == NULL)
	{
		return false;
	}

	BaseType_t higher_priority_task_woken = pdFALSE;
	vTaskNotifyGiveFromISR(current_adc_task_handle, &higher_priority_task_woken);
	return higher_priority_task_woken == pdTRUE;
}

static void current_sense_publish_frame(
	const uint8_t *buffer,
	uint32_t bytes_read)
{
	int64_t sum_u = 0;
	int64_t sum_v = 0;
	uint32_t count_u = 0;
	uint32_t count_v = 0;

    for (uint32_t offset = 0;
         offset + sizeof(adc_digi_output_data_t) <= bytes_read;
         offset += sizeof(adc_digi_output_data_t))
	{
		const adc_digi_output_data_t *sample =
			(const adc_digi_output_data_t *)&buffer[offset];

        if (sample->type1.channel == CURRENT_SENSE_U_CHANNEL)
		{
			sum_u += sample->type1.data;
			count_u++;
		}
        else if (sample->type1.channel == CURRENT_SENSE_V_CHANNEL)
		{
			sum_v += sample->type1.data;
			count_v++;
		}
	}

	if (count_u == 0U || count_v == 0U)
	{
		return;
	}

	current_sense_frame_t frame = {
		.sequence = 0U,
		.timestamp_us = esp_timer_get_time(),
		.iu_raw = (int)((sum_u + (int64_t)(count_u / 2U)) / (int64_t)count_u),
		.iv_raw = (int)((sum_v + (int64_t)(count_v / 2U)) / (int64_t)count_v),
	};

	portENTER_CRITICAL(&current_frame_lock);
	frame.sequence = ++current_frame_sequence;
	current_latest_frame = frame;
	current_frame_valid = true;
	current_dma_frame_count++;
	portEXIT_CRITICAL(&current_frame_lock);
}

static void current_sense_dma_task(void *pv_parameter)
{
	(void)pv_parameter;

    uint8_t frame_buffer[CURRENT_ADC_FRAME_SIZE_BYTES];

	while (true)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		while (true)
		{
			uint32_t bytes_read = 0U;
			esp_err_t result = adc_continuous_read(
				current_adc_handle,
				frame_buffer,
				sizeof(frame_buffer),
				&bytes_read,
				0);

			if (result == ESP_ERR_TIMEOUT)
			{
				break;
			}
			if (result != ESP_OK)
			{
				ESP_LOGE(
					TAG,
					"ADC DMA read failed: %s",
					esp_err_to_name(result));
				break;
			}
			if (bytes_read == 0U)
			{
				break;
			}

			current_sense_publish_frame(frame_buffer, bytes_read);
		}
	}
}

esp_err_t current_sense_init(void)
{
	if (current_adc_handle != NULL)
	{
		ESP_LOGW(TAG, "ADC is already initialized");
		return ESP_ERR_INVALID_STATE;
	}

	current_u_zero_voltage_mv = 0;
	current_v_zero_voltage_mv = 0;
	current_sense_calibrated = false;
	current_adc_cali_handle = NULL;
	current_adc_task_handle = NULL;
	current_frame_valid = false;
	current_frame_sequence = 0U;
	current_dma_frame_count = 0U;
	current_dma_overflow_count = 0U;

	adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = CURRENT_ADC_MAX_STORE_BUF_SIZE_BYTES,
        .conv_frame_size = CURRENT_ADC_FRAME_SIZE_BYTES,
	};

	esp_err_t result =
		adc_continuous_new_handle(&handle_config, &current_adc_handle);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			TAG,
			"failed to create continuous ADC handle: %s",
			esp_err_to_name(result));
		current_adc_handle = NULL;
		return result;
	}

	adc_digi_pattern_config_t pattern[2] = {0};
	pattern[0].atten = ADC_ATTEN_DB_12;
    pattern[0].channel = CURRENT_SENSE_U_CHANNEL & 0x7U;
    pattern[0].unit = CURRENT_SENSE_ADC_UNIT;
    pattern[0].bit_width = ADC_BITWIDTH_12;
	pattern[1].atten = ADC_ATTEN_DB_12;
    pattern[1].channel = CURRENT_SENSE_V_CHANNEL & 0x7U;
    pattern[1].unit = CURRENT_SENSE_ADC_UNIT;
    pattern[1].bit_width = ADC_BITWIDTH_12;

	adc_continuous_config_t digital_config = {
        .sample_freq_hz = CURRENT_ADC_SAMPLE_FREQ_HZ,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
		.pattern_num = 2,
		.adc_pattern = pattern,
	};

	result = adc_continuous_config(current_adc_handle, &digital_config);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			TAG,
			"failed to configure continuous ADC: %s",
			esp_err_to_name(result));
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		return result;
	}

	adc_cali_line_fitting_config_t calibration_config = {
        .unit_id = CURRENT_SENSE_ADC_UNIT,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.default_vref = 1100,
	};

	result = adc_cali_create_scheme_line_fitting(
		&calibration_config,
		&current_adc_cali_handle);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			TAG,
			"failed to create ADC calibration: %s",
			esp_err_to_name(result));
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		current_adc_cali_handle = NULL;
		return result;
	}

	adc_continuous_evt_cbs_t callbacks = {
		.on_conv_done = current_sense_on_conv_done,
		.on_pool_ovf = current_sense_on_pool_overflow,
	};

	result = xTaskCreate(
				 current_sense_dma_task,
				 "current_adc_dma",
        CURRENT_ADC_TASK_STACK_WORDS,
				 NULL,
        CURRENT_ADC_TASK_PRIORITY,
				 &current_adc_task_handle) == pdPASS
				 ? ESP_OK
				 : ESP_ERR_NO_MEM;
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "failed to create ADC DMA task");
		adc_cali_delete_scheme_line_fitting(current_adc_cali_handle);
		current_adc_cali_handle = NULL;
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		return result;
	}

	result = adc_continuous_register_event_callbacks(
		current_adc_handle,
		&callbacks,
		NULL);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			TAG,
			"failed to register ADC DMA callbacks: %s",
			esp_err_to_name(result));
		vTaskDelete(current_adc_task_handle);
		current_adc_task_handle = NULL;
		adc_cali_delete_scheme_line_fitting(current_adc_cali_handle);
		current_adc_cali_handle = NULL;
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		return result;
	}

	result = adc_continuous_start(current_adc_handle);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			TAG,
			"failed to start continuous ADC: %s",
			esp_err_to_name(result));
		vTaskDelete(current_adc_task_handle);
		current_adc_task_handle = NULL;
		adc_cali_delete_scheme_line_fitting(current_adc_cali_handle);
		current_adc_cali_handle = NULL;
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		return result;
	}

	ESP_LOGI(
		TAG,
		"continuous ADC DMA started: U=ADC1_CH0(GPIO36), V=ADC1_CH3(GPIO39), "
		"sample=%u Hz, frame=%u bytes",
        CURRENT_ADC_SAMPLE_FREQ_HZ,
        CURRENT_ADC_FRAME_SIZE_BYTES);

	return ESP_OK;
}

esp_err_t current_sense_read_latest_frame(current_sense_frame_t *frame)
{
	if (frame == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (current_adc_handle == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}

	portENTER_CRITICAL(&current_frame_lock);
	bool valid = current_frame_valid;
	if (valid)
	{
		*frame = current_latest_frame;
	}
	portEXIT_CRITICAL(&current_frame_lock);

	return valid ? ESP_OK : ESP_ERR_TIMEOUT;
}

uint32_t current_sense_dma_frame_count(void)
{
	return current_dma_frame_count;
}

uint32_t current_sense_dma_overflow_count(void)
{
	return current_dma_overflow_count;
}

esp_err_t current_sense_read(int *iu_raw, int *iv_raw)
{
	if (iu_raw == NULL || iv_raw == NULL)
	{
		ESP_LOGW(TAG, "current output pointer is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	current_sense_frame_t frame = {0};
	esp_err_t result = current_sense_read_latest_frame(&frame);
	if (result != ESP_OK)
	{
		return result;
	}

	*iu_raw = frame.iu_raw;
	*iv_raw = frame.iv_raw;
	return ESP_OK;
}

esp_err_t current_sense_calibrate(void)
{
	if (current_adc_handle == NULL)
	{
		ESP_LOGW(TAG, "ADC is not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	int64_t sum_u_voltage_mv = 0;
	int64_t sum_v_voltage_mv = 0;
	uint32_t last_sequence = 0U;
	int valid_samples = 0;

	for (int attempt = 0;
         attempt < CURRENT_SENSE_CALIBRATION_SAMPLES * 20;
		 attempt++)
	{
		vTaskDelay(pdMS_TO_TICKS(1));

		current_sense_frame_t frame = {0};
		esp_err_t result = current_sense_read_latest_frame(&frame);
		if (result != ESP_OK || frame.sequence == last_sequence)
		{
			continue;
		}
		last_sequence = frame.sequence;

		int u_voltage_mv = 0;
		int v_voltage_mv = 0;
		result = current_sense_raw_to_voltage(frame.iu_raw, &u_voltage_mv);
		if (result != ESP_OK)
		{
			return result;
		}
		result = current_sense_raw_to_voltage(frame.iv_raw, &v_voltage_mv);
		if (result != ESP_OK)
		{
			return result;
		}

		sum_u_voltage_mv += u_voltage_mv;
		sum_v_voltage_mv += v_voltage_mv;
		valid_samples++;

        if (valid_samples >= CURRENT_SENSE_CALIBRATION_SAMPLES)
		{
			break;
		}
	}

    if (valid_samples != CURRENT_SENSE_CALIBRATION_SAMPLES)
	{
		ESP_LOGE(TAG, "ADC DMA calibration timed out: samples=%d", valid_samples);
		return ESP_ERR_TIMEOUT;
	}

	current_u_zero_voltage_mv =
        (int)(sum_u_voltage_mv / CURRENT_SENSE_CALIBRATION_SAMPLES);
	current_v_zero_voltage_mv =
        (int)(sum_v_voltage_mv / CURRENT_SENSE_CALIBRATION_SAMPLES);
	current_sense_calibrated = true;

	ESP_LOGI(
		TAG,
		"calibration complete: U zero=%d mV, V zero=%d mV",
		current_u_zero_voltage_mv,
		current_v_zero_voltage_mv);

	return ESP_OK;
}

esp_err_t current_sense_read_amperes(float *iu_a, float *iv_a)
{
	if (iu_a == NULL || iv_a == NULL)
	{
		ESP_LOGW(TAG, "ampere output pointer is NULL");
		return ESP_ERR_INVALID_ARG;
	}
	if (!current_sense_calibrated)
	{
		ESP_LOGW(TAG, "current sense is not calibrated");
		return ESP_ERR_INVALID_STATE;
	}

	current_sense_frame_t frame = {0};
	esp_err_t result = current_sense_read_latest_frame(&frame);
	if (result != ESP_OK)
	{
		return result;
	}

	int u_voltage_mv = 0;
	int v_voltage_mv = 0;
	result = current_sense_raw_to_voltage(frame.iu_raw, &u_voltage_mv);
	if (result != ESP_OK)
	{
		return result;
	}
	result = current_sense_raw_to_voltage(frame.iv_raw, &v_voltage_mv);
	if (result != ESP_OK)
	{
		return result;
	}

	*iu_a = -(((float)u_voltage_mv - current_u_zero_voltage_mv) /
              CURRENT_SENSE_MILLIVOLTS_PER_AMP);
	*iv_a = -(((float)v_voltage_mv - current_v_zero_voltage_mv) /
              CURRENT_SENSE_MILLIVOLTS_PER_AMP);
	return ESP_OK;
}

esp_err_t current_sense_read_three_phase(
	float *iu_a,
	float *iv_a,
	float *iw_a)
{
	if (iu_a == NULL || iv_a == NULL || iw_a == NULL)
	{
		ESP_LOGW(TAG, "three-phase current output pointer is NULL");
		return ESP_ERR_INVALID_ARG;
	}
	if (current_adc_handle == NULL || !current_sense_calibrated)
	{
		ESP_LOGW(TAG, "current sense is not initialized or calibrated");
		return ESP_ERR_INVALID_STATE;
	}

	esp_err_t result = current_sense_read_amperes(iu_a, iv_a);
	if (result != ESP_OK)
	{
		*iu_a = 0.0f;
		*iv_a = 0.0f;
		*iw_a = 0.0f;
		return result;
	}

	*iw_a = -(*iu_a + *iv_a);
	return ESP_OK;
}
