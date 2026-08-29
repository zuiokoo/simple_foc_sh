#include "current_sense.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// 同步采样需要知道 PWM 低侧导通窗口，依赖 motor_pwm 提供的接口。
#include "motor/motor_pwm.h"
#include "motor/motor_config.h"
#include "soc/soc_caps.h"

#define CURRENT_SENSE_ADC_UNIT ADC_UNIT_1
#define CURRENT_SENSE_U_CHANNEL ADC_CHANNEL_0
#define CURRENT_SENSE_V_CHANNEL ADC_CHANNEL_3
#define CURRENT_SENSE_CALIBRATION_SAMPLES 100
#define CURRENT_ADC_SAMPLE_FREQ_HZ 200000U
#define CURRENT_ADC_FRAME_SIZE_BYTES 8U
#define CURRENT_ADC_MAX_READS_PER_CALL 1U
// 相位扫描最多记录的档位数，用于事后统计可用触发相位带。
#define CURRENT_SENSE_SWEEP_MAX_STEPS 128

// 5 mΩ 分流电阻，单位是 Ω。
#define CURRENT_SENSE_SHUNT_RESISTANCE_OHM 0.005f

// INA240A2PW 的固定增益是 50 V/V。
#define CURRENT_SENSE_AMPLIFIER_GAIN 50.0f

// 每 1 A 电流对应的 INA240 输出电压变化，单位是 mV/A：
// 1 A × 0.005 Ω × 50 = 0.25 V = 250 mV。
#define CURRENT_SENSE_MILLIVOLTS_PER_AMP  \
	(CURRENT_SENSE_SHUNT_RESISTANCE_OHM * \
	 CURRENT_SENSE_AMPLIFIER_GAIN * 1000.0f)

static const char *TAG = "CURRENT_SENSE";
static adc_continuous_handle_t current_adc_handle;
static uint8_t current_adc_result_buffer[1024];
static int current_latest_raw_u;
static int current_latest_raw_v;
static bool current_have_latest_u;
static bool current_have_latest_v;
static adc_cali_handle_t current_adc_cali_handle;
// 零电流时 INA240 OUT 的实际电压，单位是 mV。
// 不直接写死 1650 mV，因为实际电路会有放大器偏置和 ADC 误差。
static int current_u_zero_voltage_mv;
static int current_v_zero_voltage_mv;
static int current_sense_calibrated;
#if M1_ENABLE_TIMING_LOG
static uint32_t current_sense_timing_log_count;
#endif

/**
 * @brief 使用 ESP-IDF ADC 校准曲线，把原始 ADC 计数转换为 mV。
 */
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

	return adc_cali_raw_to_voltage(
		current_adc_cali_handle,
		raw,
		voltage_mv);
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
	current_sense_calibrated = 0;
	current_adc_cali_handle = NULL;
	current_have_latest_u = false;
	current_have_latest_v = false;

	adc_continuous_handle_cfg_t handle_config = {
		.max_store_buf_size = 2048,
		.conv_frame_size = CURRENT_ADC_FRAME_SIZE_BYTES,
	};

	esp_err_t result = adc_continuous_new_handle(&handle_config, &current_adc_handle);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "failed to create continuous ADC handle: %s", esp_err_to_name(result));
		current_adc_handle = NULL;
		return result;
	}

	adc_digi_pattern_config_t pattern[2] = {0};
	pattern[0].atten = ADC_ATTEN_DB_12;
	pattern[0].channel = CURRENT_SENSE_U_CHANNEL & 0x7;
	pattern[0].unit = CURRENT_SENSE_ADC_UNIT;
	pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
	pattern[1].atten = ADC_ATTEN_DB_12;
	pattern[1].channel = CURRENT_SENSE_V_CHANNEL & 0x7;
	pattern[1].unit = CURRENT_SENSE_ADC_UNIT;
	pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

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
		ESP_LOGE(TAG, "failed to configure continuous ADC: %s", esp_err_to_name(result));
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
		ESP_LOGE(TAG, "failed to create ADC calibration: %s", esp_err_to_name(result));
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		current_adc_cali_handle = NULL;
		return result;
	}

	result = adc_continuous_start(current_adc_handle);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "failed to start continuous ADC: %s", esp_err_to_name(result));
		adc_cali_delete_scheme_line_fitting(current_adc_cali_handle);
		current_adc_cali_handle = NULL;
		adc_continuous_deinit(current_adc_handle);
		current_adc_handle = NULL;
		return result;
	}

	ESP_LOGI(
		TAG,
		"continuous ADC started: U=ADC1_CH0(GPIO36), V=ADC1_CH3(GPIO39), sample=%u Hz",
		CURRENT_ADC_SAMPLE_FREQ_HZ);

	return ESP_OK;
}

static esp_err_t current_sense_read_latest_raw(int *iu_raw, int *iv_raw)
{
	if (iu_raw == NULL || iv_raw == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (current_adc_handle == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}

	/* Bound continuous DMA draining so it cannot starve the watchdog. */
	for (uint32_t read_count = 0;
		 read_count < CURRENT_ADC_MAX_READS_PER_CALL;
		 read_count++)
	{
		uint32_t bytes_read = 0;
		esp_err_t result = adc_continuous_read(
			current_adc_handle,
			current_adc_result_buffer,
			sizeof(current_adc_result_buffer),
			&bytes_read,
			0);
		if (result == ESP_ERR_TIMEOUT)
		{
			break;
		}
		if (result != ESP_OK)
		{
			return result;
		}

		for (uint32_t offset = 0;
			 offset + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read;
			 offset += SOC_ADC_DIGI_RESULT_BYTES)
		{
			adc_digi_output_data_t *sample =
				(adc_digi_output_data_t *)&current_adc_result_buffer[offset];
			uint32_t channel = sample->type1.channel;
			uint32_t raw = sample->type1.data;
			if (channel == CURRENT_SENSE_U_CHANNEL)
			{
				current_latest_raw_u = (int)raw;
				current_have_latest_u = true;
			}
			else if (channel == CURRENT_SENSE_V_CHANNEL)
			{
				current_latest_raw_v = (int)raw;
				current_have_latest_v = true;
			}
		}
	}

	if (!current_have_latest_u || !current_have_latest_v)
	{
		return ESP_ERR_TIMEOUT;
	}

	*iu_raw = current_latest_raw_u;
	*iv_raw = current_latest_raw_v;
	return ESP_OK;
}
/*
 * Discard samples accumulated before the PWM low-side window.  Continuous
 * ADC is DMA-backed but not PWM-triggered, so old samples must not be reused
 * as the current-loop feedback value.
 */
static void current_sense_discard_dma_samples(void)
{
	if (current_adc_handle == NULL)
	{
		return;
	}

	/* Bound continuous DMA draining so it cannot starve the watchdog. */
	for (uint32_t read_count = 0;
		 read_count < CURRENT_ADC_MAX_READS_PER_CALL;
		 read_count++)
	{
		uint32_t bytes_read = 0;
		esp_err_t result = adc_continuous_read(
			current_adc_handle,
			current_adc_result_buffer,
			sizeof(current_adc_result_buffer),
			&bytes_read,
			0);
		if (result == ESP_ERR_TIMEOUT)
		{
			break;
		}
		if (result != ESP_OK)
		{
			break;
		}
	}

	/*
	 * The discarded samples may have refreshed the cached raw values.
	 * Require both channels to arrive again after the PWM window.
	 */
	current_have_latest_u = false;
	current_have_latest_v = false;
}

esp_err_t current_sense_read(int *iu_raw, int *iv_raw)
{
	if (iu_raw == NULL || iv_raw == NULL)
	{
		ESP_LOGW(TAG, "current output pointer is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	return current_sense_read_latest_raw(iu_raw, iv_raw);
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

	for (int sample = 0;
		 sample < CURRENT_SENSE_CALIBRATION_SAMPLES;
		 sample++)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
        int iu_raw = 0;
		int iv_raw = 0;

		esp_err_t result =
			current_sense_read(&iu_raw, &iv_raw);

		if (result != ESP_OK)
		{
			return result;
		}

		int u_voltage_mv = 0;
		int v_voltage_mv = 0;

		result = current_sense_raw_to_voltage(iu_raw, &u_voltage_mv);
		if (result != ESP_OK)
		{
			return result;
		}

		result = current_sense_raw_to_voltage(iv_raw, &v_voltage_mv);
		if (result != ESP_OK)
		{
			return result;
		}

		sum_u_voltage_mv += u_voltage_mv;
		sum_v_voltage_mv += v_voltage_mv;
	}

	current_u_zero_voltage_mv =
		(int)(sum_u_voltage_mv / CURRENT_SENSE_CALIBRATION_SAMPLES);
	current_v_zero_voltage_mv =
		(int)(sum_v_voltage_mv / CURRENT_SENSE_CALIBRATION_SAMPLES);
	current_sense_calibrated = 1;

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

	if (current_sense_calibrated == 0)
	{
		ESP_LOGW(TAG, "current sense is not calibrated");
		return ESP_ERR_INVALID_STATE;
	}

	int iu_raw = 0;
	int iv_raw = 0;
	esp_err_t result = current_sense_read(&iu_raw, &iv_raw);
	if (result != ESP_OK)
	{
		return result;
	}

	int u_voltage_mv = 0;
	int v_voltage_mv = 0;

	result = current_sense_raw_to_voltage(iu_raw, &u_voltage_mv);
	if (result != ESP_OK)
	{
		return result;
	}

	result = current_sense_raw_to_voltage(iv_raw, &v_voltage_mv);
	if (result != ESP_OK)
	{
		return result;
	}

	/*
	 * INA240 输出关系：
	 * Vout = 零电流输出电压 + 电流 × 250 mV/A。
	 * 因此：
	 * 电流 = (当前输出电压 - 零电流输出电压) / 250 mV/A。
	 */
	*iu_a = ((float)u_voltage_mv - current_u_zero_voltage_mv) /
			CURRENT_SENSE_MILLIVOLTS_PER_AMP;
	*iu_a = -(*iu_a); // U 相电流传感器方向与 V 相相反，统一为电机相电流正方向
	*iv_a = ((float)v_voltage_mv - current_v_zero_voltage_mv) /
			CURRENT_SENSE_MILLIVOLTS_PER_AMP;
	*iv_a = -(*iv_a); // 在计算完 *iv_a 后立即取反
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

	if (current_adc_handle == NULL || current_sense_calibrated == 0)
	{
		ESP_LOGW(TAG, "current sense is not initialized or calibrated");
		return ESP_ERR_INVALID_STATE;
	}

	/*
	 * A single low-side sample is strongly affected by PWM ripple. Take
	 * four fresh samples at consecutive PWM windows and average them.
	 * At the present 1 kHz control rate this spans only about 0.2 ms.
	 */
	float sum_iu_a = 0.0f;
	float sum_iv_a = 0.0f;
	float sum_iw_a = 0.0f;
	int valid_samples = 0;

	for (int sample = 0; sample < 4; sample++)
	{
		current_sense_discard_dma_samples();

		esp_err_t result = motor_pwm_wait_lowside_window(1500);
		if (result != ESP_OK)
		{
			continue;
		}

		int iu_raw = 0;
		int iv_raw = 0;
		int64_t sample_deadline_us = esp_timer_get_time() + 50;
		do
		{
			result = current_sense_read(&iu_raw, &iv_raw);
			if (result == ESP_OK)
			{
				break;
			}
			esp_rom_delay_us(1);
		} while (esp_timer_get_time() < sample_deadline_us);

		if (result != ESP_OK)
		{
			continue;
		}

		int u_voltage_mv = 0;
		int v_voltage_mv = 0;
		result = current_sense_raw_to_voltage(iu_raw, &u_voltage_mv);
		if (result != ESP_OK)
		{
			continue;
		}
		result = current_sense_raw_to_voltage(iv_raw, &v_voltage_mv);
		if (result != ESP_OK)
		{
			continue;
		}

		float sample_iu_a =
			-(((float)u_voltage_mv - current_u_zero_voltage_mv) /
			  CURRENT_SENSE_MILLIVOLTS_PER_AMP);
		float sample_iv_a =
			-(((float)v_voltage_mv - current_v_zero_voltage_mv) /
			  CURRENT_SENSE_MILLIVOLTS_PER_AMP);
		float sample_iw_a = -(sample_iu_a + sample_iv_a);

		sum_iu_a += sample_iu_a;
		sum_iv_a += sample_iv_a;
		sum_iw_a += sample_iw_a;
		valid_samples++;
	}

	if (valid_samples == 0)
	{
		*iu_a = 0.0f;
		*iv_a = 0.0f;
		*iw_a = 0.0f;
		return ESP_ERR_TIMEOUT;
	}

	*iu_a = sum_iu_a / (float)valid_samples;
	*iv_a = sum_iv_a / (float)valid_samples;
	*iw_a = sum_iw_a / (float)valid_samples;
	return ESP_OK;
}

/*
 * 诊断用：扫描"启动 ADC 的 PWM 相位"，定位采样瞬间真正落在低侧导通窗口的
 * 启动相位。
 *
 * 背景：adc_oneshot_read 从调用到真正采样存在固定延迟 T。20kHz 下一个 PWM
 * 周期只有 50us，低侧窗口更只有 14~25us。若 T 远大于窗口宽度，那么"先等窗口
 * 再开始读"是无效的——采样瞬间早已飞出窗口。正确做法是提前 T 启动读取。
 *
 * 本函数在每个相位档位启动一次 ADC，并在读回后立刻记录计数器值（近似采样
 * 瞬间的相位），从而直接读出 T 与各相位的命中情况，同时给出抖动范围。
 */
void current_sense_phase_sweep(int steps, int repeat)
{
	if (current_adc_handle == NULL || current_sense_calibrated == 0)
	{
		ESP_LOGW(TAG, "phase sweep: sensor is not ready");
		return;
	}
	if (steps <= 0 || repeat <= 0)
	{
		return;
	}
	if (steps > CURRENT_SENSE_SWEEP_MAX_STEPS)
	{
		steps = CURRENT_SENSE_SWEEP_MAX_STEPS;
	}

	static uint32_t s_target[CURRENT_SENSE_SWEEP_MAX_STEPS];
	static float s_ivmin[CURRENT_SENSE_SWEEP_MAX_STEPS];
	static int s_valid[CURRENT_SENSE_SWEEP_MAX_STEPS];

	/* 先量单次 adc_oneshot_read 的纯耗时：连续读 20 次取平均 */
	{
		int64_t p0 = esp_timer_get_time();
		int dummy = 0;
		for (int k = 0; k < 20; k++)
		{
			current_sense_read(&dummy, &dummy);
		}
		int64_t p1 = esp_timer_get_time();
		ESP_LOGI("ADCSWEEP",
				 "adc_oneshot_read x20 = %lld us, each ~ %.1f us",
				 (long long)(p1 - p0), (double)(p1 - p0) / 20.0);
	}

	// 三角波一个完整周期 = 2 * 峰值，tick = 0.1us
	const uint32_t period_ticks = 2u * (uint32_t)M1_PWM_COMPARE_MAX_TICKS;
	const uint32_t step_ticks = period_ticks / (uint32_t)steps;

	ESP_LOGI("ADCSWEEP",
			 "=== phase sweep: %d steps x %lu ticks over %lu ticks (%.1f us per PWM period) ===",
			 steps, (unsigned long)step_ticks,
			 (unsigned long)period_ticks, (double)period_ticks * 0.1);

	float best_min = 0.0f;

	for (int s = 0; s < steps; s++)
	{
		uint32_t target = (uint32_t)(((uint64_t)s * period_ticks) / (uint32_t)steps);

		float iv_sum = 0.0f;
		float iv_min = 1.0e9f;
		float iv_max = -1.0e9f;
		uint32_t cnt_sum = 0;
		int valid = 0;

		for (int k = 0; k < repeat; k++)
		{
			motor_pwm_wait_count_rising(target, 200);

			int raw_u = 0;
			int raw_v = 0;
			esp_err_t r = current_sense_read(&raw_u, &raw_v);
			// 读完立刻取计数器，用于反推采样延迟
			uint32_t cnt_v = motor_pwm_debug_count();

			int mv = 0;
			if (r != ESP_OK ||
				current_sense_raw_to_voltage(raw_v, &mv) != ESP_OK)
			{
				continue;
			}

			float iv = ((float)mv - current_v_zero_voltage_mv) /
					   CURRENT_SENSE_MILLIVOLTS_PER_AMP;
			iv = -iv; // 与正式代码一致：V 相取反

			iv_sum += iv;
			if (iv < iv_min)
			{
				iv_min = iv;
			}
			if (iv > iv_max)
			{
				iv_max = iv;
			}
			cnt_sum += cnt_v;
			valid++;

			vTaskDelay(pdMS_TO_TICKS(2));
		}

		s_target[s] = target;
		s_valid[s] = 0;

		if (valid == 0)
		{
			ESP_LOGI("ADCSWEEP", "start=%3lu tick | no valid sample",
					 (unsigned long)target);
			continue;
		}

		ESP_LOGI("ADCSWEEP",
				 "start=%3lu tick | cnt@V=%3lu | iv avg=%+.3f min=%+.3f max=%+.3f",
				 (unsigned long)target,
				 (unsigned long)(cnt_sum / (uint32_t)valid),
				 iv_sum / (float)valid, iv_min, iv_max);

		s_ivmin[s] = iv_min;
		s_valid[s] = 1;
		if (iv_min > best_min)
		{
			best_min = iv_min;
		}
	}

	/*
	 * 统计可用触发相位带：取"最坏一次采样仍然命中"的最长连续档位段。
	 * 判据用 min 而不是 avg —— avg 高可能是 1/4 命中拉出来的，不可靠。
	 */
	if (best_min < 0.05f)
	{
		ESP_LOGW("ADCSWEEP",
				 "=== RESULT: no hit at all (best min = %+.3f) ===", best_min);
		return;
	}

	const float threshold = best_min * 0.8f;
	int best_start = -1;
	int best_len = 0;
	int run_start = -1;
	int run_len = 0;

	for (int s = 0; s < steps; s++)
	{
		int ok = (s_valid[s] != 0) && (s_ivmin[s] >= threshold);
		if (ok)
		{
			if (run_start < 0)
			{
				run_start = s;
				run_len = 0;
			}
			run_len++;
			if (run_len > best_len)
			{
				best_len = run_len;
				best_start = run_start;
			}
		}
		else
		{
			run_start = -1;
			run_len = 0;
		}
	}

	if (best_len <= 0)
	{
		ESP_LOGW("ADCSWEEP", "=== RESULT: no stable band (best min = %+.3f) ===",
				 best_min);
		return;
	}

	uint32_t t_lo = s_target[best_start];
	uint32_t t_hi = s_target[best_start + best_len - 1];
	uint32_t width = (t_hi - t_lo) + step_ticks;
	uint32_t center = (t_lo + t_hi) / 2u;

	ESP_LOGI("ADCSWEEP",
			 "=== RESULT: usable band target %lu..%lu, center=%lu tick, width=%lu tick (%.1f us) ===",
			 (unsigned long)t_lo, (unsigned long)t_hi, (unsigned long)center,
			 (unsigned long)width, (double)width * 0.1);
	ESP_LOGI("ADCSWEEP",
			 "=== RESULT: recommended M1_CURRENT_SAMPLE_TARGET_TICKS = %lu ===",
			 (unsigned long)center);
}
