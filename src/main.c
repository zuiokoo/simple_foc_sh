#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "control/foc_speed_pi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor/motor_config.h"
#include "motor/motor_pwm.h"
#include "sensor/as5600.h"
#include "sensor/current_sense.h"
#include "control/foc_math.h"
#include "control/foc_svpwm.h"
#include "control/foc_controller.h"
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#if CONFIG_FREERTOS_HZ < 1000
#error "FOC current loop requires CONFIG_FREERTOS_HZ >= 1000"
#endif

#define LED1_GPIO 12
#define LED2_GPIO 13
#define WS2812_GPIO 2
#define WS2812_COUNT 1
#define LED1_ON gpio_set_level(LED1_GPIO, 1);
#define LED1_OFF gpio_set_level(LED1_GPIO, 0);
#define LED2_ON gpio_set_level(LED2_GPIO, 1);
#define LED2_OFF gpio_set_level(LED2_GPIO, 0);
#define WS2812_SPI_HZ 2400000
#define WS2812_BYTES_PER_LED 9
#define WS2812_RESET_BYTES 120
#define WS2812_DATA_BYTES (WS2812_COUNT * WS2812_BYTES_PER_LED)
#define WS2812_TX_BYTES (WS2812_DATA_BYTES + WS2812_RESET_BYTES)

static const char *TAG = "SPI_WS2812";
static spi_device_handle_t ws2812_spi;
static uint8_t *ws2812_tx_buffer;
static foc_controller_t foc_controller;
static foc_speed_pi_t foc_speed_controller;
static float foc_electrical_zero_offset_rad = M1_ELECTRICAL_ZERO_OFFSET_RAD;
typedef struct
{
	float dt_s;
	float mechanical_speed_rad_s;
	float iq_ref_a;
	float iu_a;
	float iv_a;
	float iw_a;
	float id_a;
	float iq_a;
	float vd_v;
	float vq_v;
	float vd_decoupling_v;
	float vq_decoupling_v;
	float voltage_vector_limit_v;
	float duty_u;
	float duty_v;
	float duty_w;
	uint32_t missed_ticks;
	uint32_t overrun_count;
	uint32_t max_loop_us;
	uint32_t angle_valid;
	uint32_t angle_error_count;
    uint32_t angle_age_us;
    uint32_t current_age_us;
    uint32_t current_sequence;
    int32_t angle_to_current_sample_us;
    int32_t electrical_angle_mrad;
    uint32_t actual_period_us;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t period_jitter_us;
    float angle_velocity_mrad_s;
    float id_pi_integral_v;
    float iq_pi_integral_v;
} foc_current_snapshot_t;

static volatile foc_current_snapshot_t foc_current_snapshot;

// Current-loop timing: four 20 kHz PWM periods = 5 kHz / 200 us.
#define FOC_CURRENT_TS_S ((float)M1_CURRENT_LOOP_PWM_PERIODS / (float)M1_PWM_FREQUENCY_HZ)

// 当前 d 轴和 q 轴目标电流都为0A，用于零电流数学测试。
#define FOC_TEST_ID_REF_A M1_CURRENT_LOOP_TEST_ID_REF_A
#define FOC_TEST_IQ_REF_A M1_CURRENT_LOOP_TEST_IQ_REF_A

// 临时测试参数，不是最终电机参数。
#define FOC_TEST_ID_PI_KP M1_CURRENT_LOOP_ID_PI_KP
#define FOC_TEST_ID_PI_KI M1_CURRENT_LOOP_ID_PI_KI
#define FOC_TEST_IQ_PI_KP M1_CURRENT_LOOP_IQ_PI_KP
#define FOC_TEST_IQ_PI_KI M1_CURRENT_LOOP_IQ_PI_KI
#define FOC_TEST_PI_OUTPUT_MIN_V (-6.0f)
#define FOC_TEST_PI_OUTPUT_MAX_V (6.0f)
#define FOC_TEST_CURRENT_LIMIT_A 2.0f

// 实际直流母线电压，供SVPWM电压换算使用。
#define FOC_TEST_BUS_VOLTAGE_V 12.0f
static void ws2812_encode_byte(uint8_t value, uint8_t output[3])
{
	uint32_t encode = 0;
	for (int bit = 7; bit >= 0; bit--)
	{
		encode <<= 3;
		if (value & (1u << bit))
		{
			encode |= 0x06; // 二进制 110
		}
		else
		{
			encode |= 0x04; // 二进制 100
		}
	}
	output[0] = (encode >> 16) & 0xff;
	output[1] = (encode >> 8) & 0xff;
	output[2] = encode & 0xff;
}
static esp_err_t ws2812_set_pixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	if (ws2812_tx_buffer == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}
	if (index >= WS2812_COUNT)
	{
		return ESP_ERR_INVALID_ARG;
	}
	size_t offset = index * WS2812_BYTES_PER_LED;
	ws2812_encode_byte(green, &ws2812_tx_buffer[offset]);
	ws2812_encode_byte(red, &ws2812_tx_buffer[offset + 3]);
	ws2812_encode_byte(blue, &ws2812_tx_buffer[offset + 6]);
	return ESP_OK;
}
static esp_err_t ws2812_show(void)
{
	if (ws2812_spi == NULL || ws2812_tx_buffer == NULL)
	{
		return ESP_ERR_INVALID_STATE;
	}
	memset(&ws2812_tx_buffer[WS2812_DATA_BYTES], 0, WS2812_RESET_BYTES);
	spi_transaction_t trransaction = {
		.length = WS2812_TX_BYTES * 8,
		.tx_buffer = ws2812_tx_buffer,
	};
	return spi_device_transmit(ws2812_spi, &trransaction);
}
static esp_err_t ws2812_clear(void)
{
	for (uint16_t index = 0;
		 index < WS2812_COUNT; index++)
	{
		esp_err_t result = ws2812_set_pixel(index, 0, 0, 0);
		if (result != ESP_OK)
		{
			return result;
		}
	}
	return ws2812_show();
}
void ws2812_init(void)
{
	spi_bus_config_t buscfg = {
		.miso_io_num = -1,

		.mosi_io_num = WS2812_GPIO,
		.sclk_io_num = -1,

		.quadwp_io_num = -1,
		.quadhd_io_num = -1,

		.max_transfer_sz = WS2812_TX_BYTES,
	};
	spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = WS2812_SPI_HZ,

		.mode = 0,

		.spics_io_num = -1,

		.queue_size = 1,

		.pre_cb = NULL,

		.post_cb = NULL,
	};
	spi_bus_add_device(HSPI_HOST, &devcfg, &ws2812_spi);
	/*
	 * DMA 缓冲区必须放在 DMA 可访问的内存中。
	 */
	ws2812_tx_buffer = heap_caps_calloc(
		1,
		WS2812_TX_BYTES,
		MALLOC_CAP_DMA);
	if (ws2812_tx_buffer == NULL)
	{
		ESP_LOGE(TAG, "WS2812 DMA buffer allocate failed");
		abort();
	}

	ESP_LOGI(
		TAG,
		"SPI WS2812 init, GPIO=%d, LEDs=%d, TX bytes=%d",
		WS2812_GPIO,
		WS2812_COUNT,
		WS2812_TX_BYTES);
}

void led_init(void)
{
	// 配置 GPIO12 和 GPIO13 为输出模式
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;							  // 禁用中断
	io_conf.mode = GPIO_MODE_OUTPUT;								  // 设置为输出模式
	io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO); // 设置引脚掩码
	io_conf.pull_down_en = 0;										  // 禁用下拉
	io_conf.pull_up_en = 0;											  // 禁用上拉
	gpio_config(&io_conf);
	// 初始化 LED 状态
	gpio_set_level(LED1_GPIO, 0); // LED1 初始为关闭
	gpio_set_level(LED2_GPIO, 0); // LED2 初始为关闭
}
static void led_task(void *pvParameter)
{
	while (1)
	{
		LED1_ON;
		LED2_ON;
		// ESP_LOGI("LED_TASK", "LED1 and LED2 are ON");
		vTaskDelay(pdMS_TO_TICKS(500));
		LED1_OFF;
		LED2_OFF;
		// ESP_LOGI("LED_TASK", "LED1 and LED2 are OFF");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
static void ws2812_task(void *pvParameter)
{
	while (1)
	{
		ws2812_set_pixel(0, 100, 0, 0);
		ws2812_show();
		ESP_LOGI(TAG, "WS2812: red");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ws2812_set_pixel(0, 0, 100, 0);
		ws2812_show();
		ESP_LOGI(TAG, "WS2812: green");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ws2812_set_pixel(0, 0, 0, 100);
		ws2812_show();
		ESP_LOGI(TAG, "WS2812: blue");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ws2812_set_pixel(0, 100, 100, 100);
		ws2812_show();
		ESP_LOGI(TAG, "WS2812: white");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ws2812_clear();
		ESP_LOGI(TAG, "WS2812: off");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
 * @brief 使用固定的低电压矢量，让转子对齐并测量电角度零点。
 *
 * 该函数默认不执行，只有把配置宏改为 1 后才会在启动时运行一次。
 */
static void electrical_zero_calibration(void)
{
	ESP_LOGW("FOC_CALIB", "Electrical zero calibration is starting");
	ESP_LOGI("CALIB_CFG", "align duty: U=%.3f V=%.3f W=%.3f",
			 M1_ALIGNMENT_DUTY_U, M1_ALIGNMENT_DUTY_V, M1_ALIGNMENT_DUTY_W);

	// U 相略高、V/W 相略低，形成固定在电角度 0 附近的电压矢量。
	esp_err_t result = motor_pwm_set_duty(0.5f, 0.5f, 0.5f);
	if (result != ESP_OK)
	{
		ESP_LOGE("FOC_CALIB", "Failed to set alignment duty: %s", esp_err_to_name(result));
		return;
	}

	// 占空比已经设置好后，才启动 PWM 输出。
	result = motor_pwm_start();
	if (result != ESP_OK)
	{
		ESP_LOGE("FOC_CALIB", "Failed to start PWM for alignment: %s", esp_err_to_name(result));
		return;
	}

    // 从零电压平滑爬升到校准矢量，避免转子被瞬间吸动。
    const TickType_t alignment_ramp_period = pdMS_TO_TICKS(10);
    const int alignment_ramp_steps = 30;
    for (int alignment_step = 1;
         alignment_step <= alignment_ramp_steps;
         alignment_step++)
    {
        float alignment_fraction =
            (float)alignment_step / (float)alignment_ramp_steps;
        result = motor_pwm_set_duty(
            0.5f + (M1_ALIGNMENT_DUTY_U - 0.5f) * alignment_fraction,
            0.5f + (M1_ALIGNMENT_DUTY_V - 0.5f) * alignment_fraction,
            0.5f + (M1_ALIGNMENT_DUTY_W - 0.5f) * alignment_fraction);
        if (result != ESP_OK)
        {
            ESP_LOGE(
                "FOC_CALIB",
                "Failed to ramp alignment duty: %s",
                esp_err_to_name(result));
            break;
        }
        vTaskDelay(alignment_ramp_period);
    }

	// 保持固定磁场，同时监测电流，防止异常接线时持续通电。
	const TickType_t sample_period = pdMS_TO_TICKS(20);
	const int sample_count = (int)(M1_ALIGNMENT_TIME_MS / 20U);
	float alignment_i_alpha_sum = 0.0f;
	float alignment_i_beta_sum = 0.0f;
	int alignment_current_samples = 0;
	for (int sample = 0; sample < sample_count; sample++)
	{
		float iu_a = 0.0f;
		float iv_a = 0.0f;
		float iw_a = 0.0f;
		result = current_sense_read_three_phase(&iu_a, &iv_a, &iw_a);
		if (result != ESP_OK)
		{
			ESP_LOGE("FOC_CALIB", "Current read failed during alignment: %s", esp_err_to_name(result));
			break;
		}
		if (sample >= sample_count / 2)
		{
			alignment_i_alpha_sum += iu_a;
			alignment_i_beta_sum += (iu_a + 2.0f * iv_a) / sqrtf(3.0f);
			alignment_current_samples++;
		}
		if ((sample % 5) == 0) // 每100ms打一次
		{
			ESP_LOGI("FOC_CALIB", "align: iu=%.3f, iv=%.3f, iw=%.3f",
					 iu_a, iv_a, iw_a);
		}
		if (fabsf(iu_a) > M1_ALIGNMENT_MAX_CURRENT_A ||
			fabsf(iv_a) > M1_ALIGNMENT_MAX_CURRENT_A ||
			fabsf(iw_a) > M1_ALIGNMENT_MAX_CURRENT_A)
		{
			ESP_LOGE(
				"FOC_CALIB",
				"Alignment current is too high: iu=%.3f, iv=%.3f, iw=%.3f A",
				iu_a,
				iv_a,
				iw_a);
			result = ESP_ERR_INVALID_STATE;
			break;
		}

		vTaskDelay(sample_period);
	}

	if (result == ESP_OK)
	{
		float mechanical_angle = 0.0f;
		float velocity_rad_s = 0.0f;
		result = as5600_measure_angle_velocity(&mechanical_angle, &velocity_rad_s);
		if (result == ESP_OK)
		{
			// 固定矢量目标电角度为 0，因此偏移量就是 mech_angle × pole_pairs。
			float mechanical_electrical_angle = foc_mechanical_to_electrical_angle(
                mechanical_angle,
                M1_MOTOR_POLE_PAIRS,
                0.0f);
            float electrical_zero_offset = mechanical_electrical_angle;
            bool alignment_valid = false;
            if (alignment_current_samples > 0)
            {
                float alignment_i_alpha = alignment_i_alpha_sum / (float)alignment_current_samples;
                float alignment_i_beta = alignment_i_beta_sum / (float)alignment_current_samples;
                float alignment_current_magnitude = sqrtf(
                    alignment_i_alpha * alignment_i_alpha +
                    alignment_i_beta * alignment_i_beta);
                if (alignment_current_magnitude < M1_ALIGNMENT_MIN_CURRENT_A)
                {
                    ESP_LOGE(
                        "FOC_CALIB",
                        "Alignment current too low: magnitude=%.3f A, keeping configured offset=%.6f rad",
                        alignment_current_magnitude,
                        foc_electrical_zero_offset_rad);
                }
                else
                {
                    ESP_LOGI(
                        "FOC_CALIB",
                        "Alignment current valid: magnitude=%.3f A; using commanded U-axis vector for zero",
                        alignment_current_magnitude);
                    alignment_valid = true;
                }
            }
            if (alignment_valid)
            {
                foc_electrical_zero_offset_rad = electrical_zero_offset;
            }
			ESP_LOGI(
				"FOC_CALIB",
				"mechanical angle: %.6f rad, electrical zero offset: %.6f rad",
				mechanical_angle,
				electrical_zero_offset);
		}
		else
		{
			ESP_LOGE("FOC_CALIB", "Failed to read aligned angle: %s", esp_err_to_name(result));
		}
	}

	// 无论测量是否成功，只要启动过 PWM，最后都必须安全停止。
	esp_err_t stop_result = motor_pwm_stop();
	if (stop_result != ESP_OK)
	{
		ESP_LOGE("FOC_CALIB", "Failed to stop PWM after alignment: %s", esp_err_to_name(stop_result));
	}
}
/**
 * @brief 根据开环电角度计算三相PWM占空比。
 *
 * 只负责数学计算，不启动PWM。
 */
static void foc_current_telemetry_task(void *pvParameter)
{
	(void)pvParameter;
	while (1)
	{
		foc_current_snapshot_t snapshot = {0};
		snapshot.dt_s = foc_current_snapshot.dt_s;
		snapshot.mechanical_speed_rad_s = foc_current_snapshot.mechanical_speed_rad_s;
		snapshot.iq_ref_a = foc_current_snapshot.iq_ref_a;
		snapshot.iu_a = foc_current_snapshot.iu_a;
		snapshot.iv_a = foc_current_snapshot.iv_a;
		snapshot.iw_a = foc_current_snapshot.iw_a;
		snapshot.id_a = foc_current_snapshot.id_a;
		snapshot.iq_a = foc_current_snapshot.iq_a;
		snapshot.vd_v = foc_current_snapshot.vd_v;
		snapshot.vq_v = foc_current_snapshot.vq_v;
		snapshot.vd_decoupling_v = foc_current_snapshot.vd_decoupling_v;
		snapshot.vq_decoupling_v = foc_current_snapshot.vq_decoupling_v;
		snapshot.voltage_vector_limit_v = foc_current_snapshot.voltage_vector_limit_v;
		snapshot.duty_u = foc_current_snapshot.duty_u;
		snapshot.duty_v = foc_current_snapshot.duty_v;
		snapshot.duty_w = foc_current_snapshot.duty_w;
	snapshot.missed_ticks = foc_current_snapshot.missed_ticks;
	snapshot.overrun_count = foc_current_snapshot.overrun_count;
	snapshot.max_loop_us = foc_current_snapshot.max_loop_us;
	snapshot.angle_valid = foc_current_snapshot.angle_valid;
	snapshot.angle_error_count = foc_current_snapshot.angle_error_count;
        snapshot.angle_age_us = foc_current_snapshot.angle_age_us;
        snapshot.current_age_us = foc_current_snapshot.current_age_us;
        snapshot.current_sequence = foc_current_snapshot.current_sequence;
        snapshot.angle_to_current_sample_us = foc_current_snapshot.angle_to_current_sample_us;
        snapshot.electrical_angle_mrad = foc_current_snapshot.electrical_angle_mrad;
        snapshot.actual_period_us = foc_current_snapshot.actual_period_us;
        snapshot.period_min_us = foc_current_snapshot.period_min_us;
        snapshot.period_max_us = foc_current_snapshot.period_max_us;
        snapshot.period_jitter_us = foc_current_snapshot.period_jitter_us;
        snapshot.angle_velocity_mrad_s = foc_current_snapshot.angle_velocity_mrad_s;
        snapshot.id_pi_integral_v = foc_current_snapshot.id_pi_integral_v;
        snapshot.iq_pi_integral_v = foc_current_snapshot.iq_pi_integral_v;

		/*
		 * Do not use floating-point printf here.  Newlib's float formatter
		 * consumes a large amount of task stack and previously overflowed
		 * this low-priority telemetry task immediately after startup.
		 * Values are scaled for readability while keeping the current-loop
		 * task completely free of logging.
		 */
		ESP_LOGI(
			"FOC_TELEM",
			"dt_us=%ld period_us=%lu period_min_us=%lu period_max_us=%lu period_jitter_us=%lu speed_mrad_s=%ld angle_velocity_mrad_s=%ld angle_age_us=%lu current_age_us=%lu current_sequence=%lu angle_to_current_sample_us=%ld electrical_angle_mrad=%ld iq_ref_mA=%ld iu_mA=%ld iv_mA=%ld iw_mA=%ld id_mA=%ld iq_mA=%ld vd_mV=%ld vq_mV=%ld vd_dec_mV=%ld vq_dec_mV=%ld v_limit_mV=%ld id_pi_int_mV=%ld iq_pi_int_mV=%ld duty=%ld/%ld/%ld missed=%lu overrun=%lu max_loop_us=%lu angle_valid=%lu angle_err=%lu",
			(long)(snapshot.dt_s * 1000000.0f),
            (unsigned long)snapshot.actual_period_us,
            (unsigned long)snapshot.period_min_us,
            (unsigned long)snapshot.period_max_us,
            (unsigned long)snapshot.period_jitter_us,
			(long)(snapshot.mechanical_speed_rad_s * 1000.0f),
            (long)(snapshot.angle_velocity_mrad_s),
            (unsigned long)snapshot.angle_age_us,
            (unsigned long)snapshot.current_age_us,
            (unsigned long)snapshot.current_sequence,
            (long)snapshot.angle_to_current_sample_us,
            (long)snapshot.electrical_angle_mrad,
			(long)(snapshot.iq_ref_a * 1000.0f),
			(long)(snapshot.iu_a * 1000.0f),
			(long)(snapshot.iv_a * 1000.0f),
			(long)(snapshot.iw_a * 1000.0f),
			(long)(snapshot.id_a * 1000.0f),
			(long)(snapshot.iq_a * 1000.0f),
						(long)(snapshot.vd_v * 1000.0f),
			(long)(snapshot.vq_v * 1000.0f),
			(long)(snapshot.vd_decoupling_v * 1000.0f),
			(long)(snapshot.vq_decoupling_v * 1000.0f),
			(long)(snapshot.voltage_vector_limit_v * 1000.0f),
            (long)(snapshot.id_pi_integral_v * 1000.0f),
            (long)(snapshot.iq_pi_integral_v * 1000.0f),
			(long)(snapshot.duty_u * 1000.0f),
			(long)(snapshot.duty_v * 1000.0f),
			(long)(snapshot.duty_w * 1000.0f),
			(unsigned long)snapshot.missed_ticks,
			(unsigned long)snapshot.overrun_count,
			(unsigned long)snapshot.max_loop_us,
		(unsigned long)snapshot.angle_valid,
		(unsigned long)snapshot.angle_error_count);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
static volatile float foc_latest_mechanical_angle_rad = 0.0f;
static volatile float foc_latest_mechanical_velocity_rad_s = 0.0f;
static volatile uint32_t foc_latest_angle_timestamp_us = 0U;
static volatile bool foc_latest_angle_valid = false;
static portMUX_TYPE foc_angle_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t foc_angle_error_count = 0U;
static TaskHandle_t foc_angle_sensor_task_handle = NULL;
static esp_timer_handle_t foc_angle_sensor_timer = NULL;

static void foc_angle_timer_callback(void *pvParameter)
{
    (void)pvParameter;
    if (foc_angle_sensor_task_handle != NULL)
    {
        xTaskNotifyGive(foc_angle_sensor_task_handle);
    }
}

static void foc_angle_sensor_task(void *pvParameter)
{
    (void)pvParameter;

    foc_angle_sensor_task_handle = xTaskGetCurrentTaskHandle();
    const esp_timer_create_args_t timer_args = {
        .callback = foc_angle_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "foc_angle_timer",
        .skip_unhandled_events = true,
    };
    esp_err_t timer_result = esp_timer_create(&timer_args, &foc_angle_sensor_timer);
    if (timer_result != ESP_OK)
    {
        ESP_LOGE("FOC_ANGLE", "Failed to create angle timer: %s", esp_err_to_name(timer_result));
        vTaskDelete(NULL);
        return;
    }
    timer_result = esp_timer_start_periodic(foc_angle_sensor_timer, 500);
    if (timer_result != ESP_OK)
    {
        ESP_LOGE("FOC_ANGLE", "Failed to start angle timer: %s", esp_err_to_name(timer_result));
        esp_timer_delete(foc_angle_sensor_timer);
        foc_angle_sensor_timer = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        /* The 500 us esp_timer provides the sample cadence; no 1 ms RTOS tick is involved. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float mechanical_angle = 0.0f;
        float mechanical_velocity_rad_s = 0.0f;
        int64_t angle_read_start_us = esp_timer_get_time();
        esp_err_t result = as5600_measure_angle_velocity(
            &mechanical_angle,
            &mechanical_velocity_rad_s);
        int64_t angle_read_end_us = esp_timer_get_time();
        if (result == ESP_OK)
        {
            uint32_t angle_snapshot_timestamp_us =
                (uint32_t)((angle_read_start_us + angle_read_end_us) / 2);
            portENTER_CRITICAL(&foc_angle_lock);
            foc_latest_mechanical_angle_rad = mechanical_angle;
            foc_latest_mechanical_velocity_rad_s = mechanical_velocity_rad_s;
            foc_latest_angle_timestamp_us = angle_snapshot_timestamp_us;
            foc_latest_angle_valid = true;
            portEXIT_CRITICAL(&foc_angle_lock);
        }
        else
        {
            foc_angle_error_count++;
            if ((foc_angle_error_count % 1000U) == 1U)
            {
                ESP_LOGW(
                    "FOC_ANGLE",
                    "AS5600 read failed: %s, errors=%lu",
                    esp_err_to_name(result),
                    (unsigned long)foc_angle_error_count);
            }

        }
    }
}
static void foc_current_task(void *pvParameter)
{
    (void)pvParameter;
    esp_err_t result = motor_pwm_register_control_task(xTaskGetCurrentTaskHandle());
    if (result != ESP_OK)
    {
        ESP_LOGE("FOC_CURRENT", "Failed to register PWM control task: %s", esp_err_to_name(result));
        vTaskDelete(NULL);
        return;
    }
    result = motor_pwm_set_duty(0.5f, 0.5f, 0.5f);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_CURRENT",
			"Failed to set initial duty: %s",
			esp_err_to_name(result));

		vTaskDelete(NULL);
		return;
	}
	foc_controller_reset(&foc_controller);
	result = motor_pwm_start();
	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_CURRENT",
			"Failed to start PWM: %s",
			esp_err_to_name(result));

		motor_pwm_stop();
		vTaskDelete(NULL);
		return;
	}
	ESP_LOGI(
		"FOC_CURRENT",
		"PWM started, current loop begins");
	/* Discard notifications accumulated while PWM was being started. */
	(void)ulTaskNotifyTake(pdTRUE, 0);

	uint32_t loop_count = 0;
    int64_t previous_iteration_start_us = 0;
    uint32_t actual_period_us = 0U;
    uint32_t period_min_us = 0U;
    uint32_t period_max_us = 0U;
    uint32_t period_jitter_us = 0U;
	uint32_t current_loop_missed_ticks = 0U;
	uint32_t current_loop_overrun_count = 0U;
	uint32_t current_loop_max_us = 0U;
	float iq_ref_a = 0.0f;
#if M1_ENABLE_SPEED_LOOP
	float speed_loop_elapsed_s = 0.0f;
#endif
	while (1)
	{
		uint32_t pending_ticks =
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (pending_ticks > 1U)
		{
			current_loop_missed_ticks += pending_ticks - 1U;
		}

		loop_count++;
		int64_t iteration_start_us = esp_timer_get_time();
        /* The PWM event wakes this task; use the newest ADC frame already complete at loop entry. */
        int64_t current_target_timestamp_us = iteration_start_us;
        if (previous_iteration_start_us != 0 &&
            iteration_start_us >= previous_iteration_start_us)
        {
            actual_period_us = (uint32_t)(iteration_start_us - previous_iteration_start_us);
            if (period_min_us == 0U || actual_period_us < period_min_us)
            {
                period_min_us = actual_period_us;
            }
            if (actual_period_us > period_max_us)
            {
                period_max_us = actual_period_us;
            }
            uint32_t nominal_period_us = (uint32_t)(FOC_CURRENT_TS_S * 1000000.0f);
            uint32_t jitter_us = actual_period_us > nominal_period_us
                ? actual_period_us - nominal_period_us
                : nominal_period_us - actual_period_us;
            if (jitter_us > period_jitter_us)
            {
                period_jitter_us = jitter_us;
            }
        }
        previous_iteration_start_us = iteration_start_us;
#if M1_ENABLE_TIMING_LOG
		int64_t angle_done_us = 0;
		int64_t current_done_us = 0;
		int64_t pwm_done_us = 0;
#endif

        /* PI integration uses the PWM notification interval, not scheduler latency. */
        float control_dt_s = FOC_CURRENT_TS_S;

        float measured_mechanical_angle = 0.0f;
        float mechanical_velocity_rad_s = 0.0f;
        uint32_t angle_snapshot_timestamp_us = 0U;
        bool angle_snapshot_valid = false;
        portENTER_CRITICAL(&foc_angle_lock);
        measured_mechanical_angle = foc_latest_mechanical_angle_rad;
        mechanical_velocity_rad_s = foc_latest_mechanical_velocity_rad_s;
        angle_snapshot_timestamp_us = foc_latest_angle_timestamp_us;
        angle_snapshot_valid = foc_latest_angle_valid;
        portEXIT_CRITICAL(&foc_angle_lock);

        if (!angle_snapshot_valid)
        {
            foc_current_snapshot.dt_s = control_dt_s;
            foc_current_snapshot.missed_ticks = current_loop_missed_ticks;
            foc_current_snapshot.overrun_count = current_loop_overrun_count;
            foc_current_snapshot.max_loop_us = current_loop_max_us;
            foc_current_snapshot.angle_valid = 0U;
            foc_current_snapshot.angle_error_count = foc_angle_error_count;
            continue;
        }

        uint32_t angle_age_us =
            (uint32_t)iteration_start_us - angle_snapshot_timestamp_us;
        if (angle_age_us > 2000U)
        {
            angle_age_us = 2000U;
        }
        float mechanical_angle = 0.0f;
        float electrical_angle = 0.0f;

		float iu_a = 0.0f;
		float iv_a = 0.0f;
		float iw_a = 0.0f;
        int64_t current_sample_timestamp_us = 0;
        uint32_t current_age_us = 0U;
        uint32_t current_sequence = 0U;
        int64_t angle_to_current_sample_us = 0;
        int32_t electrical_angle_mrad = 0;

#if M1_ENABLE_TIMING_LOG
		angle_done_us = esp_timer_get_time();
#endif
        /* 从ADC DMA任务发布的最新帧读取三相电流。 */
		if (result == ESP_OK)
		{
			result = current_sense_read_three_phase_at_or_before_timestamp(
                &iu_a,
                &iv_a,
                &iw_a,
                current_target_timestamp_us,
                &current_sample_timestamp_us,
                &current_sequence);
#if M1_ENABLE_TIMING_LOG
			current_done_us = esp_timer_get_time();
#endif
		}
        if (result == ESP_OK)
        {
            angle_to_current_sample_us =
                current_sample_timestamp_us -
                (int64_t)angle_snapshot_timestamp_us;
            if (angle_to_current_sample_us > 2000)
            {
                angle_to_current_sample_us = 2000;
            }
            else if (angle_to_current_sample_us < -2000)
            {
                angle_to_current_sample_us = -2000;
            }
            mechanical_angle = measured_mechanical_angle +
                mechanical_velocity_rad_s *
                    ((float)angle_to_current_sample_us / 1000000.0f);
            electrical_angle = foc_mechanical_to_electrical_angle(
                mechanical_angle,
                M1_MOTOR_POLE_PAIRS,
                foc_electrical_zero_offset_rad);
            if (current_sample_timestamp_us < iteration_start_us)
            {
                current_age_us = (uint32_t)(iteration_start_us - current_sample_timestamp_us);
            }
        }

		/*
		 * Independent bench-test protection: stop before a bad feedback
		 * value or an accumulating PI can drive excessive phase current.
		 */
		if (result == ESP_OK &&
			(!isfinite(iu_a) || !isfinite(iv_a) || !isfinite(iw_a) ||
			 fabsf(iu_a) > FOC_TEST_CURRENT_LIMIT_A ||
			 fabsf(iv_a) > FOC_TEST_CURRENT_LIMIT_A ||
			 fabsf(iw_a) > FOC_TEST_CURRENT_LIMIT_A))
		{
			ESP_LOGE(
				"FOC_CURRENT",
				"Over-current or invalid current: iu=%.3f, iv=%.3f, iw=%.3f A",
				iu_a,
				iv_a,
				iw_a);

			foc_controller_reset(&foc_controller);
			foc_speed_pi_reset(&foc_speed_controller);
			motor_pwm_set_duty(0.5f, 0.5f, 0.5f);
			motor_pwm_stop();
			vTaskDelete(NULL);
			return;
		}

        float iq_target_a = iq_ref_a;
#if M1_ENABLE_SPEED_LOOP
        if (result == ESP_OK)
        {
            speed_loop_elapsed_s += control_dt_s;
            if (speed_loop_elapsed_s >=
                (float)M1_SPEED_LOOP_PERIOD_MS / 1000.0f)
            {
                float speed_feedback_rad_s =
                    M1_SPEED_FEEDBACK_SIGN * mechanical_velocity_rad_s;
                result = foc_speed_pi_update(
                    &foc_speed_controller,
                    M1_SPEED_REF_RAD_S,
                    speed_feedback_rad_s,
                    speed_loop_elapsed_s,
                    &iq_target_a);
                speed_loop_elapsed_s = 0.0f;
            }
        }
#else
        iq_target_a = FOC_TEST_IQ_REF_A;
#endif
        if (result == ESP_OK)
        {
            float current_reference_ramp =
                M1_CURRENT_REFERENCE_RAMP_A_PER_S * control_dt_s;
            float current_reference_error = iq_target_a - iq_ref_a;
            if (fabsf(current_reference_error) <= current_reference_ramp)
            {
                iq_ref_a = iq_target_a;
            }
            else
            {
                iq_ref_a += copysignf(current_reference_ramp,
                    current_reference_error);
            }
        }
        /*
         * current_age_us is measured from the ADC frame midpoint to loop entry.
         * Add one center-aligned PWM update window (about 50 us) so inverse Park
         * predicts the angle when the new compare value becomes effective.
         */
        int64_t estimated_output_delay_us = (int64_t)current_age_us + 50;
        foc_controller_input_t controller_input = {0};
		foc_controller_output_t controller_output = {0};
		if (result == ESP_OK)
		{
			controller_input.iu_a = iu_a;
			controller_input.iv_a = iv_a;
			controller_input.iw_a = iw_a;
			controller_input.electrical_angle_rad = electrical_angle;
            /* Compensate the one-sample computation/PWM update delay at high speed. */
            controller_input.output_electrical_angle_rad = electrical_angle +
                mechanical_velocity_rad_s * M1_MOTOR_POLE_PAIRS *
                    ((float)estimated_output_delay_us / 1000000.0f);
			controller_input.id_ref_a = FOC_TEST_ID_REF_A;
			controller_input.iq_ref_a = iq_ref_a;
			controller_input.dt_s = control_dt_s;
			controller_input.bus_voltage_v = FOC_TEST_BUS_VOLTAGE_V;
			controller_input.electrical_velocity_rad_s =
				mechanical_velocity_rad_s * M1_MOTOR_POLE_PAIRS;
			controller_input.motor_phase_resistance_ohm =
				M1_MOTOR_PHASE_RESISTANCE_OHM;
			controller_input.motor_inductance_d_h =
				M1_MOTOR_INDUCTANCE_D_H;
			controller_input.motor_inductance_q_h =
				M1_MOTOR_INDUCTANCE_Q_H;
			controller_input.motor_flux_linkage_wb =
				M1_MOTOR_FLUX_LINKAGE_WB;
			result = foc_controller_step(&foc_controller, &controller_input, &controller_output);
		}
		if (result == ESP_OK)
		{
			result = motor_pwm_set_duty(
				controller_output.duty_u,
				controller_output.duty_v,
				controller_output.duty_w);
#if M1_ENABLE_TIMING_LOG
			pwm_done_us = esp_timer_get_time();
#endif
		}
		if (result == ESP_OK)
		{
			uint32_t loop_elapsed_us =
				(uint32_t)(esp_timer_get_time() - iteration_start_us);
			if (loop_elapsed_us > current_loop_max_us)
			{
				current_loop_max_us = loop_elapsed_us;
			}
			if (loop_elapsed_us >
				(uint32_t)(FOC_CURRENT_TS_S * 1000000.0f))
			{
				current_loop_overrun_count++;
			}

			foc_current_snapshot.dt_s = control_dt_s;
			foc_current_snapshot.mechanical_speed_rad_s =
				M1_SPEED_FEEDBACK_SIGN * mechanical_velocity_rad_s;
			foc_current_snapshot.iq_ref_a = iq_ref_a;
			foc_current_snapshot.iu_a = iu_a;
			foc_current_snapshot.iv_a = iv_a;
			foc_current_snapshot.iw_a = iw_a;
			foc_current_snapshot.id_a = controller_output.i_d_a;
			foc_current_snapshot.iq_a = controller_output.i_q_a;
			foc_current_snapshot.vd_v = controller_output.vd_v;
			foc_current_snapshot.vq_v = controller_output.vq_v;
			foc_current_snapshot.vd_decoupling_v =
				controller_output.vd_decoupling_v;
			foc_current_snapshot.vq_decoupling_v =
				controller_output.vq_decoupling_v;
			foc_current_snapshot.voltage_vector_limit_v =
				controller_output.voltage_vector_limit_v;
			foc_current_snapshot.duty_u = controller_output.duty_u;
			foc_current_snapshot.duty_v = controller_output.duty_v;
			foc_current_snapshot.duty_w = controller_output.duty_w;
			foc_current_snapshot.missed_ticks = current_loop_missed_ticks;
			foc_current_snapshot.overrun_count = current_loop_overrun_count;
			foc_current_snapshot.max_loop_us = current_loop_max_us;
			foc_current_snapshot.angle_valid = angle_snapshot_valid ? 1U : 0U;
			foc_current_snapshot.angle_error_count = foc_angle_error_count;
            foc_current_snapshot.angle_age_us = angle_age_us;
            foc_current_snapshot.current_age_us = current_age_us;
            foc_current_snapshot.current_sequence = current_sequence;
            foc_current_snapshot.angle_to_current_sample_us = (int32_t)angle_to_current_sample_us;
            electrical_angle_mrad = (int32_t)(electrical_angle * 1000.0f);
            foc_current_snapshot.electrical_angle_mrad = electrical_angle_mrad;
            foc_current_snapshot.actual_period_us = actual_period_us;
            foc_current_snapshot.period_min_us = period_min_us;
            foc_current_snapshot.period_max_us = period_max_us;
            foc_current_snapshot.period_jitter_us = period_jitter_us;
            foc_current_snapshot.angle_velocity_mrad_s = mechanical_velocity_rad_s * 1000.0f;
            foc_current_snapshot.id_pi_integral_v = foc_controller.id_pi.integral;
            foc_current_snapshot.iq_pi_integral_v = foc_controller.iq_pi.integral;
#if M1_ENABLE_CURRENT_TRACE
if ((loop_count % M1_CURRENT_TRACE_INTERVAL_LOOPS) == 0U)
			{
				#if M1_ENABLE_TIMING_LOG
				ESP_LOGI(
					"FOC_TIMING",
					"sample_dt=%.3f ms, angle=%.0f us, current=%.0f us, math_pwm=%.0f us, loop=%.0f us",
					control_dt_s * 1000.0f,
					(float)(angle_done_us - iteration_start_us),
					(float)(current_done_us - angle_done_us),
					(float)(pwm_done_us - current_done_us),
					(float)(pwm_done_us - iteration_start_us));
				#endif

				ESP_LOGI(
					"FOC_CURRENT_TEST",
					"dt: %.3f ms, mech: %.2f, elec: %.2f, "
					"speed_ref: %.3f, speed: %.3f, iq_ref: %.3f, "
					"iu: %.3f, iv: %.3f, iw: %.3f, "
					"ialpha: %.3f, ibeta: %.3f, "
					"id: %.3f, iq: %.3f, "
					"id_error: %.3f, iq_error: %.3f, "
					"vd: %.3f V, vq: %.3f V, "
					"valpha: %.3f V, vbeta: %.3f V, "
					"u_voltage: %.3f V, v_voltage: %.3f V, w_voltage: %.3f V, "
					"duty_u: %.3f, duty_v: %.3f, duty_w: %.3f",
					control_dt_s * 1000.0f,
					mechanical_angle,
					electrical_angle,
					M1_SPEED_REF_RAD_S,
					M1_SPEED_FEEDBACK_SIGN * mechanical_velocity_rad_s,
					iq_ref_a,
					iu_a,
					iv_a,
					iw_a,
					controller_output.i_alpha_a,
					controller_output.i_beta_a,
					controller_output.i_d_a,
					controller_output.i_q_a,
					controller_output.id_error_a,
					controller_output.iq_error_a,
					controller_output.vd_v,
					controller_output.vq_v,
					controller_output.v_alpha_v,
					controller_output.v_beta_v,
					controller_output.u_voltage_v,
					controller_output.v_voltage_v,
					controller_output.w_voltage_v,
					controller_output.duty_u,
					controller_output.duty_v,
					controller_output.duty_w);
			}
#endif
		}
		else
		{
			ESP_LOGE(
				"FOC_CURRENT",
				"FOC current loop failed: %s",
				esp_err_to_name(result));

			motor_pwm_stop();
			vTaskDelete(NULL);
			return;
		}

	}
}
/* 参数化电压的占空比计算（把宏换成参数） */

void app_main()
{
    led_init();
    ws2812_init();
    motor_pwm_init();
	esp_err_t as5600_result = as5600_init();
	if (as5600_result != ESP_OK)
	{
		ESP_LOGE("MAIN", "AS5600 initialization failed: %s", esp_err_to_name(as5600_result));
	}
	current_sense_init();
	current_sense_calibrate();

	foc_controller_init(
		&foc_controller,
		FOC_TEST_ID_PI_KP,
		FOC_TEST_ID_PI_KI,
		FOC_TEST_IQ_PI_KP,
		FOC_TEST_IQ_PI_KI,
		FOC_TEST_PI_OUTPUT_MIN_V,
		FOC_TEST_PI_OUTPUT_MAX_V);
	foc_speed_pi_init(
		&foc_speed_controller,
		M1_SPEED_PI_KP,
		M1_SPEED_PI_KI,
		-M1_SPEED_IQ_LIMIT_A,
		M1_SPEED_IQ_LIMIT_A);
#if M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION
	// 只有明确打开配置宏时，启动阶段才执行一次电角度零点校准。
	electrical_zero_calibration();
#endif
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    xTaskCreate(ws2812_task, "ws2812_task", 2048, NULL, 3, NULL);
    xTaskCreatePinnedToCore(foc_angle_sensor_task, "foc_angle_sensor_task", 3072, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(foc_current_task, "foc_current_task", 4096, NULL, 9, NULL, 1);
    xTaskCreate(foc_current_telemetry_task, "foc_current_telemetry_task", 4096, NULL, 4, NULL);
}
