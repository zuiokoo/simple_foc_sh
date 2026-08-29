#include <stdio.h>
#include "control/foc_speed_pi.h"
#include "freertos/FreeRTOS.h" // 添加ESP-IDF FreeRTOS头文件
#include "freertos/task.h"	   // 添加ESP-IDF任务头文件
#include "driver/gpio.h"	   // GPIO 驱动头文件
#include "esp_log.h"		   // 日志功能头文件
#include "driver/spi_master.h"
#include "esp_timer.h"
#include <string.h>		   //使用 memset() 清零复位区域。
#include <math.h>		   //使用 fabsf() 判断电流绝对值。
#include "esp_heap_caps.h" //因为代码使用了：heap_caps_calloc()
#include "motor/motor_config.h"
#include "motor/motor_pwm.h"
#include "sensor/as5600.h"
#include "sensor/current_sense.h"
#include "control/foc_math.h"
#include "control/foc_svpwm.h"
#include "control/foc_controller.h"
#include "control/foc_open_loop.h"

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
/*
 * SPI 频率 2.4 MHz：
 *
 * 一个 SPI 位时间约为 0.416 us
 * 一个 WS2812 数据位使用 3 个 SPI 位
 * 一个 WS2812 数据位总时间约为 1.25 us
 *
 * WS2812 的 0 -> SPI 100
 * WS2812 的 1 -> SPI 110
 */
#define WS2812_SPI_HZ 2400000

/*
 * 每个 WS2812 数据位编码成 3 个 SPI 位
 *
 * 每个灯有 24 个数据位：
 * 24 * 3 = 72 个 SPI 位 = 9 个字节
 */
#define WS2812_BYTES_PER_LED 9
/*
 * 最后补 24 个 0 字节：
 *
 * 120 * 8 / 2.4 MHz = 400 us
 *
 * WS2812 要求数据结束后保持低电平超过约 280 us以上，
 * 这 24 个字节就是复位低电平。
 */
#define WS2812_RESET_BYTES 120
#define WS2812_DATA_BYTES (WS2812_COUNT * WS2812_BYTES_PER_LED)
#define WS2812_TX_BYTES (WS2812_DATA_BYTES + WS2812_RESET_BYTES)
static const char *TAG = "SPI_WS2812";
static spi_device_handle_t ws2812_spi;
static uint8_t *ws2812_tx_buffer;

/*
 * 电机控制参数。
 */
static foc_controller_t foc_controller;
static foc_open_loop_t foc_open_loop;
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
	float duty_u;
	float duty_v;
	float duty_w;
} foc_current_snapshot_t;

static volatile foc_current_snapshot_t foc_current_snapshot;

// 当前测试任务每10 ms执行一次，暂时用固定周期验证PI计算链路。
#define FOC_CURRENT_LOOP_PERIOD_MS 1U
#define FOC_TEST_DT_S 0.001f
#define FOC_TEST_DT_MIN_S 0.0005f
#define FOC_TEST_DT_MAX_S 0.005f

// 当前 d 轴和 q 轴目标电流都为0A，用于零电流数学测试。
#define FOC_TEST_ID_REF_A M1_CURRENT_LOOP_TEST_ID_REF_A
#define FOC_TEST_IQ_REF_A M1_CURRENT_LOOP_TEST_IQ_REF_A

// 临时测试参数，不是最终电机参数。
#define FOC_TEST_ID_PI_KP M1_CURRENT_LOOP_ID_PI_KP
#define FOC_TEST_ID_PI_KI M1_CURRENT_LOOP_ID_PI_KI
#define FOC_TEST_IQ_PI_KP M1_CURRENT_LOOP_IQ_PI_KP
#define FOC_TEST_IQ_PI_KI M1_CURRENT_LOOP_IQ_PI_KI
#define FOC_TEST_PI_OUTPUT_MIN_V (-2.0f)
#define FOC_TEST_PI_OUTPUT_MAX_V (2.0f)
#define FOC_TEST_CURRENT_LIMIT_A 2.0f

// 当前只用于软件计算测试，不代表实际测量值。
#define FOC_TEST_BUS_VOLTAGE_V 8.0f
// 开环电角速度，单位 rad/s。
#define FOC_OPEN_LOOP_SPEED_RAD_S 5.0f

// 开环测试电压，单位 V。
#define FOC_OPEN_LOOP_VOLTAGE_V 0.8f
// 开环控制任务周期，单位 s。
#define FOC_OPEN_LOOP_CONTROL_DT_S 0.01f
// 开环控制任务周期，单位 ms。
#define FOC_OPEN_LOOP_CONTROL_PERIOD_MS 10
// 上电后先固定磁场，让转子对齐。
#define FOC_OPEN_LOOP_ALIGN_TIME_MS 500
// 对齐结束后，用1秒逐渐增加速度。
#define FOC_OPEN_LOOP_RAMP_TIME_MS 1000

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
	esp_err_t result = motor_pwm_set_duty(
		M1_ALIGNMENT_DUTY_U,
		M1_ALIGNMENT_DUTY_V,
		M1_ALIGNMENT_DUTY_W);
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
			if (alignment_current_samples > 0)
			{
				float alignment_i_alpha = alignment_i_alpha_sum / (float)alignment_current_samples;
				float alignment_i_beta = alignment_i_beta_sum / (float)alignment_current_samples;
				float alignment_electrical_angle = atan2f(alignment_i_beta, alignment_i_alpha);
				if (alignment_electrical_angle < 0.0f)
				{
					alignment_electrical_angle += 2.0f * 3.14159265358979323846f;
				}
				electrical_zero_offset = fmodf(
					mechanical_electrical_angle - alignment_electrical_angle +
						2.0f * 3.14159265358979323846f,
					2.0f * 3.14159265358979323846f);
				ESP_LOGI(
					"FOC_CALIB",
					"alignment current vector: alpha=%.3f beta=%.3f angle=%.6f rad",
					alignment_i_alpha,
					alignment_i_beta,
					alignment_electrical_angle);
			}
			foc_electrical_zero_offset_rad = electrical_zero_offset;
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
static esp_err_t foc_open_loop_calculate_duty(
	float electrical_angle_rad,
	float *duty_u,
	float *duty_v,
	float *duty_w)
{
	float v_alpha_v = 0.0f;
	float v_beta_v = 0.0f;

	float u_voltage_v = 0.0f;
	float v_voltage_v = 0.0f;
	float w_voltage_v = 0.0f;

	esp_err_t result = foc_inverse_park_transform(
		0.0f,
		FOC_OPEN_LOOP_VOLTAGE_V,
		electrical_angle_rad,
		&v_alpha_v,
		&v_beta_v);

	if (result != ESP_OK)
	{
		return result;
	}

	result = foc_inverse_clarke_transform(
		v_alpha_v,
		v_beta_v,
		&u_voltage_v,
		&v_voltage_v,
		&w_voltage_v);

	if (result != ESP_OK)
	{
		return result;
	}

	return foc_svpwm_calculate(
		u_voltage_v,
		v_voltage_v,
		w_voltage_v,
		FOC_TEST_BUS_VOLTAGE_V,
		duty_u,
		duty_v,
		duty_w);
}

static void foc_open_task(void *pvParameter)
{
	(void)pvParameter;

	foc_open_loop_t generator;
	foc_open_loop_init(&generator, 0.0f);

	float duty_u = 0.0f;
	float duty_v = 0.0f;
	float duty_w = 0.0f;

	/*
	 * 先计算电角度0对应的固定电压矢量。
	 */
	esp_err_t result = foc_open_loop_calculate_duty(
		generator.electrical_angle_rad,
		&duty_u,
		&duty_v,
		&duty_w);

	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_OPEN_LOOP",
			"Alignment duty calculation failed: %s",
			esp_err_to_name(result));

		vTaskDelete(NULL);
		return;
	}

	/*
	 * 先设置固定对齐占空比，再启动PWM。
	 */
	result = motor_pwm_set_duty(
		duty_u,
		duty_v,
		duty_w);

	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_OPEN_LOOP",
			"Failed to set alignment duty: %s",
			esp_err_to_name(result));

		vTaskDelete(NULL);
		return;
	}

	result = motor_pwm_start();

	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_OPEN_LOOP",
			"Failed to start PWM: %s",
			esp_err_to_name(result));

		motor_pwm_stop();
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(
		"FOC_OPEN_LOOP",
		"PWM started, rotor alignment begins");

	TickType_t last_wake_time = xTaskGetTickCount();
	TickType_t start_time = last_wake_time;

	while (1)
	{
		TickType_t now = xTaskGetTickCount();

		uint32_t elapsed_ms =
			pdTICKS_TO_MS(now - start_time);

		float electrical_speed_rad_s = 0.0f;

		if (elapsed_ms < FOC_OPEN_LOOP_ALIGN_TIME_MS)
		{
			/*
			 * 对齐阶段：
			 * 电角度保持0不变。
			 */
			generator.electrical_angle_rad = 0.0f;
		}
		else
		{
			/*
			 * 对齐结束后，速度逐渐增加。
			 */
			uint32_t ramp_elapsed_ms =
				elapsed_ms - FOC_OPEN_LOOP_ALIGN_TIME_MS;

			if (ramp_elapsed_ms < FOC_OPEN_LOOP_RAMP_TIME_MS)
			{
				float ramp_ratio =
					(float)ramp_elapsed_ms /
					(float)FOC_OPEN_LOOP_RAMP_TIME_MS;

				electrical_speed_rad_s =
					FOC_OPEN_LOOP_SPEED_RAD_S *
					ramp_ratio;
			}
			else
			{
				electrical_speed_rad_s =
					FOC_OPEN_LOOP_SPEED_RAD_S;
			}
			// 开环项目中，外部给目标速度，程序把它换算成电角速度并积分生成电角度
			result = foc_open_loop_step(
				&generator,
				electrical_speed_rad_s,
				FOC_OPEN_LOOP_CONTROL_DT_S,
				&generator.electrical_angle_rad);

			if (result != ESP_OK)
			{
				ESP_LOGE(
					"FOC_OPEN_LOOP",
					"Angle update failed: %s",
					esp_err_to_name(result));

				motor_pwm_stop();
				vTaskDelete(NULL);
				return;
			}
		}

		/*
		 * 当前命令电角度重新计算三相占空比。
		 */
		result = foc_open_loop_calculate_duty(
			generator.electrical_angle_rad,
			&duty_u,
			&duty_v,
			&duty_w);

		if (result != ESP_OK)
		{
			ESP_LOGE(
				"FOC_OPEN_LOOP",
				"Duty calculation failed: %s",
				esp_err_to_name(result));

			motor_pwm_stop();
			vTaskDelete(NULL);
			return;
		}

		/*
		 * 把开环占空比更新到PWM比较器。
		 */
		result = motor_pwm_set_duty(
			duty_u,
			duty_v,
			duty_w);

		if (result != ESP_OK)
		{
			ESP_LOGE(
				"FOC_OPEN_LOOP",
				"Duty update failed: %s",
				esp_err_to_name(result));

			motor_pwm_stop();
			vTaskDelete(NULL);
			return;
		}
		// 在开环循环中，每次更新电角度后，读取机械角度并打印对比
		float mechanical_angle_rad = 0.0f;
		float mechanical_velocity_rad_s = 0.0f;
		as5600_measure_angle_velocity(&mechanical_angle_rad, &mechanical_velocity_rad_s);

		float theoretical_electrical_angle = foc_mechanical_to_electrical_angle(
			mechanical_angle_rad,
			M1_MOTOR_POLE_PAIRS,
			0.0f); // 暂时假设偏移为0

		ESP_LOGI("FOC_OPEN_LOOP_DEBUG",
				 "cmd_elec=%.3f, mech=%.3f, pole=%.3f, theo_elec(offset0)=%.3f, offset_used=%.3f",
				 generator.electrical_angle_rad,
				 mechanical_angle_rad,
				 (float)M1_MOTOR_POLE_PAIRS,
				 theoretical_electrical_angle,
				 M1_ELECTRICAL_ZERO_OFFSET_RAD);
		/*
		 * 每10 ms执行一次。
		 */
		vTaskDelayUntil(
			&last_wake_time,
			pdMS_TO_TICKS(
				FOC_OPEN_LOOP_CONTROL_PERIOD_MS));
	}
}
static void foc_current_telemetry_task(void *pvParameter)
{
	(void)pvParameter;
	while (1)
	{
		foc_current_snapshot_t snapshot;
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
		snapshot.duty_u = foc_current_snapshot.duty_u;
		snapshot.duty_v = foc_current_snapshot.duty_v;
		snapshot.duty_w = foc_current_snapshot.duty_w;

		/*
		 * Do not use floating-point printf here.  Newlib's float formatter
		 * consumes a large amount of task stack and previously overflowed
		 * this low-priority telemetry task immediately after startup.
		 * Values are scaled for readability while keeping the current-loop
		 * task completely free of logging.
		 */
		ESP_LOGI(
			"FOC_TELEM",
			"dt_us=%ld speed_mrad_s=%ld iq_ref_mA=%ld iu_mA=%ld iv_mA=%ld iw_mA=%ld id_mA=%ld iq_mA=%ld vd_mV=%ld vq_mV=%ld duty=%ld/%ld/%ld",
			(long)(snapshot.dt_s * 1000000.0f),
			(long)(snapshot.mechanical_speed_rad_s * 1000.0f),
			(long)(snapshot.iq_ref_a * 1000.0f),
			(long)(snapshot.iu_a * 1000.0f),
			(long)(snapshot.iv_a * 1000.0f),
			(long)(snapshot.iw_a * 1000.0f),
			(long)(snapshot.id_a * 1000.0f),
			(long)(snapshot.iq_a * 1000.0f),
			(long)(snapshot.vd_v * 1000.0f),
			(long)(snapshot.vq_v * 1000.0f),
			(long)(snapshot.duty_u * 1000.0f),
			(long)(snapshot.duty_v * 1000.0f),
			(long)(snapshot.duty_w * 1000.0f));

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
static void foc_current_task(void *pvParameter)
{
	(void)pvParameter;
	TickType_t last_wake_time = xTaskGetTickCount();

	esp_err_t result = motor_pwm_set_duty(0.5f, 0.5f, 0.5f);
	if (result != ESP_OK)
	{
		ESP_LOGE(
			"FOC_CURRENT",
			"Failed to set initial duty: %s",
			esp_err_to_name(result));

		vTaskDelete(NULL);
		return;
	}
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
	uint32_t loop_count = 0;
	float iq_ref_a = 0.0f;
	float speed_loop_elapsed_s = 0.0f;
	int64_t last_current_sample_time_us = 0;
	while (1)
	{
		loop_count++;
		int64_t iteration_start_us = esp_timer_get_time();
		int64_t angle_done_us = 0;
		int64_t current_done_us = 0;
		int64_t pwm_done_us = 0;

		/* 用实际循环间隔作为 PI 的 dt，异常时回退到标称 1 ms。 */
		float control_dt_s = FOC_TEST_DT_S;

		float mechanical_angle = 0.0f;
		float mechanical_velocity_rad_s = 0.0f;
		float electrical_angle = 0.0f;

		float iu_a = 0.0f;
		float iv_a = 0.0f;
		float iw_a = 0.0f;

		/*
		 * 第一步：读取机械角度。
		 */
		result = as5600_measure_angle_velocity(
			&mechanical_angle,
			&mechanical_velocity_rad_s);
		angle_done_us = esp_timer_get_time();
		/*
		 * 第二步：机械角度转换为电角度。
		 */
		if (result == ESP_OK)
		{
			electrical_angle = foc_mechanical_to_electrical_angle(mechanical_angle, M1_MOTOR_POLE_PAIRS, foc_electrical_zero_offset_rad);
		}
		/*
		 * 第三步：读取三相电流。
		 */
		if (result == ESP_OK)
		{
			result = current_sense_read_three_phase(&iu_a, &iv_a, &iw_a);
			current_done_us = esp_timer_get_time();
		}
		if (result == ESP_OK)
		{
			int64_t current_sample_time_us = esp_timer_get_time();
			if (last_current_sample_time_us > 0)
			{
				float measured_dt_s =
					(float)(current_sample_time_us - last_current_sample_time_us) /
					1000000.0f;
				if (isfinite(measured_dt_s) &&
					measured_dt_s >= FOC_TEST_DT_MIN_S &&
					measured_dt_s <= FOC_TEST_DT_MAX_S)
				{
					control_dt_s = measured_dt_s;
				}
			}
			last_current_sample_time_us = current_sample_time_us;
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
					&iq_ref_a);
				speed_loop_elapsed_s = 0.0f;
			}
		}
		#else
		iq_ref_a = FOC_TEST_IQ_REF_A;
		#endif
		foc_controller_input_t controller_input = {0};
		foc_controller_output_t controller_output = {0};
		if (result == ESP_OK)
		{
			controller_input.iu_a = iu_a;
			controller_input.iv_a = iv_a;
			controller_input.iw_a = iw_a;
			controller_input.electrical_angle_rad = electrical_angle;
			controller_input.id_ref_a = FOC_TEST_ID_REF_A;
			controller_input.iq_ref_a = iq_ref_a;
			controller_input.dt_s = control_dt_s;
			controller_input.bus_voltage_v = FOC_TEST_BUS_VOLTAGE_V;
			result = foc_controller_step(&foc_controller, &controller_input, &controller_output);
		}
		#if M1_ENABLE_FIXED_PWM_TEST
		if (result == ESP_OK)
		{
			controller_output.duty_u = M1_FIXED_PWM_TEST_DUTY_U;
			controller_output.duty_v = M1_FIXED_PWM_TEST_DUTY_V;
			controller_output.duty_w = M1_FIXED_PWM_TEST_DUTY_W;
		}
		#endif
		if (result == ESP_OK)
		{
			result = motor_pwm_set_duty(
				controller_output.duty_u,
				controller_output.duty_v,
				controller_output.duty_w);
			pwm_done_us = esp_timer_get_time();
		}
		if (result == ESP_OK)
		{

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
			foc_current_snapshot.duty_u = controller_output.duty_u;
			foc_current_snapshot.duty_v = controller_output.duty_v;
			foc_current_snapshot.duty_w = controller_output.duty_w;
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

		vTaskDelayUntil(
			&last_wake_time,
			pdMS_TO_TICKS(FOC_CURRENT_LOOP_PERIOD_MS));
	}
}
/* 参数化电压的占空比计算（把宏换成参数） */
static esp_err_t calc_duty_at(float angle_rad, float voltage_v,
							  float *du, float *dv, float *dw)
{
	float va = 0.0f, vb = 0.0f;
	float vu = 0.0f, vv = 0.0f, vw = 0.0f;

	esp_err_t r = foc_inverse_park_transform(0.0f, voltage_v, angle_rad, &va, &vb);
	if (r != ESP_OK)
		return r;
	r = foc_inverse_clarke_transform(va, vb, &vu, &vv, &vw);
	if (r != ESP_OK)
		return r;
	return foc_svpwm_calculate(vu, vv, vw, FOC_TEST_BUS_VOLTAGE_V, du, dv, dw);
}


void app_main()
{

	led_init();
	ws2812_init();
	motor_pwm_init();
	as5600_init();
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
	foc_open_loop_init(&foc_open_loop, 0.0f);
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
#if !M1_ENABLE_FOC_CURRENT_LOOP
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(ws2812_task, "ws2812_task", 2048, NULL, 5, NULL);
#endif
#if M1_ENABLE_FOC_CURRENT_LOOP
    xTaskCreate(foc_current_task, "foc_current_task", 4096, NULL, 7, NULL);
#if M1_ENABLE_CURRENT_TELEMETRY_TASK
    	// Stack depth is in words. Keep telemetry isolated from the real-time current loop.
    xTaskCreate(foc_current_telemetry_task, "foc_current_telemetry_task", 4096, NULL, 4, NULL);
#endif
#endif


}
