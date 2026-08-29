#include "motor_pwm.h"
#include "motor_config.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
// IDF 5.5 的 mcpwm prelude 驱动没有公开读取计数器的 API，
// 同步采样只能通过 HAL/LL 层直接读 MCPWM0 的计数器。
#include "hal/mcpwm_ll.h"
#include "soc/mcpwm_struct.h"

// 记录三相最近一次写入的 compare 值（tick），供采样同步用
static uint32_t s_last_compare_ticks[3] = {125, 125, 125};

static const char *TAG = "MOTOR_PWM";

// One timer is shared by all three phases.
static mcpwm_timer_handle_t pwm_timer = NULL;
// U, V and W each have one operator, comparator and generator.
static mcpwm_oper_handle_t pwm_operators[3] = {NULL, NULL, NULL};
static mcpwm_cmpr_handle_t pwm_comparators[3] = {NULL, NULL, NULL};
static mcpwm_gen_handle_t pwm_generators[3] = {NULL, NULL, NULL};

static bool pwm_initialized = false;
static bool pwm_timer_enabled = false;
static bool pwm_running = false;

static float clamp_duty(float duty)
{
	if (duty < M1_PWM_MIN_DUTY)
	{
		return M1_PWM_MIN_DUTY;
	}
	if (duty > M1_PWM_MAX_DUTY)
	{
		return M1_PWM_MAX_DUTY;
	}
	return duty;
}

esp_err_t motor_pwm_init(void)
{
	if (pwm_timer != NULL)
	{
		ESP_LOGW(TAG, "MCPWM timer already created");
		return ESP_ERR_INVALID_STATE;
	}

	mcpwm_timer_config_t timer_config = {
		.group_id = 0,
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.resolution_hz = M1_PWM_TIMER_RESOLUTION_HZ,
		.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
		.period_ticks = M1_PWM_PERIOD_TICKS,
	};

	esp_err_t result = mcpwm_new_timer(&timer_config, &pwm_timer);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to create MCPWM timer: %s", esp_err_to_name(result));
		return result;
	}

	ESP_LOGI(TAG,
			 "MCPWM timer created: resolution=%lu Hz, PWM=%lu Hz, period=%lu ticks",
			 M1_PWM_TIMER_RESOLUTION_HZ,
			 M1_PWM_FREQUENCY_HZ,
			 M1_PWM_PERIOD_TICKS);

	mcpwm_operator_config_t operator_config = {
		.group_id = 0,
	};

	for (int phase = 0; phase < 3; phase++)
	{
		result = mcpwm_new_operator(&operator_config, &pwm_operators[phase]);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to create operator %d: %s", phase, esp_err_to_name(result));
			return result;
		}

		result = mcpwm_operator_connect_timer(pwm_operators[phase], pwm_timer);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to connect operator %d: %s", phase, esp_err_to_name(result));
			return result;
		}
	}
	ESP_LOGI(TAG, "Three MCPWM operators created and connected");

	mcpwm_comparator_config_t comparator_config = {
		.flags.update_cmp_on_tez = true,
	};

	for (int phase = 0; phase < 3; phase++)
	{
		result = mcpwm_new_comparator(
			pwm_operators[phase],
			&comparator_config,
			&pwm_comparators[phase]);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to create comparator %d: %s", phase, esp_err_to_name(result));
			return result;
		}

		result = mcpwm_comparator_set_compare_value(
			pwm_comparators[phase],
			M1_PWM_PERIOD_TICKS / 2);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to set comparator %d: %s", phase, esp_err_to_name(result));
			return result;
		}
	}
	ESP_LOGI(TAG, "Three comparators created, initial duty=50%%");

	const int pwm_gpio[3] = {
		M1_PWM_U_GPIO,
		M1_PWM_V_GPIO,
		M1_PWM_W_GPIO,
	};

	for (int phase = 0; phase < 3; phase++)
	{
		mcpwm_generator_config_t generator_config = {
			.gen_gpio_num = pwm_gpio[phase],
		};

		result = mcpwm_new_generator(
			pwm_operators[phase],
			&generator_config,
			&pwm_generators[phase]);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG,
					 "Failed to create generator %d on GPIO%d: %s",
					 phase,
					 pwm_gpio[phase],
					 esp_err_to_name(result));
			return result;
		}

		// Counting up to compare value makes the output LOW.
		result = mcpwm_generator_set_action_on_compare_event(
			pwm_generators[phase],
			MCPWM_GEN_COMPARE_EVENT_ACTION(
				MCPWM_TIMER_DIRECTION_UP,
				pwm_comparators[phase],
				MCPWM_GEN_ACTION_LOW));
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to set generator %d UP action: %s", phase, esp_err_to_name(result));
			return result;
		}

		// Counting down to compare value makes the output HIGH.
		result = mcpwm_generator_set_action_on_compare_event(
			pwm_generators[phase],
			MCPWM_GEN_COMPARE_EVENT_ACTION(
				MCPWM_TIMER_DIRECTION_DOWN,
				pwm_comparators[phase],
				MCPWM_GEN_ACTION_HIGH));
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to set generator %d DOWN action: %s", phase, esp_err_to_name(result));
			return result;
		}

		// Initialization is safe: no PWM reaches the gate driver yet.
		result = mcpwm_generator_set_force_level(pwm_generators[phase], 0, true);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to force generator %d LOW: %s", phase, esp_err_to_name(result));
			return result;
		}
	}

	pwm_initialized = true;
	ESP_LOGI(TAG,
			 "Three PWM generators configured: U=GPIO%d, V=GPIO%d, W=GPIO%d",
			 pwm_gpio[0],
			 pwm_gpio[1],
			 pwm_gpio[2]);
	ESP_LOGI(TAG, "PWM initialized in safe LOW state");
	return ESP_OK;
}

esp_err_t motor_pwm_start(void)
{
	if (!pwm_initialized)
	{
		ESP_LOGE(TAG, "PWM is not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	if (pwm_running)
	{
		ESP_LOGW(TAG, "PWM is already running");
		return ESP_OK;
	}

	esp_err_t result;
	if (!pwm_timer_enabled)
	{
		result = mcpwm_timer_enable(pwm_timer);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to enable PWM timer: %s", esp_err_to_name(result));
			return result;
		}
		pwm_timer_enabled = true;
	}

	// Start counting while the three outputs are still forced LOW.
	result = mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to start PWM timer: %s", esp_err_to_name(result));
		return result;
	}

	// Remove the forced LOW level and hand control to the PWM event rules.
	for (int phase = 0; phase < 3; phase++)
	{
		result = mcpwm_generator_set_force_level(pwm_generators[phase], -1, true);
		if (result != ESP_OK)
		{
			ESP_LOGE(TAG, "Failed to release generator %d: %s", phase, esp_err_to_name(result));
			motor_pwm_stop();
			return result;
		}
	}

	pwm_running = true;
	ESP_LOGI(TAG, "PWM started");
	return ESP_OK;
}

esp_err_t motor_pwm_set_duty(float duty_u, float duty_v, float duty_w)
{
	if (!pwm_initialized)
	{
		ESP_LOGE(TAG, "PWM is not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	// Array order is fixed: index 0=U, 1=V, 2=W.
	float duty[3] = {
		clamp_duty(duty_u),
		clamp_duty(duty_v),
		clamp_duty(duty_w),
	};

	for (int phase = 0; phase < 3; phase++)
	{
		// 20 kHz center-aligned PWM: compare range is 0..250, and 50% is 125.
		uint32_t compare_ticks =
			(uint32_t)(duty[phase] * M1_PWM_COMPARE_MAX_TICKS + 0.5f);

		esp_err_t result = mcpwm_comparator_set_compare_value(
			pwm_comparators[phase],
			compare_ticks);

		if (result != ESP_OK)
		{
			ESP_LOGE(TAG,
					 "Failed to set phase %d duty %.3f (%lu ticks): %s",
					 phase,
					 duty[phase],
					 (unsigned long)compare_ticks,
					 esp_err_to_name(result));
			return result;
		}

		// 写入成功才记录，保证同步窗口计算用的 compare 与硬件一致。
		s_last_compare_ticks[phase] = compare_ticks;
	}

	return ESP_OK;
}

esp_err_t motor_pwm_stop(void)
{
	if (!pwm_initialized)
	{
		ESP_LOGE(TAG, "PWM is not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	// Force LOW first, so stopping the counter cannot leave a phase HIGH.
	esp_err_t first_error = ESP_OK;
	for (int phase = 0; phase < 3; phase++)
	{
		esp_err_t result = mcpwm_generator_set_force_level(pwm_generators[phase], 0, true);
		if (result != ESP_OK && first_error == ESP_OK)
		{
			first_error = result;
		}
	}

	if (pwm_running)
	{
		esp_err_t result = mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_STOP_EMPTY);
		if (result != ESP_OK && first_error == ESP_OK)
		{
			first_error = result;
		}
	}

	pwm_running = false;
	if (first_error != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to stop PWM safely: %s", esp_err_to_name(first_error));
		return first_error;
	}

	ESP_LOGI(TAG, "PWM stopped, all phases forced LOW");
	return ESP_OK;
}
/*
 * 等待三相低侧共同导通窗口：count > max(compare)，以计数器 TOP 为中心。
 * 返回 ESP_OK 后立刻读 ADC。timeout_us 给 200（约 4 个 PWM 周期）足够。
 *
 * 几个事实（已对照 IDF 5.5.2 源码确认）：
 * - UP_DOWN 模式下硬件计数器峰值 = period_ticks / 2 = M1_PWM_COMPARE_MAX_TICKS = 250，
 *   计数范围 0..250，tick = 1/10MHz = 0.1us，PWM 周期 2*250 tick = 50us（20kHz）。
 * - 本项目只用 group 0 的第一个 timer，因此 timer_id 固定为 0。
 * - MCPWM0 是经典 ESP32 上唯一的 MCPWM 外设。
 */
esp_err_t motor_pwm_wait_lowside_window(uint32_t timeout_us)
{
	if (!pwm_running)
	{
		return ESP_ERR_INVALID_STATE;
	}

	uint32_t cmax = s_last_compare_ticks[0];
	if (s_last_compare_ticks[1] > cmax)
	{
		cmax = s_last_compare_ticks[1];
	}
	if (s_last_compare_ticks[2] > cmax)
	{
		cmax = s_last_compare_ticks[2];
	}

	// 在窗口入口（上行越过 cmax+5）返回，剩余可用时间约 2*(250-cmax-5)*0.1us。
	uint32_t threshold = cmax + 5;
	if (threshold > M1_PWM_COMPARE_MAX_TICKS - 5)
	{
		threshold = M1_PWM_COMPARE_MAX_TICKS - 5; // 防悬死
	}

	int64_t t0 = esp_timer_get_time();
	uint32_t loop_count = 0;
	for (;;)
	{
		uint32_t count = mcpwm_ll_timer_get_count_value(&MCPWM0, 0);
		if (count > threshold)
		{
			return ESP_OK;
		}
		// 每 16 次循环才查一次超时，避免 esp_timer 调用拖慢轮询。
		if ((++loop_count & 0x0F) == 0 &&
			esp_timer_get_time() - t0 > (int64_t)timeout_us)
		{
			return ESP_ERR_TIMEOUT;
		}
	}
}

// 返回 compare 最大的相：0=U 1=V 2=W（current_sense 用它决定读序）
int motor_pwm_max_compare_phase(void)
{
    uint32_t cmax = s_last_compare_ticks[0];
    int idx = 0;
    if (s_last_compare_ticks[1] > cmax) { cmax = s_last_compare_ticks[1]; idx = 1; }
    if (s_last_compare_ticks[2] > cmax) idx = 2;
    return idx;
}

// 调试用：读当前计数值（诊断 status 寄存器是否实时更新）
uint32_t motor_pwm_debug_count(void)
{
	return mcpwm_ll_timer_get_count_value(&MCPWM0, 0);
}

/*
 * 等计数器从下方越过 target（上升沿触发），相位扫描实验用。
 *
 * 不能直接判 count >= target：如果调用时计数器已经落在这个区间内，函数会
 * 立刻返回，起始相位完全不受控（target=75 时这个区间宽达 35us，占周期 70%）。
 * 必须先等它落到 target 以下，再等它重新涨上来，触发点才严格锁在上升沿的
 * target 处，精度约等于一个轮询周期（~0.2us）。
 */
esp_err_t motor_pwm_wait_count_rising(uint32_t target, uint32_t timeout_us)
{
	if (!pwm_running)
	{
		return ESP_ERR_INVALID_STATE;
	}

	// 只留极小的余量：计数器峰值恰好等于 COMPARE_MAX，目标太接近峰值时
	// 上升沿触发会不可靠。之前 clamp 到 COMPARE_MAX-10 在 20kHz 下吃掉了
	// 一半扫描档位（240~495 全部退化成 240），这里收紧到 -5。
	if (target < 2)
	{
		target = 2;
	}
	if (target > M1_PWM_COMPARE_MAX_TICKS - 5)
	{
		target = M1_PWM_COMPARE_MAX_TICKS - 5;
	}

	int64_t t0 = esp_timer_get_time();
	uint32_t loop_count = 0;

	// 阶段 1：先等计数器落到 target 以下
	for (;;)
	{
		if (mcpwm_ll_timer_get_count_value(&MCPWM0, 0) < target)
		{
			break;
		}
		if ((++loop_count & 0x0F) == 0 &&
			esp_timer_get_time() - t0 > (int64_t)timeout_us)
		{
			return ESP_ERR_TIMEOUT;
		}
	}

	// 阶段 2：再等它涨回 target，此刻即为触发点
	for (;;)
	{
		if (mcpwm_ll_timer_get_count_value(&MCPWM0, 0) >= target)
		{
			return ESP_OK;
		}
		if ((++loop_count & 0x0F) == 0 &&
			esp_timer_get_time() - t0 > (int64_t)timeout_us)
		{
			return ESP_ERR_TIMEOUT;
		}
	}
}
