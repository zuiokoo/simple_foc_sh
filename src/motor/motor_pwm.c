#include "motor_pwm.h"
#include "motor_config.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
// IDF 5.5 的 mcpwm prelude 驱动没有公开读取计数器的 API，
// 同步采样只能通过 HAL/LL 层直接读 MCPWM0 的计数器。



// 记录三相最近一次写入的 compare 值（tick），供采样同步用
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

// The current loop is notified from the MCPWM full event. One notification
// is emitted every M1_CURRENT_LOOP_PWM_PERIODS 20 kHz PWM periods.
static TaskHandle_t s_control_task = NULL;
static volatile uint32_t s_pwm_full_events = 0;
static portMUX_TYPE s_pwm_timestamp_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_last_control_tick_timestamp_us = 0;

static bool IRAM_ATTR motor_pwm_on_full(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    int64_t timestamp_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_pwm_timestamp_lock);
    s_last_control_tick_timestamp_us = timestamp_us;
    portEXIT_CRITICAL_ISR(&s_pwm_timestamp_lock);

    s_pwm_full_events++;
    if (s_control_task == NULL ||
        (s_pwm_full_events % M1_CURRENT_LOOP_PWM_PERIODS) != 0U)
    {
        return false;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

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
			M1_PWM_COMPARE_MAX_TICKS / 2U);
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

		mcpwm_timer_event_callbacks_t timer_callbacks = {
		/* MCPWM_TIMER_EVENT_FULL is the center of up-down PWM. */
        .on_full = motor_pwm_on_full,
	};
	result = mcpwm_timer_register_event_callbacks(pwm_timer, &timer_callbacks, NULL);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register MCPWM timer callbacks: %s", esp_err_to_name(result));
		return result;
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

esp_err_t motor_pwm_register_control_task(TaskHandle_t task)
{
    if (task == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (pwm_running)
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_control_task = task;
    s_pwm_full_events = 0;
    portENTER_CRITICAL(&s_pwm_timestamp_lock);
    s_last_control_tick_timestamp_us = 0;
    portEXIT_CRITICAL(&s_pwm_timestamp_lock);
    return ESP_OK;
}


int64_t motor_pwm_get_control_tick_timestamp_us(void)
{
    int64_t timestamp_us;
    portENTER_CRITICAL(&s_pwm_timestamp_lock);
    timestamp_us = s_last_control_tick_timestamp_us;
    portEXIT_CRITICAL(&s_pwm_timestamp_lock);
    return timestamp_us;
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
