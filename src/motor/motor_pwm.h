#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Runtime PWM API.
 *
 * M1 hardware mapping is fixed:
 *   U = GPIO26, V = GPIO27, W = GPIO14
 */
esp_err_t motor_pwm_init(void);
esp_err_t motor_pwm_set_duty(float duty_u, float duty_v, float duty_w);
esp_err_t motor_pwm_start(void);
esp_err_t motor_pwm_stop(void);

/*
 * Real-time current-loop tick.
 *
 * The MCPWM timer ISR emits one task notification every four 20 kHz PWM
 * periods. This produces the 5 kHz current-loop wakeup
 * during current-loop bring-up. Register the consumer before motor_pwm_start().
 */
esp_err_t motor_pwm_register_control_task(TaskHandle_t task);

/* Timestamp of the latest PWM center event that generated a control tick. */
uint32_t motor_pwm_get_control_tick_timestamp_us(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
