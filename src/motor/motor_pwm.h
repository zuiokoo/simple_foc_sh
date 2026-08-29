
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "esp_err.h"

esp_err_t motor_pwm_init(void);
esp_err_t motor_pwm_set_duty(float duty_u, float duty_v, float duty_w);
esp_err_t motor_pwm_start(void);
esp_err_t motor_pwm_stop(void);
esp_err_t motor_pwm_wait_lowside_window(uint32_t timeout_us);
int motor_pwm_max_compare_phase(void);

/**
 * @brief 调试用：直接读取 MCPWM 定时器当前计数值（0..250）。
 *
 * 用于诊断计数器寄存器是否实时更新（分段计时测试用）。
 */
uint32_t motor_pwm_debug_count(void);

/**
 * @brief 调试用：等计数器从下方越过 target（上升沿触发）。
 *
 * 触发点严格锁在上升沿的 target 处，精度约一个轮询周期（~0.2us）。若直接
 * 判断 count >= target，调用时计数器可能已落在该区间内而立即返回，起始相位
 * 将完全不受控（target=75 时该区间宽达 35us，占周期 70%）。
 *
 * 用于相位扫描实验：在确定的相位启动 ADC，测出采样瞬间相对低侧窗口的位置。
 */
esp_err_t motor_pwm_wait_count_rising(uint32_t target, uint32_t timeout_us);
#endif // MOTOR_PWM_H
