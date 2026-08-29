#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "driver/gpio.h"

// M1三相PWM
#define M1_PWM_U_GPIO GPIO_NUM_26
#define M1_PWM_V_GPIO GPIO_NUM_27
#define M1_PWM_W_GPIO GPIO_NUM_14

#define M2_PWM_U_GPIO GPIO_NUM_32
#define M2_PWM_V_GPIO GPIO_NUM_33
#define M2_PWM_W_GPIO GPIO_NUM_25
// M1两相电流采样
#define M1_CURRENT_U_GPIO GPIO_NUM_36
#define M1_CURRENT_V_GPIO GPIO_NUM_39

// M1的AS5600
#define M1_SENSOR_SDA_GPIO GPIO_NUM_21
#define M1_SENSOR_SCL_GPIO GPIO_NUM_19
#define M1_AS5600_ADDRESS 0x36

// MCPWM计数器每秒计数1000万次
#define M1_PWM_TIMER_RESOLUTION_HZ 10000000UL

/*
 * 三相PWM频率。
 *
 * 10kHz 是低侧采样能可靠工作的前提：adc_oneshot_read 单次约 48.5us，采样
 * 瞬间落在调用后约 43~44us，20kHz 下低侧窗口只有 14us，实测 400 次采样命中
 * 率 0.75%（理论 28%），可用触发相位带宽仅 0.5us，工程上不可用。
 * 降到 10kHz 后窗口翻倍（duty 0.72 时 28us，duty 0.5 时 50us），
 * 可行触发相位带宽从 5 tick 扩大到约 200 tick。
 */
#define M1_PWM_FREQUENCY_HZ 20000UL

// 中心对齐PWM先向上计数，再向下计数
#define M1_PWM_PERIOD_TICKS       \
	(M1_PWM_TIMER_RESOLUTION_HZ / \
	 M1_PWM_FREQUENCY_HZ)

// 中心对齐模式下，比较器的峰值是完整周期的一半。
#define M1_PWM_COMPARE_MAX_TICKS (M1_PWM_PERIOD_TICKS / 2UL)

// EG2133采用自举高侧供电，暂时避免0%和100%
#define M1_PWM_MIN_DUTY 0.05f
#define M1_PWM_MAX_DUTY 0.95f

#define M1_MOTOR_POLE_PAIRS 7
#define M1_ELECTRICAL_ZERO_OFFSET_RAD 4.61f
// 电角度零点校准默认关闭，避免每次上电都给电机通电。
#define M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION 1

#define M1_ENABLE_FOC_CURRENT_LOOP 1
#define M1_ENABLE_SPEED_LOOP 0
#define M1_ENABLE_PWM_TRACE_TASK 0
#define M1_SPEED_REF_RAD_S 0.0f
#define M1_CURRENT_LOOP_TEST_ID_REF_A 0.05f
#define M1_CURRENT_LOOP_TEST_IQ_REF_A 0.05f
#define M1_CURRENT_LOOP_ID_PI_KP 1.5f
#define M1_CURRENT_LOOP_ID_PI_KI 8.0f
#define M1_CURRENT_LOOP_IQ_PI_KP 1.5f
#define M1_CURRENT_LOOP_IQ_PI_KI 8.0f
#define M1_ENABLE_TIMING_LOG 0
#define M1_ENABLE_CURRENT_TRACE 0
#define M1_ENABLE_CURRENT_TELEMETRY_TASK 1
#define M1_CURRENT_TRACE_INTERVAL_LOOPS 5000U
#define M1_ENABLE_FIXED_PWM_TEST 0
#define M1_FIXED_PWM_TEST_DUTY_U 0.25f
#define M1_FIXED_PWM_TEST_DUTY_V 0.75f
#define M1_FIXED_PWM_TEST_DUTY_W 0.50f
#define M1_SPEED_PI_KP 0.05f
#define M1_SPEED_PI_KI 0.5f
#define M1_SPEED_IQ_LIMIT_A 0.20f
#define M1_SPEED_FEEDBACK_SIGN 1.0f
#define M1_SPEED_LOOP_PERIOD_MS 10U

// 8V 母线、限流 0.15A～0.20A 时使用的低电压对齐占空比。
#define M1_ALIGNMENT_DUTY_U 0.60f
#define M1_ALIGNMENT_DUTY_V 0.40f
#define M1_ALIGNMENT_DUTY_W 0.40f

// 对齐保持时间和软件电流保护阈值。
#define M1_ALIGNMENT_TIME_MS 1000U
#define M1_ALIGNMENT_MAX_CURRENT_A 5.20f

#endif // !MOTOR_CONFIG_H
