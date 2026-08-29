#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include "esp_err.h"

/**
 * @brief 将三相电压指令转换为三相 PWM 占空比。
 *
 * 该函数只负责计算占空比，不会启动 PWM，也不会直接操作 MOSFET。
 *
 * @param u_voltage_v U 相电压指令，单位 V
 * @param v_voltage_v V 相电压指令，单位 V
 * @param w_voltage_v W 相电压指令，单位 V
 * @param bus_voltage_v 直流母线电压，单位 V
 * @param duty_u 输出 U 相占空比，范围 0.0f～1.0f
 * @param duty_v 输出 V 相占空比，范围 0.0f～1.0f
 * @param duty_w 输出 W 相占空比，范围 0.0f～1.0f
 */
esp_err_t foc_svpwm_calculate(
	float u_voltage_v,
	float v_voltage_v,
	float w_voltage_v,
	float bus_voltage_v,
	float *duty_u,
	float *duty_v,
	float *duty_w);

#endif // FOC_SVPWM_H
