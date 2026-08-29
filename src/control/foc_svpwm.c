#include "foc_svpwm.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 把占空比限制在0.0～1.0范围内。
 */
static float foc_svpwm_clamp_duty(float duty)
{
	if (duty < 0.0f)
	{
		return 0.0f;
	}
	if (duty > 1.0f)
	{
		return 1.0f;
	}
	return duty;
}

esp_err_t foc_svpwm_calculate(
	float u_voltage_v,
	float v_voltage_v,
	float w_voltage_v,
	float bus_voltage_v,
	float *duty_u,
	float *duty_v,
	float *duty_w)
{
	// 三个输出都必须有效，否则无法返回计算结果。
	if (duty_u == NULL || duty_v == NULL || duty_w == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (!isfinite(u_voltage_v) ||
		!isfinite(v_voltage_v) ||
		!isfinite(w_voltage_v) ||
		!isfinite(bus_voltage_v))
	{
		return ESP_ERR_INVALID_ARG;
	}
	// 母线电压必须大于0，否则不能进行电压到占空比的换算。
	if (bus_voltage_v <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * 找出三相电压指令中的最大值和最小值。
	 * 后面通过公共偏移，把三相电压整体移动到PWM可输出范围内。
	 */
	float v_max = fmaxf(u_voltage_v, fmaxf(v_voltage_v, w_voltage_v));
	float v_min = fminf(u_voltage_v, fminf(v_voltage_v, w_voltage_v));

	/*
	 * 公共偏移量：
	 *
	 * offset = (最大相电压 + 最小相电压) / 2
	 *
	 * 三相同时减去这个偏移量，不会改变线电压，
	 * 但可以让最大、最小两相尽量对称地落在母线范围内。
	 */
	float offset = 0.5f * (v_max + v_min);

	float u_centered_voltage = u_voltage_v - offset;
	float v_centered_voltage = v_voltage_v - offset;
	float w_centered_voltage = w_voltage_v - offset;

	/*
	 * 以50%占空比作为零电压中心：
	 *
	 * duty = 0.5 + 相电压 / 母线电压
	 *
	 * 这里的电压是经过公共偏移后的相电压。
	 */
	*duty_u = foc_svpwm_clamp_duty(
		0.5f + u_centered_voltage / bus_voltage_v);
	*duty_v = foc_svpwm_clamp_duty(
		0.5f + v_centered_voltage / bus_voltage_v);
	*duty_w = foc_svpwm_clamp_duty(
		0.5f + w_centered_voltage / bus_voltage_v);

	return ESP_OK;
}
