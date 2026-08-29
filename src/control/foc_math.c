#include "foc_math.h"

#include <math.h>

esp_err_t foc_clarke_transform(
	float iu_a,
	float iv_a,
	float *i_alpha_a,
	float *i_beta_a)
{
	if (i_alpha_a == NULL || i_beta_a == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * Clarke 变换把三相静止坐标系转换为两相静止坐标系：
	 *
	 * i_alpha = iu
	 * i_beta  = (iu + 2 * iv) / sqrt(3)
	 *
	 * 该简化公式利用了三相无中性线电流满足：
	 * iu + iv + iw = 0。
	 */
	*i_alpha_a = iu_a;
	*i_beta_a = (iu_a + 2.0f * iv_a) / sqrtf(3.0f);

	return ESP_OK;
}
esp_err_t foc_park_transform(
	float i_alpha_a,
	float i_beta_a,
	float electrical_angle_rad,
	float *i_d_a,
	float *i_q_a)
{
	if (i_d_a == NULL || i_q_a == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	// 计算电角度的正弦和余弦。
	float sin_angle = sinf(electrical_angle_rad);
	float cos_angle = cosf(electrical_angle_rad);
	/*
	 * Park 变换：
	 *
	 * i_d = i_alpha * cos(theta)
	 *     + i_beta  * sin(theta)
	 *
	 * i_q = -i_alpha * sin(theta)
	 *     + i_beta  * cos(theta)
	 */
	*i_d_a = i_alpha_a * cos_angle + i_beta_a * sin_angle;
	*i_q_a = -i_alpha_a * sin_angle + i_beta_a * cos_angle;
	return ESP_OK;
}

esp_err_t foc_inverse_park_transform(
	float v_d_v,
	float v_q_v,
	float electrical_angle_rad,
	float *v_alpha_v,
	float *v_beta_v)
{
	if (v_alpha_v == NULL || v_beta_v == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	// 计算当前电角度的正弦和余弦。
	float sin_angle = sinf(electrical_angle_rad);
	float cos_angle = cosf(electrical_angle_rad);

	/*
	 * 逆 Park 变换：
	 *
	 * v_alpha = v_d * cos(theta) - v_q * sin(theta)
	 * v_beta  = v_d * sin(theta) + v_q * cos(theta)
	 *
	 * 这一步把旋转坐标系电压转换回静止坐标系，
	 * 后面的 SVPWM 将使用 v_alpha 和 v_beta。
	 */
	*v_alpha_v = v_d_v * cos_angle - v_q_v * sin_angle;
	*v_beta_v = v_d_v * sin_angle + v_q_v * cos_angle;

	return ESP_OK;
}
esp_err_t foc_inverse_clarke_transform(
	float v_alpha_v,
	float v_beta_v,
	float *u_voltage_v,
	float *v_voltage_v,
	float *w_voltage_v)
{

	if (u_voltage_v == NULL || v_voltage_v == NULL || w_voltage_v == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}


	const float sqrt_3_half=0.8660254f;
	/*
	 * 逆 Clarke 变换：
	 *
	 * u = v_alpha
	 * v = -0.5 * v_alpha + sqrt(3)/2 * v_beta
	 * w = -0.5 * v_alpha - sqrt(3)/2 * v_beta
	 *
	 * 这一步把静止坐标系电压转换为三相电压指令，
	 * 后面的 SVPWM 将使用 u、v、w。
	 */
	*u_voltage_v=v_alpha_v;
	*v_voltage_v=-0.5f*v_alpha_v+sqrt_3_half*v_beta_v;
	*w_voltage_v=-0.5f*v_alpha_v-sqrt_3_half*v_beta_v;
	return ESP_OK;
}
float foc_mechanical_to_electrical_angle(
	float mechanical_angle_rad,
	int pole_pairs,
	float electrical_zero_offset_rad)
{
	// 一整圈角度，单位为 rad。
	const float two_pi = 2.0f * 3.14159265358979323846f;
	/*
	 * 电角度关系：
	 *
	 * 电角度 = 机械角度 × 极对数 - 电角度零点偏移
	 */
	float electrical_angle_rad = mechanical_angle_rad * pole_pairs - electrical_zero_offset_rad;
	/*
	 * 将电角度限制到 0 ~ 2π。
	 * fmodf() 可以取得浮点数除法的余数。
	 */
	electrical_angle_rad = fmodf(electrical_angle_rad, two_pi);
	// fmodf() 对负数可能返回负数，所以补回一整圈。
	if (electrical_angle_rad < 0.0f)
	{
		electrical_angle_rad += two_pi;
	}
	return electrical_angle_rad;
}
