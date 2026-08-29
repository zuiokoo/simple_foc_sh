#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "esp_err.h"

/**
 * @brief 使用 U/V 两相电流执行 Clarke 变换。
 *
 * 当前电流采样硬件只直接测量 U、V 两相，因此使用两电阻采样常用的
 * 简化公式，输出静止坐标系中的 i_alpha 和 i_beta，单位为 A。
 */
esp_err_t foc_clarke_transform(
	float iu_a,
	float iv_a,
	float *i_alpha_a,
	float *i_beta_a);

/**
 * @brief 将 alpha-beta 静止坐标系电流转换为 d-q 旋转坐标系电流。
 *
 * @param i_alpha_a alpha 轴电流，单位 A
 * @param i_beta_a  beta 轴电流，单位 A
 * @param electrical_angle_rad 转子电角度，单位 rad
 * @param i_d_a 输出的 d 轴电流，单位 A
 * @param i_q_a 输出的 q 轴电流，单位 A
 */
esp_err_t foc_park_transform(
	float i_alpha_a,
	float i_beta_a,
	float electrical_angle_rad,
	float *i_d_a,
	float *i_q_a);

/**
 * @brief 将 d-q 旋转坐标系电压转换回 alpha-beta 静止坐标系电压。
 *
 * @param v_d_v d 轴电压指令，单位 V
 * @param v_q_v q 轴电压指令，单位 V
 * @param electrical_angle_rad 转子电角度，单位 rad
 * @param v_alpha_v 输出的 alpha 轴电压指令，单位 V
 * @param v_beta_v 输出的 beta 轴电压指令，单位 V
 */
esp_err_t foc_inverse_park_transform(
	float v_d_v,
	float v_q_v,
	float electrical_angle_rad,
	float *v_alpha_v,
	float *v_beta_v);

/**
 * @brief 将 alpha-beta 静止坐标系电压转换为三相电压指令。
 *
 * 这是逆 Clarke 变换，不是 SVPWM。
 *
 * @param v_alpha_v alpha 轴电压，单位 V
 * @param v_beta_v  beta 轴电压，单位 V
 * @param u_voltage_v 输出 U 相电压，单位 V
 * @param v_voltage_v 输出 V 相电压，单位 V
 * @param w_voltage_v 输出 W 相电压，单位 V
 */
esp_err_t foc_inverse_clarke_transform(
	float v_alpha_v,
	float v_beta_v,
	float *u_voltage_v,
	float *v_voltage_v,
	float *w_voltage_v);

/**
 * @brief 将机械角度转换为电角度。
 *
 * 电角度 = 机械角度 × 极对数 - 电角度零点偏移。
 * 返回值范围为 0 到 2π，单位为 rad。
 */
float foc_mechanical_to_electrical_angle(
	float mechanical_angle_rad,
	int pole_pairs,
	float electrical_zero_offset_rad);

#endif // FOC_MATH_H
