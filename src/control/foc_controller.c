#include "foc_controller.h"

#include <stddef.h>
#include <math.h>
#include "foc_math.h"
#include "foc_svpwm.h"
/**
 * @brief 初始化 FOC 控制器。
 *
 * 初始化 d 轴和 q 轴两个独立的电流 PI。
 */
void foc_controller_init(
	foc_controller_t *controller,
	float id_kp,
	float id_ki,
	float iq_kp,
	float iq_ki,
	float voltage_min_v,
	float voltage_max_v)
{
	if (controller == NULL)
	{
		return;
	}
	foc_pi_init(&controller->id_pi, id_kp, id_ki, voltage_min_v, voltage_max_v);
	foc_pi_init(&controller->iq_pi, iq_kp, iq_ki, voltage_min_v, voltage_max_v);
}

void foc_controller_reset(foc_controller_t *controller)
{
	if (controller == NULL)
	{
		return;
	}

	foc_pi_reset(&controller->id_pi);
	foc_pi_reset(&controller->iq_pi);
}

esp_err_t foc_controller_step(foc_controller_t *controller, const foc_controller_input_t *input, foc_controller_output_t *output)
{
	if (controller == NULL || input == NULL || output == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (!isfinite(input->iu_a) ||
		!isfinite(input->iv_a) ||
		!isfinite(input->iw_a) ||
		!isfinite(input->electrical_angle_rad) ||
		!isfinite(input->id_ref_a) ||
		!isfinite(input->iq_ref_a) ||
		!isfinite(input->dt_s) ||
		!isfinite(input->bus_voltage_v))
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (input->dt_s <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (input->bus_voltage_v <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}
	esp_err_t result = foc_clarke_transform(input->iu_a, input->iv_a, &output->i_alpha_a, &output->i_beta_a);
	if (result != ESP_OK)
	{
		return result;
	}
	result = foc_park_transform(output->i_alpha_a, output->i_beta_a, input->electrical_angle_rad, &output->i_d_a, &output->i_q_a);
	if (result != ESP_OK)
	{
		return result;
	}
	output->id_error_a = input->id_ref_a - output->i_d_a;
	output->iq_error_a = input->iq_ref_a - output->i_q_a;

	output->vd_v = foc_pi_update(&controller->id_pi, output->id_error_a, input->dt_s);
	output->vq_v = foc_pi_update(&controller->iq_pi, output->iq_error_a, input->dt_s);
	result = foc_inverse_park_transform(output->vd_v, output->vq_v, input->electrical_angle_rad, &output->v_alpha_v, &output->v_beta_v);
	if (result != ESP_OK)
	{
		return result;
	}
	result = foc_inverse_clarke_transform(output->v_alpha_v, output->v_beta_v, &output->u_voltage_v, &output->v_voltage_v, &output->w_voltage_v);
	if (result != ESP_OK)
	{
		return result;
	}
	result = foc_svpwm_calculate(output->u_voltage_v, output->v_voltage_v, output->w_voltage_v, input->bus_voltage_v, &output->duty_u, &output->duty_v, &output->duty_w);
	if (result != ESP_OK)
	{
		return result;
	}

	// Implement FOC control logic here
	return ESP_OK;
}
