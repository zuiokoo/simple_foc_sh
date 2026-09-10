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
	controller->id_loop_enabled = true;
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

void foc_controller_set_id_loop_enabled(
	foc_controller_t *controller,
	bool enabled)
{
	if (controller == NULL)
	{
		return;
	}

	controller->id_loop_enabled = enabled;
	if (!enabled)
	{
		foc_pi_reset(&controller->id_pi);
	}
}

void foc_controller_set_pi_gains(
    foc_controller_t *controller,
    float id_kp,
    float id_ki,
    float iq_kp,
    float iq_ki)
{
    if (controller == NULL ||
        !isfinite(id_kp) || !isfinite(id_ki) ||
        !isfinite(iq_kp) || !isfinite(iq_ki) ||
        id_kp < 0.0f || id_ki < 0.0f ||
        iq_kp < 0.0f || iq_ki < 0.0f)
    {
        return;
    }

    if (controller->id_pi.kp == id_kp &&
        controller->id_pi.ki == id_ki &&
        controller->iq_pi.kp == iq_kp &&
        controller->iq_pi.ki == iq_ki)
    {
        return;
    }

    controller->id_pi.kp = id_kp;
    controller->id_pi.ki = id_ki;
    controller->iq_pi.kp = iq_kp;
    controller->iq_pi.ki = iq_ki;
    foc_controller_reset(controller);
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
		!isfinite(input->output_electrical_angle_rad) ||
		!isfinite(input->id_target_a) ||
		!isfinite(input->iq_target_a) ||
		!isfinite(input->dt_s) ||
		!isfinite(input->bus_voltage_v) ||
		!isfinite(input->electrical_velocity_rad_s) ||
		!isfinite(input->motor_phase_resistance_ohm) ||
		!isfinite(input->motor_inductance_d_h) ||
		!isfinite(input->motor_inductance_q_h) ||
		!isfinite(input->motor_flux_linkage_wb))
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (input->dt_s <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}
		if (input->bus_voltage_v <= 0.0f ||
		input->motor_phase_resistance_ohm < 0.0f ||
		input->motor_inductance_d_h < 0.0f ||
		input->motor_inductance_q_h < 0.0f ||
		input->motor_flux_linkage_wb < 0.0f)
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
	output->id_error_a = input->id_target_a - output->i_d_a;
	output->iq_error_a = input->iq_target_a - output->i_q_a;

	float vd_pi_v = 0.0f;
	if (controller->id_loop_enabled)
	{
		vd_pi_v = foc_pi_update(&controller->id_pi, output->id_error_a, input->dt_s);
	}
	else
	{
		foc_pi_reset(&controller->id_pi);
	}
	float vq_pi_v = foc_pi_update(&controller->iq_pi, output->iq_error_a, input->dt_s);

	/*
	 * PMSM dq model in the Park convention used above:
	 *   vd_ff = Rs*Id - omega_e*Lq*Iq
	 *   vq_ff = Rs*Iq + omega_e*(Ld*Id + flux)
	 * The PI then only has to correct parameter error and transients.
	 */
	output->vd_decoupling_v =
		input->motor_phase_resistance_ohm * output->i_d_a -
		input->electrical_velocity_rad_s *
			input->motor_inductance_q_h * output->i_q_a;
	output->vq_decoupling_v =
		input->motor_phase_resistance_ohm * output->i_q_a +
		input->electrical_velocity_rad_s *
			(input->motor_inductance_d_h * output->i_d_a +
			 input->motor_flux_linkage_wb);

	output->vd_v = vd_pi_v + output->vd_decoupling_v;
	output->vq_v = vq_pi_v + output->vq_decoupling_v;

	/*
	 * The phase-duty implementation reserves 5%% at both ends.  Limit the
	 * dq vector to the corresponding linear SVPWM hexagon radius before
	 * inverse Park, so the three phases cannot silently overmodulate.
	 */
	output->voltage_vector_limit_v =
		input->bus_voltage_v * 0.90f / 1.7320508075688772f;
	float voltage_magnitude_v =
		sqrtf(output->vd_v * output->vd_v + output->vq_v * output->vq_v);
	if (voltage_magnitude_v > output->voltage_vector_limit_v &&
		voltage_magnitude_v > 0.0f)
	{
		float vd_unlimited_v = output->vd_v;
		float vq_unlimited_v = output->vq_v;
		float scale = output->voltage_vector_limit_v / voltage_magnitude_v;
		output->vd_v *= scale;
		output->vq_v *= scale;

		/*
		 * Vector saturation is shared by d and q.  Back-calculate the
		 * rejected vector into both PI integrators; otherwise one axis can
		 * keep winding up while the other axis consumes the voltage limit.
		 */
		float vd_rejected_v = output->vd_v - vd_unlimited_v;
		float vq_rejected_v = output->vq_v - vq_unlimited_v;
		if (controller->id_loop_enabled)
		{
			controller->id_pi.integral += vd_rejected_v;
		}
		else
		{
			controller->id_pi.integral = 0.0f;
		}
		controller->iq_pi.integral += vq_rejected_v;
		if (controller->id_loop_enabled &&
			controller->id_pi.integral < controller->id_pi.output_min)
		{
			controller->id_pi.integral = controller->id_pi.output_min;
		}
	else if (controller->id_loop_enabled &&
			controller->id_pi.integral > controller->id_pi.output_max)
		{
			controller->id_pi.integral = controller->id_pi.output_max;
		}
		if (controller->iq_pi.integral < controller->iq_pi.output_min)
		{
			controller->iq_pi.integral = controller->iq_pi.output_min;
		}
		else if (controller->iq_pi.integral > controller->iq_pi.output_max)
		{
			controller->iq_pi.integral = controller->iq_pi.output_max;
		}
	}
	result = foc_inverse_park_transform(output->vd_v, output->vq_v, input->output_electrical_angle_rad, &output->v_alpha_v, &output->v_beta_v);
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
