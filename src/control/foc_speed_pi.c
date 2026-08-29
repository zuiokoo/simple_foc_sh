#include "foc_speed_pi.h"

#include <math.h>
#include <stddef.h>

void foc_speed_pi_init(
	foc_speed_pi_t *controller,
	float kp,
	float ki,
	float iq_min_a,
	float iq_max_a)
{
	if (controller == NULL)
	{
		return;
	}

	foc_pi_init(&controller->pi, kp, ki, iq_min_a, iq_max_a);
	controller->speed_ref_rad_s = 0.0f;
	controller->measured_speed_rad_s = 0.0f;
	controller->iq_ref_a = 0.0f;
}

void foc_speed_pi_reset(foc_speed_pi_t *controller)
{
	if (controller == NULL)
	{
		return;
	}

	foc_pi_reset(&controller->pi);
	controller->speed_ref_rad_s = 0.0f;
	controller->measured_speed_rad_s = 0.0f;
	controller->iq_ref_a = 0.0f;
}

esp_err_t foc_speed_pi_update(
	foc_speed_pi_t *controller,
	float speed_ref_rad_s,
	float measured_speed_rad_s,
	float dt_s,
	float *iq_ref_a)
{
	if (controller == NULL || iq_ref_a == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	if (!isfinite(speed_ref_rad_s) ||
		!isfinite(measured_speed_rad_s) ||
		!isfinite(dt_s) ||
		dt_s <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}

	float speed_error_rad_s = speed_ref_rad_s - measured_speed_rad_s;
	float iq_command_a = foc_pi_update(
		&controller->pi,
		speed_error_rad_s,
		dt_s);

	controller->speed_ref_rad_s = speed_ref_rad_s;
	controller->measured_speed_rad_s = measured_speed_rad_s;
	controller->iq_ref_a = iq_command_a;
	*iq_ref_a = iq_command_a;

	return ESP_OK;
}
