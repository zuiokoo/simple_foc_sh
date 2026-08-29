#ifndef FOC_SPEED_PI_H
#define FOC_SPEED_PI_H

#include "esp_err.h"
#include "foc_pi.h"

/**
 * @brief Outer speed-loop PI controller.
 *
 * The input and output units are mechanical rad/s and q-axis current A.
 */
typedef struct
{
	foc_pi_controller_t pi;
	float speed_ref_rad_s;
	float measured_speed_rad_s;
	float iq_ref_a;
} foc_speed_pi_t;

void foc_speed_pi_init(
	foc_speed_pi_t *controller,
	float kp,
	float ki,
	float iq_min_a,
	float iq_max_a);

void foc_speed_pi_reset(foc_speed_pi_t *controller);

esp_err_t foc_speed_pi_update(
	foc_speed_pi_t *controller,
	float speed_ref_rad_s,
	float measured_speed_rad_s,
	float dt_s,
	float *iq_ref_a);

#endif
