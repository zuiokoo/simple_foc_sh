#include "foc_open_loop.h"
#include <math.h>
#include <stddef.h>
#define FOC_TWO_PI 6.28318530717958647692f
// 把角度限制到0～2π范围。
static float foc_open_loop_wrap_angle(float angle_rad)
{
	angle_rad = fmodf(angle_rad, FOC_TWO_PI);

	if (angle_rad < 0.0f)
	{
		angle_rad += FOC_TWO_PI;
	}

	return angle_rad;
}
void foc_open_loop_init(foc_open_loop_t *generator, float initial_angle_rad)
{
	if (generator == NULL)
	{
		return;
	}
	generator->electrical_angle_rad = foc_open_loop_wrap_angle(initial_angle_rad);
}

esp_err_t foc_open_loop_step(foc_open_loop_t *generator, float electrical_speed_rad_s, float dt_s, float *electrical_angle_rad)
{
	if (generator == NULL || electrical_angle_rad == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}
	if (!isfinite(electrical_speed_rad_s) ||
		!isfinite(dt_s) ||
		dt_s <= 0.0f)
	{
		return ESP_ERR_INVALID_ARG;
	}
	// 角度变化量 = 角速度 × 时间。
	generator->electrical_angle_rad += electrical_speed_rad_s * dt_s;
	// 防止角度不断增大。
	generator->electrical_angle_rad = foc_open_loop_wrap_angle(generator->electrical_angle_rad);
	*electrical_angle_rad = generator->electrical_angle_rad;

	return ESP_OK;
}