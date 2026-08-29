#ifndef FOC_OPEN_LOOP_H
#define FOC_OPEN_LOOP_H
#include "esp_err.h"

typedef struct
{
	float electrical_angle_rad;
} foc_open_loop_t;

void foc_open_loop_init(foc_open_loop_t *generator, float initial_angle_rad);
esp_err_t foc_open_loop_step(foc_open_loop_t *generator, float electrical_speed_rad_s, float dt_s, float *electrical_angle_rad);

#endif