#ifndef AS5600_H
#define AS5600_H

#include "esp_err.h"
#include "stdint.h"

esp_err_t as5600_init(void);
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle);
float as5600_raw_to_mechanical_angle(uint16_t raw_angle);
esp_err_t as5600_get_mechanical_angle(float*angle_rad);//机械角度
esp_err_t as5600_measure_angle_velocity(float *angle_rad, float *velocity_rad_s);


#endif // AS5600_H
