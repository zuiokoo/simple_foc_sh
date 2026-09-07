#ifndef FOC_ANGLE_ESTIMATOR_H
#define FOC_ANGLE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * AS5600 is a relatively slow I2C sensor.  The current loop must not wait for
 * it; it uses this state to predict the rotor angle at the ADC/PWM timestamps.
 */
typedef struct
{
    float angle_rad;
    float velocity_rad_s;
    int64_t sample_timestamp_us;
    bool valid;
    bool velocity_initialized;
} foc_angle_estimator_t;

void foc_angle_estimator_init(foc_angle_estimator_t *estimator);

/* Publish one completed AS5600 measurement at its midpoint timestamp. */
void foc_angle_estimator_sample(
    foc_angle_estimator_t *estimator,
    float angle_rad,
    float velocity_rad_s,
    int64_t sample_timestamp_us);

void foc_angle_estimator_invalidate(foc_angle_estimator_t *estimator);

/* Predict angle/velocity at an arbitrary timestamp without blocking. */
bool foc_angle_estimator_predict(
    const foc_angle_estimator_t *estimator,
    int64_t target_timestamp_us,
    float *angle_rad,
    float *velocity_rad_s,
    uint32_t *age_us);

#endif
