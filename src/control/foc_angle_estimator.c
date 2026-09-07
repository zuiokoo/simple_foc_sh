#include "foc_angle_estimator.h"

#include <math.h>
#include <stddef.h>

static const float FOC_PI = 3.14159265358979323846f;
static const float FOC_TWO_PI = 6.28318530717958647692f;

static float wrap_angle(float angle_rad)
{
    while (angle_rad >= FOC_TWO_PI)
    {
        angle_rad -= FOC_TWO_PI;
    }
    while (angle_rad < 0.0f)
    {
        angle_rad += FOC_TWO_PI;
    }
    return angle_rad;
}

static float wrap_delta(float delta_angle_rad)
{
    if (delta_angle_rad > 3.14159265358979323846f)
    {
        delta_angle_rad -= FOC_TWO_PI;
    }
    else if (delta_angle_rad < -FOC_PI)
    {
        delta_angle_rad += FOC_TWO_PI;
    }
    return delta_angle_rad;
}

void foc_angle_estimator_init(foc_angle_estimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    estimator->angle_rad = 0.0f;
    estimator->velocity_rad_s = 0.0f;
    estimator->sample_timestamp_us = 0;
    estimator->valid = false;
    estimator->velocity_initialized = false;
}

void foc_angle_estimator_sample(
    foc_angle_estimator_t *estimator,
    float angle_rad,
    float velocity_rad_s,
    int64_t sample_timestamp_us)
{
    if (estimator == NULL || !isfinite(angle_rad) ||
        !isfinite(velocity_rad_s) || sample_timestamp_us <= 0)
    {
        return;
    }

    angle_rad = wrap_angle(angle_rad);
    if (estimator->valid && estimator->sample_timestamp_us > 0 &&
        sample_timestamp_us > estimator->sample_timestamp_us)
    {
        float dt_s = (float)(sample_timestamp_us -
            estimator->sample_timestamp_us) / 1000000.0f;
        if (dt_s > 0.0f && dt_s < 0.1f)
        {
            float delta_angle_rad = wrap_delta(angle_rad - estimator->angle_rad);
            float measured_velocity_rad_s = delta_angle_rad / dt_s;

            /*
             * AS5600 already supplies a filtered velocity.  Blend the
             * unwrap-derived velocity with it so an I2C sample cadence jump
             * cannot create a single violent prediction step.
             */
            float velocity_from_angle_rad_s =
                0.35f * measured_velocity_rad_s + 0.65f * velocity_rad_s;
            estimator->velocity_rad_s = estimator->velocity_initialized
                ? 0.25f * estimator->velocity_rad_s +
                    0.75f * velocity_from_angle_rad_s
                : velocity_from_angle_rad_s;
            estimator->velocity_initialized = true;
        }
    }

    if (!estimator->valid)
    {
        estimator->velocity_rad_s = velocity_rad_s;
        estimator->velocity_initialized = true;
    }

    estimator->angle_rad = angle_rad;
    estimator->sample_timestamp_us = sample_timestamp_us;
    estimator->valid = true;
}

void foc_angle_estimator_invalidate(foc_angle_estimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    estimator->valid = false;
    estimator->velocity_initialized = false;
    estimator->velocity_rad_s = 0.0f;
}

bool foc_angle_estimator_predict(
    const foc_angle_estimator_t *estimator,
    int64_t target_timestamp_us,
    float *angle_rad,
    float *velocity_rad_s,
    uint32_t *age_us)
{
    if (estimator == NULL || angle_rad == NULL || velocity_rad_s == NULL ||
        !estimator->valid || estimator->sample_timestamp_us <= 0 ||
        target_timestamp_us == 0)
    {
        return false;
    }

    int64_t delta_us = target_timestamp_us - estimator->sample_timestamp_us;
    float delta_s = (float)delta_us / 1000000.0f;
    *angle_rad = wrap_angle(estimator->angle_rad +
        estimator->velocity_rad_s * delta_s);
    *velocity_rad_s = estimator->velocity_rad_s;
    if (age_us != NULL)
    {
        *age_us = delta_us <= 0 ? 0U : (delta_us > UINT32_MAX ? UINT32_MAX : (uint32_t)delta_us);
    }
    return true;
}
