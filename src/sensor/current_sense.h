#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * M1 measures U and V current with ADC1:
 *   U = GPIO36 / ADC1_CH0
 *   V = GPIO39 / ADC1_CH3
 *
 * W is reconstructed by the three-phase current sum:
 *   iw = -(iu + iv)
 */
typedef struct
{
    uint32_t sequence;
    int64_t timestamp_us;
    int iu_raw;
    int iv_raw;
    float iu_a;
    float iv_a;
} current_sense_frame_t;

esp_err_t current_sense_init(void);
esp_err_t current_sense_calibrate(void);

/*
 * Read the most recent complete ADC DMA frame without waiting for a new
 * conversion and without touching PWM timing.
 */
esp_err_t current_sense_read_latest_frame(current_sense_frame_t *frame);

esp_err_t current_sense_read_three_phase(
    float *iu_a,
    float *iv_a,
    float *iw_a);
esp_err_t current_sense_read_three_phase_with_timestamp(
    float *iu_a,
    float *iv_a,
    float *iw_a,
    int64_t *timestamp_us);
/* Read the newest complete frame whose timestamp is not later than the target time. */
esp_err_t current_sense_read_three_phase_at_or_before_timestamp(
    float *iu_a,
    float *iv_a,
    float *iw_a,
    int64_t target_timestamp_us,
    int64_t *timestamp_us,
    uint32_t *sequence);
esp_err_t current_sense_read_three_phase_with_timestamp_and_sequence(
    float *iu_a,
    float *iv_a,
    float *iw_a,
    int64_t *timestamp_us,
    uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSE_H */
