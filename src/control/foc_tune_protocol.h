#ifndef FOC_TUNE_PROTOCOL_H
#define FOC_TUNE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TUNE_MIN_TARGET_A (-0.5f)
#define FOC_TUNE_MAX_TARGET_A (0.5f)
#define FOC_TUNE_DEFAULT_TELEMETRY_RATE_HZ 20U
#define FOC_TUNE_MIN_TELEMETRY_RATE_HZ 1U
#define FOC_TUNE_MAX_TELEMETRY_RATE_HZ 100U

#define FOC_TUNE_PI_KP_MIN 0.0f
#define FOC_TUNE_PI_KP_MAX 10.0f
#define FOC_TUNE_PI_KI_MIN 0.0f
#define FOC_TUNE_PI_KI_MAX 50.0f

typedef struct
{
    float id_kp;
    float id_ki;
    float iq_kp;
    float iq_ki;
} foc_tune_pi_config_t;

esp_err_t foc_tune_protocol_start(void);
float foc_tune_get_id_target_a(void);
float foc_tune_get_iq_target_a(void);
uint32_t foc_tune_get_telemetry_rate_hz(void);
void foc_tune_get_pi_config(foc_tune_pi_config_t *config, uint32_t *sequence);
uint32_t foc_tune_get_command_sequence(void);
uint32_t foc_tune_get_fault_code(void);
void foc_tune_set_fault_code(uint32_t fault_code);
bool foc_tune_take_alignment_request(void);
void foc_tune_complete_alignment(bool success, float applied_offset_rad);

#ifdef __cplusplus
}
#endif

#endif