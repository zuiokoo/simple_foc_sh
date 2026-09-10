/* Runtime FOC tuning command protocol. */
#include "foc_tune_protocol.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor/motor_config.h"

#define FOC_TUNE_LINE_MAX 128

static const char *TAG = "FOC_TUNE";
static volatile float foc_tune_id_target_a = M1_CURRENT_LOOP_TEST_ID_REF_A;
static volatile float foc_tune_iq_target_a = M1_CURRENT_LOOP_TEST_IQ_REF_A;
static volatile uint32_t foc_tune_telemetry_rate_hz = FOC_TUNE_DEFAULT_TELEMETRY_RATE_HZ;
static volatile uint32_t foc_tune_command_sequence = 0U;
static volatile uint32_t foc_tune_fault_code = 0U;
static volatile bool foc_tune_alignment_requested = false;
static volatile float foc_tune_id_kp = M1_CURRENT_LOOP_ID_PI_KP;
static volatile float foc_tune_id_ki = M1_CURRENT_LOOP_ID_PI_KI;
static volatile float foc_tune_iq_kp = M1_CURRENT_LOOP_IQ_PI_KP;
static volatile float foc_tune_iq_ki = M1_CURRENT_LOOP_IQ_PI_KI;
static volatile uint32_t foc_tune_pi_sequence = 0U;
static portMUX_TYPE foc_tune_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t foc_tune_task_handle = NULL;

static long foc_tune_to_milli(float value_a)
{
    return isfinite(value_a) ? lroundf(value_a * 1000.0f) : 0L;
}

static void foc_tune_reply(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    putchar('\n');
    fflush(stdout);
}

static bool foc_tune_parse_long(const char *text, long *value)
{
    char *end = NULL;
    long parsed;
    if (text == NULL || value == NULL || *text == '\0')
    {
        return false;
    }
    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0')
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool foc_tune_parse_float(const char *text, float *value)
{
    char *end = NULL;
    float parsed;
    if (text == NULL || value == NULL || *text == '\0')
    {
        return false;
    }
    errno = 0;
    parsed = strtof(text, &end);
    if (errno == ERANGE || end == text || *end != '\0' || !isfinite(parsed))
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool foc_tune_parse_target(const char *text, float *target_a)
{
    long target_ma;
    if (!foc_tune_parse_long(text, &target_ma) ||
        target_ma < lroundf(FOC_TUNE_MIN_TARGET_A * 1000.0f) ||
        target_ma > lroundf(FOC_TUNE_MAX_TARGET_A * 1000.0f))
    {
        return false;
    }
    *target_a = (float)target_ma / 1000.0f;
    return true;
}

static void foc_tune_print_status(void)
{
    float id_target_a, iq_target_a, id_kp, id_ki, iq_kp, iq_ki;
    uint32_t rate_hz, fault_code;
    bool alignment_requested;

    portENTER_CRITICAL(&foc_tune_lock);
    id_target_a = foc_tune_id_target_a;
    iq_target_a = foc_tune_iq_target_a;
    id_kp = foc_tune_id_kp;
    id_ki = foc_tune_id_ki;
    iq_kp = foc_tune_iq_kp;
    iq_ki = foc_tune_iq_ki;
    rate_hz = foc_tune_telemetry_rate_hz;
    fault_code = foc_tune_fault_code;
    alignment_requested = foc_tune_alignment_requested;
    portEXIT_CRITICAL(&foc_tune_lock);

    printf("FOC STATUS ID=%ld IQ=%ld RATE=%lu FAULT=%lu ALIGN=%d "
           "PI_ID_KP=%.6f PI_ID_KI=%.6f PI_IQ_KP=%.6f PI_IQ_KI=%.6f\n",
           foc_tune_to_milli(id_target_a), foc_tune_to_milli(iq_target_a),
           (unsigned long)rate_hz, (unsigned long)fault_code,
           alignment_requested ? 1 : 0, id_kp, id_ki, iq_kp, iq_ki);
    fflush(stdout);
}

static void foc_tune_handle_line(char *line)
{
    char *save = NULL;
    char *command = strtok_r(line, " \t\r\n", &save);
    char *operation = strtok_r(NULL, " \t\r\n", &save);
    char *argument = strtok_r(NULL, " \t\r\n", &save);
    char *extra = strtok_r(NULL, " \t\r\n", &save);
    char *extra2 = strtok_r(NULL, " \t\r\n", &save);
    char *extra3 = strtok_r(NULL, " \t\r\n", &save);

    if (command == NULL || operation == NULL || strcmp(command, "FOC") != 0)
    {
        foc_tune_reply("FOC ERR FORMAT");
        return;
    }

    if ((strcmp(operation, "ID") == 0 || strcmp(operation, "IQ") == 0) &&
        argument != NULL && extra == NULL)
    {
        float target_a;
        if (!foc_tune_parse_target(argument, &target_a))
        {
            foc_tune_reply("FOC ERR RANGE");
            return;
        }
        portENTER_CRITICAL(&foc_tune_lock);
        if (strcmp(operation, "ID") == 0)
        {
            foc_tune_id_target_a = target_a;
        }
        else
        {
            foc_tune_iq_target_a = target_a;
        }
        foc_tune_command_sequence++;
        portEXIT_CRITICAL(&foc_tune_lock);
        foc_tune_reply("FOC OK %s=%ld", operation, foc_tune_to_milli(target_a));
        return;
    }

    if (strcmp(operation, "PI") == 0 && argument != NULL && extra != NULL &&
        extra2 != NULL && extra3 != NULL &&
        strtok_r(NULL, " \t\r\n", &save) == NULL)
    {
        float id_kp, id_ki, iq_kp, iq_ki;
        if (!foc_tune_parse_float(argument, &id_kp) ||
            !foc_tune_parse_float(extra, &id_ki) ||
            !foc_tune_parse_float(extra2, &iq_kp) ||
            !foc_tune_parse_float(extra3, &iq_ki) ||
            id_kp < FOC_TUNE_PI_KP_MIN || id_kp > FOC_TUNE_PI_KP_MAX ||
            iq_kp < FOC_TUNE_PI_KP_MIN || iq_kp > FOC_TUNE_PI_KP_MAX ||
            id_ki < FOC_TUNE_PI_KI_MIN || id_ki > FOC_TUNE_PI_KI_MAX ||
            iq_ki < FOC_TUNE_PI_KI_MIN || iq_ki > FOC_TUNE_PI_KI_MAX)
        {
            foc_tune_reply("FOC ERR PI RANGE");
            return;
        }
        portENTER_CRITICAL(&foc_tune_lock);
        foc_tune_id_kp = id_kp;
        foc_tune_id_ki = id_ki;
        foc_tune_iq_kp = iq_kp;
        foc_tune_iq_ki = iq_ki;
        foc_tune_pi_sequence++;
        foc_tune_command_sequence++;
        portEXIT_CRITICAL(&foc_tune_lock);
        foc_tune_reply("FOC OK PI ID_KP=%.6f ID_KI=%.6f IQ_KP=%.6f IQ_KI=%.6f",
                       id_kp, id_ki, iq_kp, iq_ki);
        return;
    }

    if (strcmp(operation, "RATE") == 0 && argument != NULL && extra == NULL)
    {
        long rate_hz;
        if (!foc_tune_parse_long(argument, &rate_hz) ||
            rate_hz < (long)FOC_TUNE_MIN_TELEMETRY_RATE_HZ ||
            rate_hz > (long)FOC_TUNE_MAX_TELEMETRY_RATE_HZ)
        {
            foc_tune_reply("FOC ERR RATE");
            return;
        }
        portENTER_CRITICAL(&foc_tune_lock);
        foc_tune_telemetry_rate_hz = (uint32_t)rate_hz;
        portEXIT_CRITICAL(&foc_tune_lock);
        foc_tune_reply("FOC OK RATE=%ld", rate_hz);
        return;
    }

    if (strcmp(operation, "STOP") == 0 && argument == NULL)
    {
        portENTER_CRITICAL(&foc_tune_lock);
        foc_tune_id_target_a = 0.0f;
        foc_tune_iq_target_a = 0.0f;
        foc_tune_command_sequence++;
        portEXIT_CRITICAL(&foc_tune_lock);
        foc_tune_reply("FOC OK STOP");
        return;
    }

    if (strcmp(operation, "ALIGN") == 0 && argument == NULL)
    {
        portENTER_CRITICAL(&foc_tune_lock);
        foc_tune_id_target_a = 0.0f;
        foc_tune_iq_target_a = 0.0f;
        foc_tune_alignment_requested = true;
        foc_tune_command_sequence++;
        portEXIT_CRITICAL(&foc_tune_lock);
        foc_tune_reply("FOC OK ALIGN");
        return;
    }

    if (strcmp(operation, "STATUS") == 0 && argument == NULL)
    {
        foc_tune_print_status();
        return;
    }

    foc_tune_reply("FOC ERR FORMAT");
}

static void foc_tune_task(void *parameter)
{
    char line[FOC_TUNE_LINE_MAX];
    (void)parameter;
    while (true)
    {
        if (fgets(line, sizeof(line), stdin) == NULL)
        {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (strchr(line, '\n') == NULL && strlen(line) >= sizeof(line) - 1U)
        {
            int ch;
            while ((ch = getchar()) != '\n' && ch != EOF)
            {
            }
            foc_tune_reply("FOC ERR FORMAT");
            continue;
        }
        foc_tune_handle_line(line);
    }
}

esp_err_t foc_tune_protocol_start(void)
{
    if (foc_tune_task_handle != NULL)
    {
        return ESP_OK;
    }
    portENTER_CRITICAL(&foc_tune_lock);
    foc_tune_id_target_a = M1_CURRENT_LOOP_TEST_ID_REF_A;
    foc_tune_iq_target_a = M1_CURRENT_LOOP_TEST_IQ_REF_A;
    foc_tune_telemetry_rate_hz = FOC_TUNE_DEFAULT_TELEMETRY_RATE_HZ;
    foc_tune_command_sequence = 0U;
    foc_tune_fault_code = 0U;
    foc_tune_alignment_requested = false;
    foc_tune_id_kp = M1_CURRENT_LOOP_ID_PI_KP;
    foc_tune_id_ki = M1_CURRENT_LOOP_ID_PI_KI;
    foc_tune_iq_kp = M1_CURRENT_LOOP_IQ_PI_KP;
    foc_tune_iq_ki = M1_CURRENT_LOOP_IQ_PI_KI;
    foc_tune_pi_sequence = 0U;
    portEXIT_CRITICAL(&foc_tune_lock);

    if (xTaskCreate(foc_tune_task, "foc_tune_cmd", 3072, NULL, 4,
                   &foc_tune_task_handle) != pdPASS)
    {
        foc_tune_task_handle = NULL;
        ESP_LOGE(TAG, "failed to create command task");
        return ESP_ERR_NO_MEM;
    }
    foc_tune_reply("FOC OK RATE=%lu",
                   (unsigned long)FOC_TUNE_DEFAULT_TELEMETRY_RATE_HZ);
    foc_tune_print_status();
    return ESP_OK;
}

float foc_tune_get_id_target_a(void)
{
    float value;
    portENTER_CRITICAL(&foc_tune_lock);
    value = foc_tune_id_target_a;
    portEXIT_CRITICAL(&foc_tune_lock);
    return value;
}

float foc_tune_get_iq_target_a(void)
{
    float value;
    portENTER_CRITICAL(&foc_tune_lock);
    value = foc_tune_iq_target_a;
    portEXIT_CRITICAL(&foc_tune_lock);
    return value;
}

uint32_t foc_tune_get_telemetry_rate_hz(void)
{
    uint32_t value;
    portENTER_CRITICAL(&foc_tune_lock);
    value = foc_tune_telemetry_rate_hz;
    portEXIT_CRITICAL(&foc_tune_lock);
    return value;
}

void foc_tune_get_pi_config(foc_tune_pi_config_t *config, uint32_t *sequence)
{
    if (config == NULL)
    {
        return;
    }
    portENTER_CRITICAL(&foc_tune_lock);
    config->id_kp = foc_tune_id_kp;
    config->id_ki = foc_tune_id_ki;
    config->iq_kp = foc_tune_iq_kp;
    config->iq_ki = foc_tune_iq_ki;
    if (sequence != NULL)
    {
        *sequence = foc_tune_pi_sequence;
    }
    portEXIT_CRITICAL(&foc_tune_lock);
}

uint32_t foc_tune_get_command_sequence(void)
{
    uint32_t value;
    portENTER_CRITICAL(&foc_tune_lock);
    value = foc_tune_command_sequence;
    portEXIT_CRITICAL(&foc_tune_lock);
    return value;
}

uint32_t foc_tune_get_fault_code(void)
{
    uint32_t value;
    portENTER_CRITICAL(&foc_tune_lock);
    value = foc_tune_fault_code;
    portEXIT_CRITICAL(&foc_tune_lock);
    return value;
}

void foc_tune_set_fault_code(uint32_t fault_code)
{
    portENTER_CRITICAL(&foc_tune_lock);
    foc_tune_fault_code = fault_code;
    portEXIT_CRITICAL(&foc_tune_lock);
}

bool foc_tune_take_alignment_request(void)
{
    bool requested;
    portENTER_CRITICAL(&foc_tune_lock);
    requested = foc_tune_alignment_requested;
    foc_tune_alignment_requested = false;
    portEXIT_CRITICAL(&foc_tune_lock);
    return requested;
}

void foc_tune_complete_alignment(bool success, float applied_offset_rad)
{
    if (success)
    {
        foc_tune_reply("FOC ALIGN DONE offset=%.6f", applied_offset_rad);
    }
    else
    {
        foc_tune_reply("FOC ALIGN ERR CALIBRATION");
    }
}
