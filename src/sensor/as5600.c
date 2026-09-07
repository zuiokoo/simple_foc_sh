#include "as5600.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "motor/motor_config.h"
#include "esp_timer.h"

#define AS5600_I2C_FREQUENCY_HZ 400000U

#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_RAW_ANGLE_L 0x0D
#define AS5600_I2C_TIMEOUT_MS 100
/*
 * The angle task samples AS5600 every 1 ms.  Update velocity at every
 * sample, then filter it with a short time constant so delay compensation
 * follows acceleration without turning encoder quantization into noise.
 */
#define AS5600_VELOCITY_FILTER_TAU_S 0.002f

static const char *TAG = "AS5600";
static i2c_master_bus_handle_t as5600_bus;
static i2c_master_dev_handle_t as5600_device;

static float as5600_velocity_reference_angle_rad = 0.0f;
static int64_t as5600_velocity_reference_time_us = 0;
static float as5600_velocity_estimate_rad_s = 0.0f;
static int as5600_velocity_initialized = 0;
esp_err_t as5600_init(void)
{
	if (as5600_bus != NULL || as5600_device != NULL)
	{
		ESP_LOGW(TAG, "AS5600 I2C is already initialized");
		return ESP_ERR_INVALID_STATE;
	}

	i2c_master_bus_config_t bus_config = {
		.i2c_port = -1,
		.sda_io_num = M1_SENSOR_SDA_GPIO,
		.scl_io_num = M1_SENSOR_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	esp_err_t result = i2c_new_master_bus(&bus_config, &as5600_bus);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(result));
		as5600_bus = NULL;
		return result;
	}

	i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = M1_AS5600_ADDRESS,
		.scl_speed_hz = AS5600_I2C_FREQUENCY_HZ,
		/* Avoid the ESP-IDF 5.5 NACK path that can busy-wait forever. */
		.flags.disable_ack_check = true,
	};

	result = i2c_master_bus_add_device(
		as5600_bus,
		&device_config,
		&as5600_device);
	if (result != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to add AS5600 device: %s", esp_err_to_name(result));
		i2c_del_master_bus(as5600_bus);
		as5600_bus = NULL;
		as5600_device = NULL;
		return result;
	}

	ESP_LOGI(TAG,
			 "I2C initialized: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X, speed=%u Hz",
			 M1_SENSOR_SDA_GPIO,
			 M1_SENSOR_SCL_GPIO,
			 M1_AS5600_ADDRESS,
			 AS5600_I2C_FREQUENCY_HZ);
	return ESP_OK;
}

esp_err_t as5600_read_raw_angle(uint16_t *raw_angle)
{
	if (raw_angle == NULL)
	{
		ESP_LOGW(TAG, "raw_angle is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	if (as5600_device == NULL)
	{
		ESP_LOGW(TAG, "AS5600 is not initialized");
		return ESP_ERR_INVALID_STATE;
	}
	uint8_t register_address = AS5600_REG_RAW_ANGLE_H;
	uint8_t angle_data[2] = {0};
	esp_err_t result = i2c_master_transmit_receive(as5600_device, &register_address, 1, angle_data, sizeof(angle_data), AS5600_I2C_TIMEOUT_MS);
	if (result != ESP_OK)
	{
		return result;
	}
	*raw_angle = ((uint16_t)angle_data[0] << 8 | angle_data[1]) & 0x0fff;
	return ESP_OK;
}
float as5600_raw_to_mechanical_angle(uint16_t raw_angle)
{
	return (float)raw_angle * 2.0f * 3.14159265358979323846f / 4096.0f;
}
esp_err_t as5600_get_mechanical_angle(float *angle_rad)
{
	if (angle_rad == NULL)
	{
		ESP_LOGW(TAG, "angle_rad is NULL");
		return ESP_ERR_INVALID_ARG;
	}
	uint16_t raw_angle = 0;
	esp_err_t result = as5600_read_raw_angle(&raw_angle);
	if (result != ESP_OK)
	{
		return result;
	}
	*angle_rad = as5600_raw_to_mechanical_angle(raw_angle);
	return ESP_OK;
}

esp_err_t as5600_measure_angle_velocity(float *angle_rad, float *velocity_rad_s)
{
	if (angle_rad == NULL || velocity_rad_s == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	float current_angle_rad = 0.0f;
	esp_err_t result = as5600_get_mechanical_angle(&current_angle_rad);
	if (result != ESP_OK)
	{
		return result;
	}
	*angle_rad = current_angle_rad;

	int64_t current_time_us = esp_timer_get_time();
	if (as5600_velocity_initialized == 0)
	{
		as5600_velocity_reference_angle_rad = current_angle_rad;
		as5600_velocity_reference_time_us = current_time_us;
		as5600_velocity_estimate_rad_s = 0.0f;
		as5600_velocity_initialized = 1;
		*velocity_rad_s = 0.0f;
		return ESP_OK;
	}

	int64_t delta_time_us =
		current_time_us - as5600_velocity_reference_time_us;
	if (delta_time_us <= 0)
	{
		return ESP_ERR_INVALID_STATE;
	}

	float delta_angle_rad =
		current_angle_rad - as5600_velocity_reference_angle_rad;
	if (delta_angle_rad > 3.14159265358979323846f)
	{
		delta_angle_rad -= 2.0f * 3.14159265358979323846f;
	}
	else if (delta_angle_rad < -3.14159265358979323846f)
	{
		delta_angle_rad += 2.0f * 3.14159265358979323846f;
	}

	float delta_time_s = (float)delta_time_us / 1000000.0f;
	float instantaneous_velocity_rad_s = delta_angle_rad / delta_time_s;
	float filter_alpha = delta_time_s /
		(AS5600_VELOCITY_FILTER_TAU_S + delta_time_s);
	as5600_velocity_estimate_rad_s +=
		filter_alpha * (instantaneous_velocity_rad_s -
			as5600_velocity_estimate_rad_s);
	as5600_velocity_reference_angle_rad = current_angle_rad;
	as5600_velocity_reference_time_us = current_time_us;

	*velocity_rad_s = as5600_velocity_estimate_rad_s;
	return ESP_OK;
}
