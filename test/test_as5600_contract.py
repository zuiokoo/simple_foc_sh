from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "sensor" / "as5600.h"
SOURCE = ROOT / "src" / "sensor" / "as5600.c"
MAIN = ROOT / "src" / "main.c"


class As5600ContractTest(unittest.TestCase):
    def test_module_uses_native_esp_idf_i2c_initialization(self):
        self.assertTrue(HEADER.exists(), "AS5600 header has not been created")
        self.assertTrue(SOURCE.exists(), "AS5600 source has not been created")

        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("esp_err_t as5600_init(void);", header)
        self.assertIn('#include "driver/i2c_master.h"', source)
        self.assertIn("i2c_new_master_bus(", source)
        self.assertIn("i2c_master_bus_add_device(", source)
        self.assertIn("#define AS5600_I2C_FREQUENCY_HZ 400000U", source)
        self.assertIn(".flags.disable_ack_check = true", source)
        self.assertNotIn("i2c_param_config(", source)
        self.assertNotIn("i2c_driver_install(", source)

    def test_main_keeps_pwm_calibration_start_explicitly_guarded(self):
        main = MAIN.read_text(encoding="utf-8")
        config = (ROOT / "src" / "motor" / "motor_config.h").read_text(encoding="utf-8")

        self.assertIn('#include "sensor/as5600.h"', main)
        self.assertIn("as5600_init();", main)
        self.assertIn("#if M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION", main)
        self.assertIn("motor_pwm_start();", main)
        self.assertIn("#define M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION 1", config)

    def test_public_api_has_only_combined_angle_velocity_reader(self):
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("as5600_measure_angle_velocity(", header)
        self.assertNotIn("as5600_get_velocity", header)
        self.assertNotIn("as5600_get_velocity", source)


if __name__ == "__main__":
    unittest.main()
