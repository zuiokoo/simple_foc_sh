import unittest

from current_loop_tuner.telemetry import parse_line


class TelemetryTest(unittest.TestCase):
    def test_parse_compact_data_line_without_intermediate_references(self):
        sample = parse_line("I (3000) FOC_DATA,123456,0,100,4,96,0,800,2500,120,80,0")
        self.assertEqual(sample.timestamp_us, 123456)
        self.assertAlmostEqual(sample.iq_target_a, 0.100)
        self.assertAlmostEqual(sample.iq_a, 0.096)
        self.assertAlmostEqual(sample.id_a, 0.004)
        self.assertAlmostEqual(sample.vq_v, 0.800)
        self.assertAlmostEqual(sample.speed_rad_s, 2.5)
        self.assertEqual(sample.fault, 0)
        self.assertFalse(hasattr(sample, "id_ref_a"))
        self.assertFalse(hasattr(sample, "iq_ref_a"))

    def test_parse_existing_human_tune_line_without_iq_reference(self):
        line = (
            "E (2810) FOC_TUNE: dt_us=200 speed_mrad_s=4200 "
            "angle_age_us=80 current_age_us=120 iu_mA=1 iv_mA=-2 iw_mA=1 "
            "iq_target_mA=100 id_mA=3 iq_mA=94 "
            "id_err_mA=-3 iq_err_mA=6 vd_mV=0 vq_mV=812"
        )
        sample = parse_line(line)
        self.assertAlmostEqual(sample.iq_target_a, 0.100)
        self.assertAlmostEqual(sample.iq_a, 0.094)
        self.assertAlmostEqual(sample.id_target_a, 0.000)
        self.assertAlmostEqual(sample.vq_v, 0.812)
        self.assertFalse(hasattr(sample, "iq_ref_a"))

    def test_unrelated_log_is_ignored(self):
        self.assertIsNone(parse_line("I (100) FOC_CURRENT: PWM started"))


if __name__ == "__main__":
    unittest.main()