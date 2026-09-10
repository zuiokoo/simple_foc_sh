import unittest

from current_loop_tuner.data_view import sample_to_table_row
from current_loop_tuner.telemetry import TelemetrySample


class DataViewTest(unittest.TestCase):
    def test_formats_sample_without_intermediate_references(self):
        sample = TelemetrySample(
            timestamp_us=2_500_000,
            id_target_a=0.1,
            iq_target_a=0.0,
            id_a=0.084,
            iq_a=-0.003,
            vd_v=0.72,
            vq_v=-0.04,
            speed_rad_s=-0.12,
            current_age_us=125,
            angle_age_us=782,
            fault=0,
        )
        self.assertEqual(
            sample_to_table_row(sample, 2_000_000),
            ("0.500", "100", "84", "0", "-3", "0.720", "-0.040", "-0.12", "125", "782", "0"),
        )


if __name__ == "__main__":
    unittest.main()