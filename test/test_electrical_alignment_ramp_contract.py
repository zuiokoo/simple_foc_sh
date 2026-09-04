from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / 'src' / 'main.c'

class ElectricalAlignmentRampContractTest(unittest.TestCase):
    def test_alignment_voltage_is_ramped_after_pwm_start(self):
        text = MAIN.read_text(encoding='utf-8')
        self.assertIn('alignment_ramp_steps', text)
        self.assertIn('motor_pwm_set_duty(0.5f, 0.5f, 0.5f)', text)
        self.assertIn('alignment_fraction', text)
        self.assertIn('M1_ALIGNMENT_DUTY_U', text)
        self.assertIn('M1_ALIGNMENT_DUTY_V', text)
        self.assertIn('M1_ALIGNMENT_DUTY_W', text)

if __name__ == '__main__':
    unittest.main()