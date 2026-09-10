import unittest

import current_loop_tuner.protocol as protocol
from current_loop_tuner.protocol import (
    format_align,
    format_rate,
    format_set_id,
    format_set_iq,
    format_status,
    format_stop,
    parse_response,
)


class ProtocolTest(unittest.TestCase):
    def test_format_signed_targets(self):
        self.assertEqual(format_set_id(-100), b"FOC ID -100\n")
        self.assertEqual(format_set_iq(100), b"FOC IQ 100\n")
        self.assertEqual(format_stop(), b"FOC STOP\n")
        self.assertEqual(format_align(), b"FOC ALIGN\n")
        self.assertEqual(format_rate(50), b"FOC RATE 50\n")
        self.assertEqual(format_status(), b"FOC STATUS\n")

    def test_formats_runtime_pi_parameters_atomically(self):
        self.assertTrue(hasattr(protocol, "format_set_pi"))
        self.assertEqual(
            protocol.format_set_pi(2.0, 4.0, 4.0, 1.0),
            b"FOC PI 2.000000 4.000000 4.000000 1.000000\n",
        )

    def test_rejects_out_of_range_values(self):
        with self.assertRaisesRegex(ValueError, "-500..500"):
            format_set_iq(501)
        with self.assertRaisesRegex(ValueError, "-500..500"):
            format_set_id(-501)
        with self.assertRaisesRegex(ValueError, "1..100"):
            format_rate(0)

    def test_rejects_invalid_pi_parameters(self):
        self.assertTrue(hasattr(protocol, "format_set_pi"))
        if hasattr(protocol, "format_set_pi"):
            with self.assertRaisesRegex(ValueError, "PI"):
                protocol.format_set_pi(-1.0, 4.0, 4.0, 1.0)

    def test_parses_acknowledgements_and_errors(self):
        self.assertEqual(parse_response("FOC OK IQ=100").value, 100)
        self.assertEqual(parse_response("FOC OK ALIGN").kind, "ok_align")
        self.assertEqual(parse_response("FOC ALIGN DONE offset=5.58").kind, "align_done")
        self.assertEqual(parse_response("FOC ERR RANGE").kind, "error")
        status = parse_response("FOC STATUS ID=0 IQ=100 RATE=50 ALIGN=0 FAULT=2")
        self.assertEqual(status.fields["FAULT"], "2")
        self.assertIsNone(parse_response("I (100) booting"))

    def test_parses_runtime_pi_acknowledgement(self):
        response = parse_response(
            "FOC OK PI ID_KP=2.000000 ID_KI=4.000000 IQ_KP=4.000000 IQ_KI=1.000000"
        )
        self.assertIsNotNone(response)
        self.assertEqual(response.kind, "ok_pi")
        self.assertEqual(response.fields["ID_KI"], "4.000000")


if __name__ == "__main__":
    unittest.main()