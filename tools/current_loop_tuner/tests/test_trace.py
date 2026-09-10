import tempfile
import unittest
from pathlib import Path

from current_loop_tuner.telemetry import parse_line
from current_loop_tuner.trace import TraceBuffer


class TraceTest(unittest.TestCase):
    def test_buffer_is_bounded_and_writes_csv_without_intermediate_references(self):
        samples = [parse_line(f"FOC_DATA,{n},0,100,0,100,0,800,0,100,100,0") for n in (1, 2, 3)]
        trace = TraceBuffer(maxlen=2)
        for sample in samples:
            trace.append(sample)
        self.assertEqual([s.timestamp_us for s in trace.snapshot()], [2, 3])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            trace.write_csv(path)
            csv_text = path.read_text(encoding="utf-8-sig")
            self.assertIn("iq_target_a", csv_text)
            self.assertNotIn("id_ref_a", csv_text)
            self.assertNotIn("iq_ref_a", csv_text)
            self.assertIn("0.1", csv_text)


if __name__ == "__main__":
    unittest.main()