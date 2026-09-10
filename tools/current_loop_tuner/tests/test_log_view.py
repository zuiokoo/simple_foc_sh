import unittest

from current_loop_tuner.log_view import append_log_line


class LogViewTest(unittest.TestCase):
    def test_append_log_line_keeps_order_and_newest_limit(self):
        lines = []
        for value in ("first", "second", "third"):
            lines = append_log_line(lines, value, max_lines=2)

        self.assertEqual(lines, ["second", "third"])


if __name__ == "__main__":
    unittest.main()
