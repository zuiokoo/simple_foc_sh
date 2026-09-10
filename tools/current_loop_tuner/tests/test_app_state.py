import unittest
from pathlib import Path

from current_loop_tuner.app import alignment_request_should_release


APP_SOURCE = (Path(__file__).resolve().parents[1] / "current_loop_tuner" / "app.py").read_text(encoding="utf-8")


class AppStateTest(unittest.TestCase):
    def test_alignment_releases_button_on_error(self):
        self.assertTrue(alignment_request_should_release("error", 0.0))
        self.assertTrue(alignment_request_should_release("align_error", 0.0))

    def test_alignment_releases_button_after_timeout(self):
        self.assertFalse(alignment_request_should_release(None, 1.0))
        self.assertTrue(alignment_request_should_release(None, 6.0))

    def test_alignment_success_shows_info_dialog(self):
        self.assertIn("messagebox.showinfo(", APP_SOURCE)
        self.assertIn('"零点对齐成功"', APP_SOURCE)


if __name__ == "__main__":
    unittest.main()