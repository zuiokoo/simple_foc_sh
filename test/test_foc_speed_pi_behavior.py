from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src" / "control" / "foc_speed_pi.h"
SOURCE = ROOT / "src" / "control" / "foc_speed_pi.c"
PI_SOURCE = ROOT / "src" / "control" / "foc_pi.c"


class FocSpeedPiBehaviorTest(unittest.TestCase):
    def test_speed_error_generates_limited_iq_reference_and_rejects_bad_dt(self):
        self.assertTrue(HEADER.exists(), "speed PI header has not been created")
        self.assertTrue(SOURCE.exists(), "speed PI source has not been created")

        compiler = shutil.which("gcc") or shutil.which("clang")
        if compiler is None:
            self.skipTest("a host C compiler is required for the speed PI behavior test")

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            (temp / "esp_err.h").write_text(
                """#ifndef ESP_ERR_H
#define ESP_ERR_H
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 0x102
#endif
""",
                encoding="ascii",
            )
            harness = temp / "speed_pi_harness.c"
            harness.write_text(
                """#include <math.h>
#include <stdio.h>
#include "foc_speed_pi.h"

int main(void)
{
    foc_speed_pi_t controller;
    float iq_ref = 0.0f;

    foc_speed_pi_init(&controller, 0.1f, 1.0f, -0.25f, 0.25f);

    if (foc_speed_pi_update(&controller, 1.0f, 0.0f, 0.1f, &iq_ref) != ESP_OK)
        return 1;
    if (fabsf(iq_ref - 0.2f) > 0.0001f)
        return 2;

    if (foc_speed_pi_update(&controller, 1.0f, 0.0f, 0.1f, &iq_ref) != ESP_OK)
        return 3;
    if (fabsf(iq_ref - 0.25f) > 0.0001f)
        return 4;

    if (foc_speed_pi_update(&controller, 1.0f, 0.0f, 0.0f, &iq_ref) != ESP_ERR_INVALID_ARG)
        return 5;

    if (foc_speed_pi_update(&controller, -1.0f, 0.0f, 0.1f, &iq_ref) != ESP_OK)
        return 6;
    if (iq_ref >= 0.0f)
        return 7;

    puts("speed_pi_behavior=passed");
    return 0;
}
""",
                encoding="ascii",
            )

            executable = temp / "speed_pi_harness.exe"
            compile_result = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-I",
                    str(temp),
                    "-I",
                    str(ROOT / "src" / "control"),
                    str(harness),
                    str(SOURCE),
                    str(PI_SOURCE),
                    "-lm",
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                compile_result.returncode,
                0,
                compile_result.stderr,
            )

            run_result = subprocess.run(
                [str(executable)],
                capture_output=True,
                text=True,
            )
            self.assertEqual(run_result.returncode, 0, run_result.stdout + run_result.stderr)
            self.assertIn("speed_pi_behavior=passed", run_result.stdout)


if __name__ == "__main__":
    unittest.main()
