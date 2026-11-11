from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class G474CurrentControlTests(unittest.TestCase):
    def test_real_pi_current_conversion_and_transforms(self):
        with tempfile.TemporaryDirectory(prefix="ruka3-current-test-") as directory:
            binary = Path(directory) / "current-test"
            subprocess.run([
                "c++", "-std=c++20", "-O2", "-DSTM32G4", "-DRUKA3_JOINT_INDEX=1",
                "-I", str(ROOT / "tests/stubs"), "-I", str(ROOT / "App"),
                "-I", str(ROOT / "Boards/g474/Drivers/libvoltbro"),
                str(ROOT / "tests/g474_current_control_test.cpp"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)

    def test_control_period_matches_existing_timer(self):
        timer = (ROOT / "Boards/g474/Core/Src/tim.c").read_text()
        config = (ROOT / "App/robot_config.h").read_text()
        prescaler = int(re.search(r"#define RUKA3_G474_PWM_PRESCALER (\d+)U", config)[1])
        period = int(re.search(r"#define RUKA3_G474_PWM_TICKS (\d+)U", config)[1]) - 1
        self.assertIn("htim1.Init.Prescaler = RUKA3_G474_PWM_PRESCALER;", timer)
        self.assertIn("htim1.Init.Period = RUKA3_G474_PWM_TICKS - 1U;", timer)
        actual = float(re.search(r"control_period_s = ([\d.]+)F", config.split("kG474MotorProfile{", 1)[1])[1])
        self.assertAlmostEqual(actual, (prescaler + 1) * (period + 1) / 160_000_000)

    def test_adc_trigger_fits_after_pwm_and_before_period_end(self):
        config = (ROOT / "App/robot_config.h").read_text()
        ticks = lambda name: int(re.search(rf"#define RUKA3_G474_{name} (\d+)U", config)[1])
        clock = 160_000_000 / (ticks("PWM_PRESCALER") + 1)
        self.assertGreater((ticks("CURRENT_SAMPLE_TICKS") - ticks("MAX_PWM_TICKS")) / clock, 3e-6)
        scan = 4 * (6.5 + 12.5) / 40_000_000
        self.assertLess(ticks("CURRENT_SAMPLE_TICKS") / clock + scan, ticks("PWM_TICKS") / clock)
        adc = (ROOT / "Boards/g474/Core/Src/adc.c").read_text()
        self.assertIn("hadc1.Init.ContinuousConvMode = DISABLE", adc)
        self.assertIn("hadc1.Init.OversamplingMode = DISABLE", adc)
        self.assertIn("ADC_EXTERNALTRIG_T1_TRGO2", adc)
        timer = (ROOT / "Boards/g474/Core/Src/tim.c").read_text()
        self.assertIn("TIM_TRGO2_OC4REF", timer)
        self.assertIn("sConfigOC.OCMode = TIM_OCMODE_PWM2", timer)
        startup = (ROOT / "Boards/g474/Drivers/libvoltbro/voltbro/motors/bldc/bldc.cpp").read_text()
        self.assertLess(startup.index("HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4)"), startup.index("inverter.start()"))

    def test_robot_timing_header_is_c_compatible(self):
        subprocess.run(["cc", "-std=c11", "-fsyntax-only", "-x", "c", "-I", str(ROOT / "App"), "-"],
                       input='#include "robot_config.h"\n_Static_assert(RUKA3_G474_CURRENT_SAMPLE_TICKS < RUKA3_G474_PWM_TICKS, "sample");',
                       text=True, check=True)


if __name__ == "__main__":
    unittest.main()
