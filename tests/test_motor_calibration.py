from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class MotorCalibrationTests(unittest.TestCase):
    def test_raw_frame_and_monotonic_lut(self):
        with tempfile.TemporaryDirectory() as d:
            binary=str(Path(d)/'geometry')
            subprocess.run(['c++','-std=c++20','-O2','-I'+str(ROOT/'Boards/g431/Drivers/libvoltbro/voltbro/motors/bldc/foc'),str(ROOT/'tests/motor_calibration_geometry_test.cpp'),'-o',binary],check=True)
            subprocess.run([binary],check=True)
