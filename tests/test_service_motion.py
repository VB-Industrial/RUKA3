from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class ServiceMotionTests(unittest.TestCase):
    def test_shared_trajectory_limits_preemption_watchdog_and_observer(self):
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'motion')
            subprocess.run(['c++','-std=c++20','-O2','-DRUKA3_JOINT_INDEX=4','-I'+str(ROOT/'App'),str(ROOT/'tests/service_motion_test.cpp'),str(ROOT/'App/encoder_calibration.cpp'),'-o',binary],check=True)
            subprocess.run([binary],check=True)
