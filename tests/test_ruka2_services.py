from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class Ruka2ServicesTests(unittest.TestCase):
    def test_original_calibration_regressions(self):
        with tempfile.TemporaryDirectory() as d:
            binary = str(Path(d)/'calibration')
            subprocess.run(['c++','-std=c++20','-O2','-I'+str(ROOT/'App'),str(ROOT/'tests/ruka2_calibration_validation_test.cpp'),str(ROOT/'App/encoder_calibration.cpp'),'-o',binary],check=True)
            subprocess.run([binary],check=True)

    def test_eeprom_power_loss_at_every_byte(self):
        with tempfile.TemporaryDirectory() as d:
            obj=str(Path(d)/'storage.o'); binary=str(Path(d)/'storage')
            subprocess.run(['cc','-std=c11','-O2','-I'+str(ROOT/'App'),'-c',str(ROOT/'App/encoder_calibration_storage.c'),'-o',obj],check=True)
            subprocess.run(['c++','-std=c++20','-O2','-I'+str(ROOT/'App'),str(ROOT/'tests/calibration_storage_test.cpp'),obj,'-o',binary],check=True)
            subprocess.run([binary],check=True)

    def test_shared_register_codec_with_both_dsdl_trees(self):
        with tempfile.TemporaryDirectory() as d:
            for board in ('g431','g474'):
                binary=str(Path(d)/board)
                subprocess.run(['c++','-std=c++20','-O2','-I'+str(ROOT/'App'),'-I'+str(ROOT/f'Boards/{board}/Drivers/libcxxcanard/src'),str(ROOT/'tests/register_codec_test.cpp'),str(ROOT/'App/encoder_calibration.cpp'),'-o',binary],check=True)
                subprocess.run([binary],check=True)

    def test_calibration_zero_velocity_brakes_and_cancel_disables(self):
        with tempfile.TemporaryDirectory() as d:
            binary=str(Path(d)/'adapter')
            for joint in (1,2,3,4,5,6):
                subprocess.run(['c++','-std=c++20','-O2',f'-DRUKA3_JOINT_INDEX={joint}','-I'+str(ROOT/'App'),str(ROOT/'tests/calibration_service_test.cpp'),str(ROOT/'App/encoder_calibration.cpp'),str(ROOT/'App/calibration_service.cpp'),'-o',binary],check=True)
                subprocess.run([binary],check=True)
