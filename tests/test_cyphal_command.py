from pathlib import Path
import os
import shutil
import struct
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
class CyphalCommandTests(unittest.TestCase):
    def test_generated_g431_and_g474_publish_identical_command_payloads(self):
        generator=os.environ.get('CYPHAL_NNVG_EXECUTABLE') or shutil.which('nnvg')
        if not generator:
            candidate=ROOT/'build/.venv-nunavut/bin/nnvg'
            if candidate.exists():generator=str(candidate)
        if not generator:self.skipTest('Install nnvg to test DSDL generation')
        namespaces=ROOT/'Boards/g431/Drivers/cyphal_types'
        prdt=namespaces/'public_regulated_data_types/uavcan'
        voltbro=namespaces/'voltbro/voltbro'
        with tempfile.TemporaryDirectory() as temp:
            generated=Path(temp)/'generated'
            for namespace in (prdt,voltbro):
                subprocess.run([generator,'--target-language','c','--language-standard','c11','--outdir',str(generated),
                    '--lookup-dir',str(prdt),'--lookup-dir',str(voltbro),str(namespace)],check=True,stdout=subprocess.DEVNULL)
            payloads=[]
            for n,headers in enumerate([generated,ROOT/'Boards/g474/Drivers/libcxxcanard/libs']):
                binary=str(Path(temp)/f'wire{n}')
                subprocess.run(['c++','-std=c++20','-O2','-I'+str(headers),str(ROOT/'tests/cyphal_command_test.cpp'),'-o',binary],check=True)
                payloads.append(subprocess.check_output([binary]).decode())
            self.assertEqual(payloads[0],payloads[1])
            self.assertEqual(payloads[0],struct.pack('<7f',-.2,.125,-.3,20,.4,0,0).hex())
