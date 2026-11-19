from pathlib import Path
import argparse
import importlib.util
import json
import struct
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('foc_trace',ROOT/'tools/foc_trace.py')
trace=importlib.util.module_from_spec(spec);spec.loader.exec_module(trace)

def capture(profile=1,n=16):
    info=[0]*32
    info[:11]=[trace.MAGIC,1,2,profile,n,256,32,0,160000000,10000,1]
    info[28]=struct.unpack('<I',struct.pack('<f',.0925))[0]
    frames=[]
    for i in range(n):
        row=[0]*32;row[0]=i+1;row[1]=(0xffff8000+i*10000)&0xffffffff
        row[2]=10000;row[3]=1000;row[11]=300
        for j,(_,kind) in enumerate(trace.FIELDS[profile],12):
            row[j]=struct.unpack('<I',struct.pack('<f',.1*i))[0] if kind=='f' else 42
        frames.append(row)
    return {'info':info,'frames':frames}

class TraceTests(unittest.TestCase):
    def test_actual_ring_core(self):
        with tempfile.TemporaryDirectory() as td:
            exe=Path(td)/'ring'
            subprocess.run(['c++','-std=c++20','-O2','-fsanitize=undefined','-I',str(ROOT/'App'),str(ROOT/'tests/foc_trace_test.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_current_pi_with_diagnostics(self):
        with tempfile.TemporaryDirectory() as td:
            exe=Path(td)/'pi'
            subprocess.run(['c++','-std=c++20','-O2','-DSTM32G4','-DRUKA3_JOINT_INDEX=1','-DRUKA3_FOC_TRACE=1',
                '-I',str(ROOT/'tests/stubs'),'-I',str(ROOT/'App'),'-I',str(ROOT/'Boards/g474/Drivers/libvoltbro'),
                str(ROOT/'tests/g474_current_control_test.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_decode_all_profiles_and_signed_wire(self):
        for profile in trace.FIELDS:
            data=capture(profile);data['frames'][0][1]=-1
            meta,rows=trace.decode(data)
            self.assertEqual(rows[0]['cycle_start'],0xffffffff)
            self.assertEqual(len(rows),16)
        self.assertEqual(trace.float_word(0xbf800000),-1.)

    def test_reject_active_and_truncated(self):
        data=capture();data['info'][2]=1
        with self.assertRaises(ValueError):trace.decode(data)
        data=capture();data['frames'][0].pop()
        with self.assertRaises(ValueError):trace.decode(data)

    def test_fft_and_analyzer_wrap_and_gaps(self):
        import numpy as np
        fs=16000;n=256
        f,a=trace.spectrum(np.sin(2*np.pi*1000*np.arange(n)/fs),fs)
        self.assertEqual(f[np.argmax(a)],1000)
        with tempfile.TemporaryDirectory() as td:
            td=Path(td);source=td/'SYNTHETIC.json';out=td/'analysis'
            for profile in trace.FIELDS:
                source.write_text(json.dumps(capture(profile)))
                trace.analyze(argparse.Namespace(input=source,compare=None,output=out))
                metrics=json.loads((out/'metrics.json').read_text())[0]
                self.assertTrue(metrics['uniform_for_fft'])
                self.assertAlmostEqual(metrics['window_s'],15/16000)
            data=capture();data['frames'][8][0]+=1
            source.write_text(json.dumps(data));trace.analyze(argparse.Namespace(input=source,compare=None,output=out))
            self.assertFalse(json.loads((out/'metrics.json').read_text())[0]['uniform_for_fft'])

    def test_audit_observer_and_lpf_math(self):
        math_spec=importlib.util.spec_from_file_location('audit_math',ROOT/'tools/audit_foc_math.py')
        audit=importlib.util.module_from_spec(math_spec);math_spec.loader.exec_module(audit)
        self.assertAlmostEqual(audit.lpf()['cutoff_hz'],247.3599,places=3)
        for dt in [1/16000,1/40000]:
            self.assertTrue(all(p['abs']<1 for p in audit.observer_poles(dt)))
        # ADC writes bus/A/B/C, while control reads A/B/C/bus: atomic words
        # do not ensure one scan. This is a possible interleaving, not a capture.
        buffer=[0]*4
        a=buffer[1]
        buffer[:]=[1]*4
        b,c,bus=buffer[2],buffer[3],buffer[0]
        self.assertEqual((a,b,c,bus),(0,1,1,1))

    def test_reproduce_existing_q31_negation_ub(self):
        with tempfile.TemporaryDirectory() as td:
            td=Path(td);source=td/'q31.cpp';exe=td/'q31'
            source.write_text('#include <cstdint>\nint main(){volatile int32_t x=INT32_MIN; volatile int32_t y=-x; (void)y;}')
            subprocess.run(['c++','-fsanitize=undefined','-fno-sanitize-recover=undefined',str(source),'-o',str(exe)],check=True)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('negation',result.stderr)

if __name__=='__main__':unittest.main()
