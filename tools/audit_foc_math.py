"""Reproduce the G474 audit's linear observer and LPF calculations (no device I/O)."""
import json
import math
import numpy as np

def observer_poles(dt,g1=.01570098941,g2=3.92522777636,g3=387.547117953):
    # State: corrected theta, omega, acceleration, previous returned prediction.
    matrix=np.array([[1,dt,dt*dt/2,-g1],[0,1,dt,-g2],[0,0,1,-g3],[1,dt,dt*dt/2,0.]])
    z=np.linalg.eigvals(matrix)
    return [{'z_real':float(v.real),'z_imag':float(v.imag),'abs':float(abs(v)),
             's_real':float(np.log(complex(v)).real/dt),'s_imag':float(np.log(complex(v)).imag/dt)} for v in z]

def lpf(alpha=.0925,dt=1/16000):
    pole=1-alpha
    cutoff=math.acos((1+pole*pole-2*alpha*alpha)/(2*pole))/(2*math.pi*dt)
    return {'alpha':alpha,'dt_s':dt,'cutoff_hz':cutoff,'time_constant_s':-dt/math.log(pole),
            'dc_group_delay_s':pole/alpha*dt,
            'phase_deg':{f:float(np.angle(alpha/(1-pole*np.exp(-2j*np.pi*f*dt)),deg=True)) for f in [250,500,1000]}}

if __name__=='__main__':
    print(json.dumps({'lpf':lpf(),'observer_16khz':observer_poles(1/16000),
                      'observer_40khz_same_gains':observer_poles(1/40000)},indent=2))
