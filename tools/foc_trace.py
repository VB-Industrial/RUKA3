"""G474 FOC trace: explicit capture control, frozen-ring export, offline analysis.

No command in this tool writes motor ARM, targets, gains, calibration or EEPROM.
Capture data are native axes, not public joint torque/current signs.
"""
import argparse
import asyncio
import json
import struct
from pathlib import Path

MAGIC = 0x46544331
HEADER = ["period", "cycle_start", "interval_cycles", "isr_cycles", "max_isr_cycles",
          "entry_timer_count", "dma_before", "dma_after", "adc_flags", "dma_flags", "flags", "sample_offset_cycles"]
FIELDS = {
 1: [("adc_bus","u"),("adc_a","u"),("adc_b","u"),("adc_c","u"),
     ("ia","f"),("ib","f"),("ic","f"),("phase_sum","f"),("id_raw","f"),("iq_raw","f"),
     ("id_filtered","f"),("iq_filtered","f"),("offset_a","f"),("offset_b","f"),("offset_c","f"),
     ("bus_v","f"),("previous_pwm_ab","u"),("previous_pwm_c_and_sample_timer","u"),("motor_raw","u"),("electric_angle","f")],
 2: [("id_target","f"),("iq_before_limits","f"),("iq_target","f"),("id_error","f"),("iq_error","f"),
     ("id_measured","f"),("iq_measured","f"),("vd_pi_raw","f"),("vq_pi_raw","f"),
     ("vd_pi","f"),("vq_pi","f"),("vd_applied","f"),("vq_applied","f"),
     ("voltage_available","f"),("vq_limit","f"),("id_integral_as","f"),("iq_integral_as","f"),
     ("staged_pwm_ab","u"),("staged_pwm_c","u"),("bus_v","f")],
 3: [("motor_raw","u"),("rotor_corrected","f"),("observer_prediction","f"),("observer_corrected","f"),
     ("observer_omega","f"),("observer_accel","f"),("predicted_omega","f"),("electric_angle","f"),
     ("innovation","f"),("previous_prediction","f"),("configured_dt_s","f"),("cordic_cos_raw","u"),("cordic_sin_raw","u"),
     ("encoder_cpr","u"),("pole_pairs","u"),("electric_offset","i"),("lut_delta","i"),
     ("encoder_config_flags","u"),("observer_g1","f"),("observer_g2","f")],
}
PROFILES={"measurement":1,"control":2,"angle":3}
FLAGS={"software_pwm_entry":1,"d_saturated":1<<4,"q_saturated":1<<5,"vector_limited":1<<6,
       "pwm_clipped":1<<7,"dma_moved":1<<8,"adc_overrun":1<<9,"dma_error":1<<10,
       "update_pending":1<<11,"late_entry":1<<12,"current_loop_ran":1<<13,
       "cordic_min":1<<14,"hardware_moe_end":1<<15,"observer_valid":1<<16,"d_pulse_window":1<<17,"d_pulse_energized":1<<18}


def float_word(word):
    return struct.unpack("<f",struct.pack("<I",int(word)&0xffffffff))[0]


def decode(capture):
    meta=[int(x)&0xffffffff for x in capture["info"]]
    if len(meta)!=32 or meta[0]!=MAGIC or meta[1]!=1 or meta[6]!=32 or meta[3] not in FIELDS:
        raise ValueError("Unsupported FOC trace schema")
    if meta[2]!=2 or meta[4]==0 or len(capture["frames"])!=meta[4]:
        raise ValueError("Expected a complete, nonempty frozen capture")
    rows=[]
    for frame in capture["frames"]:
        if len(frame)!=32:raise ValueError("Truncated frame")
        w=[int(x)&0xffffffff for x in frame];row=dict(zip(HEADER,w[:12]))
        for word,(name,kind) in zip(w[12:],FIELDS[meta[3]]):
            row[name]=float_word(word) if kind=="f" else word if kind=="u" else struct.unpack("<i",struct.pack("<I",word))[0]
        row["mode"]=(row["flags"]>>1)&7
        rows.append(row)
    return meta,rows


async def device(args):
    import pycyphal.application
    import pycyphal.dsdl
    import uavcan.node
    import uavcan.register
    import uavcan.primitive.array
    node=pycyphal.application.make_node(uavcan.node.GetInfo_1_0.Response());node.start()
    client=node.make_client(uavcan.register.Access_1_0,args.node);client.response_timeout=2
    async def access(name,value=None):
        request=uavcan.register.Access_1_0.Request(name=uavcan.register.Name_1_0(name=name))
        if value is not None:
            request.value=uavcan.register.Value_1_0(integer32=uavcan.primitive.array.Integer32_1_0(value=[value]))
        result=await client.call(request)
        if result is None:raise RuntimeError(f"Timeout: {name}")
        obj=pycyphal.dsdl.to_builtin(result[0].value)
        if "integer32" not in obj:raise RuntimeError(f"{name}: unavailable; is a trace-enabled G474 image running?")
        return obj["integer32"]["value"]
    try:
        if args.command=="arm":
            before=await access("trace_info")
            await access("foc_trace",PROFILES[args.profile]+(3 if args.ring else 0))
            after=await access("trace_info")
            if after[19]==before[19]:raise RuntimeError("Capture arm was not accepted")
            print(json.dumps({"capture_id":after[19],"state":after[2],"profile":after[3],"frames":after[4]}))
        elif args.command=="stop":
            await access("foc_trace",0);print(json.dumps(await access("trace_info")))
        else:
            info=await access("trace_info")
            if info[2]!=2 or info[4]<=0:raise RuntimeError("Recorder must be frozen and nonempty; dump never stops/arms it automatically")
            frames=[]
            for page in range(info[4]):
                await access("trace_page",page);frames.append(await access("trace_data"))
            after=await access("trace_info")
            if any(info[i]!=after[i] for i in [2,3,4,5,6,19]):raise RuntimeError("Capture changed during export")
            result={"node":args.node,"info":info,"frames":frames}
            decode(result)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2)+"\n")
            print(f"Exported {len(frames)} immutable frames to {args.output}")
    finally:client.close();node.close()


def spectrum(values,fs):
    import numpy as np
    x=np.asarray(values,dtype=float)
    if len(x)<4 or not np.all(np.isfinite(x)):return np.array([]),np.array([])
    window=np.hanning(len(x));x=x-np.mean(x)
    power=abs(np.fft.rfft(x*window))**2/(fs*np.sum(window**2))
    if len(x)%2==0:power[1:-1]*=2
    else:power[1:]*=2
    return np.fft.rfftfreq(len(x),1/fs),np.sqrt(power)


def analyze(args):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    paths=[args.input]+([args.compare] if args.compare else [])
    args.output.mkdir(parents=True,exist_ok=True)
    captures=[];metrics=[]
    for path in paths:
        meta,rows=decode(json.loads(path.read_text()));n=len(rows);cpu=meta[8];expected=meta[9];stride=meta[10]
        if not cpu or not expected or not stride:raise ValueError("Missing capture timing")
        cycles=np.array([r['cycle_start'] for r in rows],dtype=np.uint64)
        dc=(np.diff(cycles.astype(np.int64))&0xffffffff)
        t=np.r_[0,np.cumsum(dc)]/cpu
        periods=np.array([r['period'] for r in rows],dtype=np.uint64)
        dp=(np.diff(periods.astype(np.int64))&0xffffffff)
        fs=cpu/(expected*stride)
        regular=bool(np.all(dp==stride) and (not len(dc) or np.max(abs(dc-expected*stride))<.1*expected*stride))
        status={name:int(sum(bool(r['flags']&mask) for r in rows)) for name,mask in FLAGS.items()}
        m={"input":str(path),"profile":meta[3],"frames":n,"sample_rate_hz_nominal":fs,
           "window_s":float(t[-1]) if len(t) else 0,"fft_resolution_hz":fs/n,
           "isr_cycles_max_capture":meta[16],"isr_us_max_frames":max(r['isr_cycles'] for r in rows)/cpu*1e6,
           "entry_timer_count_max":max(r['entry_timer_count'] for r in rows),
           "late_entries_capture":meta[11],"pending_updates_capture":meta[12],
           "sampled_flag_counts":status,"period_step_min":int(min(dp)) if len(dp) else None,
           "period_step_max":int(max(dp)) if len(dp) else None,"uniform_for_fft":regular,
           "decimation":stride,"warnings":[]}
        if stride>1:m['warnings'].append('Decimation has no added anti-alias filter; folding above reduced Nyquist is possible.')
        if not regular:m['warnings'].append('Missing/irregular periods: FFT suppressed; do not infer peaks from this capture.')
        if t[-1]<.1:m['warnings'].append('Short window cannot resolve low-frequency mechanical oscillations or observer settling.')
        data={k:np.array([r[k] for r in rows]) for k in rows[0]}
        if meta[3]==1:
            m['phase_sum_mean_a']=float(np.mean(data['phase_sum']));m['phase_sum_rms_a']=float(np.sqrt(np.mean(data['phase_sum']**2)))
            m['iq_raw_std_a']=float(np.std(data['iq_raw']));m['iq_filtered_std_a']=float(np.std(data['iq_filtered']))
            alpha=float_word(meta[28]);m['current_lpf_alpha']=alpha
            m['lpf_dc_group_delay_us']=(1-alpha)/alpha*expected/cpu*1e6 if 0<alpha<=1 else None
        elif meta[3]==2:
            error=data['iq_measured']-data['iq_target'];valid=np.isfinite(error)
            m['iq_error_rms_a']=float(np.sqrt(np.mean(error[valid]**2))) if np.any(valid) else None
        else:
            pp=data['pole_pairs'];err=(data['observer_prediction']-data['rotor_corrected']+np.pi)%(2*np.pi)-np.pi
            valid=(data['flags'].astype(np.uint32)&FLAGS['observer_valid'])!=0
            m['observer_prediction_vs_raw_electrical_rms_deg']=float(np.degrees(np.sqrt(np.mean((err[valid]*pp[valid])**2)))) if np.any(valid) else None
            m['observer_note']='Prediction-vs-raw is not independently measured electrical-zero error.'
            m['observer_g3']=float_word(meta[29])
        m['sample_offset_us_mean']=float(np.mean(data['sample_offset_cycles']))/cpu*1e6
        m['sample_offset_us_max']=float(np.max(data['sample_offset_cycles']))/cpu*1e6
        m['interval_us_min']=float(min(dc))/cpu*1e6 if len(dc) else None
        m['interval_us_max']=float(max(dc))/cpu*1e6 if len(dc) else None
        captures.append((path.stem,meta,t,data,regular,fs));metrics.append(m)
    (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2)+"\n")
    # Per-profile traces avoid fabricating signals absent from another profile.
    groups={1:[('ia','ib','ic'),('phase_sum',),('id_raw','id_filtered'),('iq_raw','iq_filtered')],
            2:[('iq_before_limits','iq_target','iq_measured'),('id_target','id_measured'),
               ('vd_pi_raw','vd_applied'),('vq_pi_raw','vq_applied','vq_limit')],
            3:[('rotor_corrected','observer_prediction','observer_corrected'),('observer_omega','predicted_omega'),
               ('innovation',),('electric_angle',)]}
    for profile in sorted(set(c[1][3] for c in captures)):
        selected=[c for c in captures if c[1][3]==profile]
        fig,axes=plt.subplots(6,1,figsize=(12,13),sharex=True)
        for label,meta,t,data,_,_ in selected:
            if profile==3:
                valid=(data['flags'].astype(np.uint32)&FLAGS['observer_valid'])!=0
                for key in ['observer_prediction','observer_corrected','observer_omega','predicted_omega','innovation']:
                    data[key]=np.where(valid,data[key],np.nan)
            for ax,names in zip(axes,groups[profile]):
                for name in names:ax.plot(t*1000,data[name],label=f'{label}: {name}')
            axes[4].plot(t*1000,data['isr_cycles']/meta[8]*1e6,label=f'{label}: ISR body, us')
            axes[5].plot(t*1000,data['entry_timer_count'],label=f'{label}: TIM1 entry count')
        for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize='small')
        axes[-1].set_xlabel('Time from first ISR sample, ms')
        fig.tight_layout();fig.savefig(args.output/f'profile-{profile}-traces.png',dpi=140);plt.close(fig)
        fig,ax=plt.subplots(figsize=(11,5))
        keys={1:['ia','iq_raw','iq_filtered'],2:['iq_measured','vq_applied'],3:['innovation']}[profile]
        for label,meta,t,data,regular,fs in selected:
            if not regular:continue
            for key in keys:
                frequency,amplitude=spectrum(data[key],fs)
                if len(frequency):ax.semilogy(frequency,np.maximum(amplitude,1e-15),label=f'{label}: {key}')
        ax.set_xlabel('Frequency, Hz');ax.set_ylabel('Amplitude spectral density (signal units / sqrt(Hz))')
        ax.grid(alpha=.25)
        if ax.lines:ax.legend()
        fig.tight_layout();fig.savefig(args.output/f'profile-{profile}-spectra.png',dpi=140);plt.close(fig)
    for label,meta,t,data,_,_ in captures:
        fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        if meta[3] in (1,2):
            ab=data['previous_pwm_ab' if meta[3]==1 else 'staged_pwm_ab'].astype(np.uint32)
            c=data['previous_pwm_c_and_sample_timer' if meta[3]==1 else 'staged_pwm_c'].astype(np.uint32)
            for name,value in [('CCR A',ab&65535),('CCR B',ab>>16),('CCR C',c&65535)]:
                axes[0].plot(t*1000,value/(meta[22]+1),label=name+' duty')
            if meta[3]==1:
                for key in ['adc_a','adc_b','adc_c']:axes[1].plot(t*1000,data[key],label=key)
                axes[2].plot(t*1000,c>>16,label='TIM1 count at current reads')
            else:
                for key in ['id_integral_as','iq_integral_as']:axes[1].plot(t*1000,data[key],label=key)
                axes[2].plot(t*1000,data['sample_offset_cycles']/meta[8]*1e6,label='Current read offset from ISR entry, us')
        else:
            err=(data['observer_prediction']-data['rotor_corrected']+np.pi)%(2*np.pi)-np.pi
            axes[0].plot(t*1000,np.degrees(err*data['pole_pairs']),label='Prediction minus rotor, electrical deg (not true zero error)')
            for key in ['cordic_cos_raw','cordic_sin_raw']:axes[1].plot(t*1000,data[key].astype(np.uint32).view(np.int32)/2**31,label=key)
            axes[2].plot(t*1000,data['observer_accel'],label='Observer acceleration, rotor rad/s²')
        for ax in axes:
            ax.grid(alpha=.25)
            if ax.lines:ax.legend(fontsize='small')
        axes[-1].set_xlabel('Time from first ISR sample, ms')
        fig.tight_layout();fig.savefig(args.output/f'{label}-details.png',dpi=140);plt.close(fig)
    print(json.dumps(metrics,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    for command in ['arm','stop','dump']:
        p=sub.add_parser(command);p.add_argument('--node',type=int,required=True)
        if command=='arm':p.add_argument('--profile',choices=PROFILES,required=True);p.add_argument('--ring',action='store_true')
        if command=='dump':p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('analyze');p.add_argument('input',type=Path);p.add_argument('--compare',type=Path);p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='analyze':analyze(args)
    else:asyncio.run(device(args))


if __name__=='__main__':main()
