#include "service_motion.hpp"
#include "command_supervisor.hpp"
#include "output_state_observer.hpp"
#include <cassert>
#include <cmath>
#include <limits>
extern "C" bool encoder_calibration_storage_load(uint8_t,encoder_calibration_data*) { return false; }
extern "C" bool encoder_calibration_storage_save(uint8_t,encoder_calibration_data*) { return true; }
using namespace ruka3;
static bool near(float a,float b,float e=.0001F) { return std::fabs(a-b)<e; }
static MechanicalCalibrationSnapshot limits() {
    MechanicalCalibrationSnapshot l{}; l.limits_valid=true;
    l.lower_limit=-2; l.upper_limit=2; l.soft_lower_limit=-1.8F; l.soft_upper_limit=1.8F;
    return l;
}
static void profile(float goal,std::uint32_t start) {
    ServiceMotion m(kJ4CommandTuning); auto l=limits();
    assert(m.position(goal,0,l,start));
    float previous=0,previous_velocity=0;
    bool completed=false;
    for(std::uint32_t elapsed=0;elapsed<=8000;++elapsed) {
        if (!m.update(start+elapsed,previous,l)) continue;
        const auto& p=m.snapshot();
        assert(std::isfinite(p.target) && std::isfinite(p.velocity));
        assert(std::fabs(p.velocity)<=.5001F);
        assert((p.target-previous)*(goal<0?-1:1)>=-.00001F);
        assert(std::fabs(p.target)<=std::fabs(goal)+.0001F);
        assert(std::fabs(p.velocity-previous_velocity)<=.0101F);
        previous=p.target; previous_velocity=p.velocity;
        if(!m.trajectory_active()) { assert(near(p.target,goal)); assert(p.velocity==0); completed=true; break; }
    }
    assert(completed);
}
int main() {
    {
        ServiceMotion slow(kDefaultCommandTuning);auto bounds=limits();
        assert(slow.set_profile_velocity(.05F));assert(slow.set_profile_acceleration(.2F));
        assert(!slow.set_profile_velocity(0));assert(!slow.set_profile_acceleration(2));
        assert(slow.position(.1F,0,bounds,0));
        // Changing settings during a move must not distort the active profile.
        assert(slow.set_profile_acceleration(.5F));
        float previous=0;
        for(unsigned t=0;t<=3000;t+=10) {
            slow.update(t,0,bounds);const auto v=slow.snapshot().velocity;
            assert(std::fabs(v)<=.05001F && std::fabs(v-previous)<=.00201F);previous=v;
        }
        assert(!slow.trajectory_active() && near(slow.snapshot().target,.1F));
    }

    for(float d:{0.0F,.04F,-.04F,1.0F,-1.0F}) { profile(d,100); profile(d,0xFFFFFF00U); }
    auto l=limits(); ServiceMotion m(kJ4CommandTuning);
    assert(m.position(1,0,l,0));
    for(unsigned t=0;t<=2500;t+=10) {
        if(t%100==0) assert(m.position(1,0,l,t)); // RPC repeats must not restart acceleration.
        m.update(t,0,l);
    }
    assert(!m.trajectory_active() && near(m.snapshot().target,1));
    assert(m.position(-1,.3F,l,3000)); m.update(3000,.3F,l); assert(near(m.snapshot().target,.3F));
    assert(m.direct(ServiceMotionMode::current,.2F,3,l,3010)); assert(!m.trajectory_active());
    m.update(6000,.3F,l); assert(m.snapshot().mode==ServiceMotionMode::current);
    assert(m.servo(.4F,.1F,.2F,5,0,.3F,l,6010)); m.update(6010,.3F,l);
    assert(near(m.snapshot().error,.1F) && m.snapshot().kp==5);
    m.hold(.35F,6020); m.update(6020,.35F,l); assert(m.snapshot().mode==ServiceMotionMode::hold);
    m.cancel(); assert(!m.update(6030,.35F,l) && !m.trajectory_active());
    assert(!m.position(std::numeric_limits<float>::quiet_NaN(),0,l,0));
    assert(!m.position(2.1F,0,l,0));
    assert(!m.servo(0,0,0,-1,0,0,l,0));
    assert(!m.direct(ServiceMotionMode::current,3.1F,3,l,0));
    for(float sign:{1.0F,-1.0F}) {
        assert(soft_limited_target(sign*1.95F,sign*1.9F,l)==sign*1.9F);
        assert(soft_limited_target(sign*1.85F,sign*1.9F,l)==sign*1.85F);
        assert(soft_limited_target(sign*1.95F,sign*1.7F,l)==sign*1.95F);
        assert(near(outward_command_scale(sign,sign*1.9F,l),.5F));
        assert(outward_command_scale(-sign,sign*2.1F,l)==1);
        assert(outward_command_scale(sign,sign*2.1F,l)==0);
        assert(m.position(sign*1.85F,sign*2.1F,l,0));
        m.update(10,sign*2.05F,l); // Mechanism ahead of inward trajectory: don't cancel retreat.
        assert(m.trajectory_active() && m.snapshot().kp==kJ4CommandTuning.default_position_kp);
        for(unsigned t=20;t<=2000;t+=10) m.update(t,sign*2.05F,l);
        assert(near(m.snapshot().target,sign*1.85F));
        assert(m.position(sign*1.95F,sign*1.9F,l,3000));
        m.update(3000,sign*1.9F,l); assert(near(m.snapshot().target,sign*1.9F));
        assert(m.snapshot().kp==kJ4CommandTuning.default_position_kp); // Holding stiffness is not faded at limits.
    }
    // Hybrid Cyphal position control: threshold is public output radians.
    for(float sign:{-1.0F,1.0F}) {
        ServiceMotion stream(kJ4CommandTuning,3);
        assert(near(kJ4CommandTuning.acquire_threshold_rad,.1F));
        for(float distance:{.099F,.1F}) {
            stream.cancel();assert(stream.servo(sign*distance,.03F,.1F,2,.01F,0,l,0));
            stream.update(0,0,l);assert(!stream.trajectory_active());
            assert(near(stream.snapshot().target,sign*distance));
            assert(near(stream.snapshot().velocity,.03F));
        }
        stream.cancel();assert(stream.servo(sign*.3F,0,.1F,2,0,0,l,0));
        assert(stream.trajectory_active() && !stream.autonomous_motion_active());
        float previous=0;
        for(unsigned t=0;t<=1200;t+=5) {
            // Updating gains/feedforward must not restart the same goal's ramp,
            // including after the actual position enters the 0.1-rad band.
            assert(stream.servo(sign*.3F,0,.2F,3,.02F,previous,l,t));
            if(!stream.update(t,previous,l)) continue;
            const auto& command=stream.snapshot();
            assert(command.kp==3 && command.kd==.02F && command.torque==.2F);
            assert((command.target-previous)*sign>=-.00001F);
            assert(std::fabs(command.velocity)<=.5001F);
            if(t<1000) assert(std::fabs(command.target)<.3F);
            previous=command.target;
        }
        assert(!stream.trajectory_active() && near(stream.snapshot().target,sign*.3F));
        // New nearby goal is direct; a new far goal replans from measured angle.
        assert(stream.servo(sign*.35F,0,0,3,0,sign*.3F,l,1210));
        stream.update(1210,sign*.3F,l);assert(!stream.trajectory_active());
        assert(near(stream.snapshot().target,sign*.35F));
        assert(stream.servo(-sign*.3F,0,0,3,0,sign*.35F,l,1220));
        stream.update(1220,sign*.35F,l);assert(stream.trajectory_active());
        assert(near(stream.snapshot().target,sign*.35F));
        // A stream trajectory must not suppress timeout as autonomous set_pos does.
        CommandSupervisor timeout;timeout.initialize(1220,1000);timeout.note_accepted_command(1220);
        assert(timeout.update(2221,stream.autonomous_motion_active()));
        stream.hold(sign*.2F,2221);assert(!stream.trajectory_active());
        // Even a tiny register set_pos still uses a trajectory.
        assert(stream.position(sign*.01F,0,l,2230));
        assert(stream.trajectory_active() && stream.autonomous_motion_active());
    }
    // Capture is not cancelled or restarted by small ROS reference steps.
    // After the capture ends, packets go straight to PD even if the axis still lags.
    for (unsigned joint=1; joint<=6; ++joint) for (float sign : {-1.0F, 1.0F}) {
        const auto& tuning=joint_profile(joint).command;
        ServiceMotion stream(tuning,2);
        const float start=sign*.2F;
        assert(stream.set_integral(.2F,.5F));
        stream.hold(start,0);stream.update(0,start-.01F,l);
        stream.compensate_load(0,start-.01F,0,2,l);
        stream.compensate_load(100,start-.01F,0,2,l);
        const float learned=stream.integral_current();assert(learned>0);
        assert(stream.servo(0,0,0,3,.02F,start,l,100));
        for(unsigned elapsed=0;elapsed<=200;elapsed+=10) {
            const float reference=-sign*.00001F*elapsed;
            assert(stream.servo(reference,-sign*.001F,0,3,.02F,start,l,100+elapsed));
            stream.update(100+elapsed,start,l);
            assert(stream.trajectory_active());
            // This is the original acceleration, not a new ramp or a direct jump.
            assert(near(stream.snapshot().target,start-sign*.5F*elapsed*elapsed*.000001F));
            assert(near(stream.integral_current(),learned));
        }
        stream.update(1100,0,l);assert(!stream.trajectory_active());
        // Direct sampled motion, reversal, repeated position and changed velocity.
        for(unsigned t=1110;t<=1210;t+=10) {
            const float q=sign*.0001F*(t-1110);
            const float v=t<1160?sign*.01F:-sign*.01F;
            assert(stream.servo(q,v,.03F,3,.02F,q-sign*.02F,l,t));
            assert(stream.update(t,q-sign*.02F,l));
            assert(!stream.trajectory_active());
            assert(near(stream.snapshot().target,q));
            assert(near(stream.snapshot().velocity,v));
            assert(near(position_current_request(stream.snapshot(),0,1,1),3*sign*.02F+.02F*v+.03F));
            assert(near(stream.integral_current(),learned));
        }
        // v_ref=0 still corrects a position error; no prediction between packets.
        assert(stream.servo(.02F,0,0,3,.02F,0,l,1220));stream.update(1220,0,l);
        assert(near(position_current_request(stream.snapshot(),0,1,1),.06F));
        stream.update(1500,0,l);assert(near(stream.snapshot().target,.02F));
        // Residual lag cannot repeatedly erase position error and holding torque.
        assert(stream.servo(.021F,.01F,0,3,.02F,-.2F,l,1510));
        stream.update(1510,-.2F,l);
        assert(!stream.trajectory_active());
        assert(near(stream.snapshot().target,.021F));
        // A new discontinuous far command still captures once.
        assert(stream.servo(.8F,0,0,3,.02F,-.2F,l,1520));
        assert(stream.trajectory_active());
        stream.hold(-.2F,2520);stream.update(2520,-.2F,l);
        assert(!stream.trajectory_active() && stream.snapshot().velocity==0);
        assert(near(stream.integral_current(),learned));
    }
    // Regression from J5: constant ROS target, loaded/stalled axis >0.1 rad away.
    // Completing acquisition must not start another ramp every ~0.8 seconds.
    for(unsigned joint=1;joint<=6;++joint) for(float sign:{-1.0F,1.0F}) {
        ServiceMotion stream(joint_profile(joint).command,2);
        const float measured=sign*.2F;
        assert(stream.servo(0,0,0,30,0,measured,l,0));
        stream.update(1000,measured,l);
        assert(!stream.trajectory_active());
        for(unsigned t=1010;t<=20000;t+=10) {
            assert(stream.servo(0,0,0,30,0,measured,l,t));
            stream.update(t,measured,l);
            assert(!stream.trajectory_active());
            assert(stream.snapshot().target==0 && stream.snapshot().velocity==0);
            assert(near(stream.snapshot().error,-measured));
            assert(near(position_current_request(stream.snapshot(),0,1,1),-30*measured));
        }
        // Continuous samples retain full feedback despite persistent lag.
        for(unsigned t=20010;t<=21000;t+=10) {
            const float goal=sign*(t-20000)*.00001F;
            assert(stream.servo(goal,sign*.01F,0,30,.02F,measured,l,t));
            stream.update(t,measured,l);
            assert(!stream.trajectory_active() && near(stream.snapshot().target,goal));
            assert(near(stream.snapshot().velocity,sign*.01F));
        }
        // Watchdog hold ends that stream; the next far initial goal captures again.
        stream.hold(measured,22001);stream.update(22001,measured,l);
        assert(stream.servo(0,0,0,30,0,measured,l,22010));
        assert(stream.trajectory_active() && !stream.autonomous_motion_active());
    }
    // A finite set_pos owns its final target as well as the trajectory.
    // Reproduce J5 sag after completion; watchdog must not adopt the sagged angle.
    for(unsigned joint=1;joint<=6;++joint) for(float sign:{-1.0F,1.0F}) {
        ServiceMotion motion(joint_profile(joint).command,2);
        CommandSupervisor watchdog;watchdog.initialize(0,1000);
        assert(motion.position(sign*.3F,0,l,0));watchdog.note_accepted_command(0);
        for(unsigned t=0;t<=10000;t+=10) {
            const float measured=t<1200?0:sign*.15F;
            motion.update(t,measured,l);
            if(watchdog.update(t,motion.autonomous_motion_active())) motion.hold(measured,t);
            if(t>=1200) {
                assert(!motion.trajectory_active());
                assert(motion.snapshot().mode==ServiceMotionMode::position);
                assert(near(motion.snapshot().target,sign*.3F));
                assert(near(motion.snapshot().error,sign*.15F));
                assert(!watchdog.snapshot().timed_out);
            }
        }
        // A stream then loses its lease and captures actual position instead.
        assert(motion.servo(sign*.2F,0,0,5,0,sign*.15F,l,10010));
        watchdog.note_accepted_command(10010);
        assert(!watchdog.update(11010,motion.autonomous_motion_active()));
        assert(watchdog.update(11011,motion.autonomous_motion_active()));
        motion.hold(sign*.17F,11011);motion.update(11011,sign*.17F,l);
        assert(near(motion.snapshot().target,sign*.17F));
        assert(motion.snapshot().mode==ServiceMotionMode::hold);
        // Direct debug modes retain the normal command timeout as well.
        assert(motion.direct(ServiceMotionMode::current,.1F,6,l,12000));
        watchdog.note_accepted_command(12000);
        assert(!watchdog.update(12000,motion.autonomous_motion_active()));
        assert(watchdog.update(13001,motion.autonomous_motion_active()));
    }
    // Runtime register gains belong to position/hold; packet gains belong to the stream.
    ServiceMotion gains(kJ4CommandTuning);
    assert(gains.position_kp()==kJ4CommandTuning.default_position_kp && gains.position_kd()==0);
    assert(gains.set_position_gains(3,.02F));
    assert(gains.position(.2F,0,l,0)); gains.update(100,0,l);
    const float progress=gains.snapshot().target;
    assert(progress>0 && progress<.2F && gains.trajectory_active());
    assert(gains.snapshot().kp==3 && gains.snapshot().kd==.02F);
    assert(gains.set_position_gains(6,.03F));
    assert(gains.snapshot().kp==6 && gains.snapshot().kd==.03F);
    assert(gains.snapshot().target==progress && gains.trajectory_active());
    assert(gains.position(.2F,0,l,101)); // Same target must retain the ramp with new gains.
    gains.update(200,0,l); assert(gains.snapshot().target>progress);
    assert(gains.servo(.1F,.2F,.3F,2,.01F,0,l,210));
    gains.update(210,0,l);
    assert(!gains.trajectory_active() && gains.snapshot().target==.1F);
    assert(gains.snapshot().kp==2 && gains.snapshot().kd==.01F);
    assert(gains.set_position_gains(8,.04F));
    assert(gains.snapshot().kp==2 && gains.snapshot().kd==.01F); // No global override of packet gains.
    assert(gains.position_kp()==8 && gains.position_kd()==.04F);
    gains.hold(.1F,220);gains.update(220,.1F,l);
    assert(gains.snapshot().kp==8 && gains.snapshot().kd==.04F);
    assert(gains.servo(.12F,0,0,4,.02F,.1F,l,230));gains.update(230,.1F,l);
    assert(gains.snapshot().kp==4 && gains.snapshot().kd==.02F);
    gains.cancel();assert(gains.position(.2F,.1F,l,240));
    assert(gains.snapshot().target==.1F && gains.trajectory_active());
    assert(gains.snapshot().kp==8 && gains.snapshot().kd==.04F);
    assert(!gains.set_position_gains(-1,0));
    assert(!gains.set_position_gains(1,-1));
    assert(!gains.set_position_gains(std::numeric_limits<float>::quiet_NaN(),0));
    assert(!gains.set_position_gains(1,std::numeric_limits<float>::infinity()));
    assert(gains.position_kp()==8 && gains.position_kd()==.04F);
    ServiceMotion rebooted(kJ4CommandTuning);
    assert(rebooted.position_kp()==kJ4CommandTuning.default_position_kp && rebooted.position_kd()==0); // No EEPROM for tuning gains.
    // Load compensation is in amperes, independent of board Kt and gearing.
    for (float sign : {-1.0F, 1.0F}) {
        ServiceMotion load(kJ4CommandTuning, 3);
        assert(load.position_ki()==kJ4CommandTuning.default_position_ki && load.integral_current()==0 && load.integral_limit()==kJ4CommandTuning.default_integral_current_limit_a);
        load.hold(0,0); load.update(0,-sign*.1F,l);
        assert(load.compensate_load(0,-sign*.1F,0,3,l)==0); // Initial call establishes the integration clock.
        assert(load.set_integral(1,.5F));
        for(unsigned t=10;t<=1000;t+=10) {
            load.update(t,-sign*.1F,l);
            load.compensate_load(t,-sign*.1F,0,3,l);
        }
        assert(near(load.integral_current(),sign*.1F));
        load.update(1010,0,l);
        assert(near(load.compensate_load(1010,0,0,3,l),sign*.1F)); // Zero error retains load.
        // Current saturation prevents further winding, but permits unwinding.
        load.update(1020,-sign*.1F,l);
        load.compensate_load(1020,-sign*.1F,sign*3,3,l);
        assert(near(load.integral_current(),sign*.1F));
        load.update(1030,sign*.1F,l);
        load.compensate_load(1030,sign*.1F,sign*3,3,l);
        assert(near(load.integral_current(),sign*.099F));
        assert(load.set_integral(1,.05F)); assert(near(load.integral_current(),sign*.05F));
        // Moving trajectory preserves compensation without learning a motion error.
        assert(load.position(sign*.5F,0,l,1040));load.update(1100,0,l);
        load.compensate_load(1100,0,0,3,l);assert(near(load.integral_current(),sign*.05F));
        // Stream packet replacement and watchdog hold retain the learned load.
        assert(load.servo(0,0,0,2,0,0,l,1110));load.update(1110,0,l);
        load.compensate_load(1110,0,0,3,l);assert(near(load.integral_current(),sign*.05F));
        load.hold(0,1120);load.update(1120,0,l);
        load.compensate_load(1120,0,0,3,l);assert(near(load.integral_current(),sign*.05F));
        // A hard limit removes outward learned current, with no outward buildup.
        load.update(1130,sign*2,l);
        assert(load.compensate_load(1130,sign*2,0,3,l)==0);
        assert(load.integral_current()==0);
        load.cancel();assert(load.integral_current()==0 && load.position_ki()==1);
        assert(load.direct(ServiceMotionMode::current,0,3,l,1200));assert(load.integral_current()==0);
        assert(!load.set_integral(-1,.5F));assert(!load.set_integral(1,-.5F));
        assert(!load.set_integral(1,3.01F));
        assert(!load.set_integral(std::numeric_limits<float>::quiet_NaN(),1));
        assert(!load.set_integral(1,std::numeric_limits<float>::infinity()));
        assert(load.set_integral(0,.5F));assert(load.integral_current()==0);
    }
    // Same stationary stream at 50 and 200Hz learns the same current (real dt).
    for(unsigned step:{5U,20U}) {
        ServiceMotion stream(kJ4CommandTuning,3);assert(stream.set_integral(1,1));
        for(unsigned t=0;t<=1000;t+=step) {
            assert(stream.servo(.1F,0,0,0,0,0,l,t));stream.update(t,0,l);
            stream.compensate_load(t,0,0,3,l);
        }
        assert(near(stream.integral_current(),.1F));
    }
    // Unsigned clock rollover and disabling Ki must not retain learned current.
    {
        ServiceMotion rollover(kJ4CommandTuning,3);rollover.set_integral(1,1);
        rollover.hold(0,0xFFFFFFF0U);rollover.update(0xFFFFFFF0U,-.1F,l);
        rollover.compensate_load(0xFFFFFFF0U,-.1F,0,3,l);
        rollover.update(4,-.1F,l);rollover.compensate_load(4,-.1F,0,3,l);
        assert(near(rollover.integral_current(),.002F));
        assert(rollover.set_integral(0,1));assert(rollover.integral_current()==0);
    }
    // A quasi-static loaded plant settles at the goal with non-zero Iq.
    for(float sign:{-1.0F,1.0F}) {
        ServiceMotion plant(kJ4CommandTuning,3);plant.set_integral(2,1);plant.hold(0,0);
        float position=0;
        for(unsigned t=0;t<=20000;t+=10) {
            plant.update(t,position,l);
            const float pd=-5*position;
            const float integral=plant.compensate_load(t,position,pd,3,l);
            position+=(pd+integral-sign*.5F)*.01F;
        }
        assert(std::fabs(position)<.0001F);
        assert(near(plant.integral_current(),sign*.5F,.001F));
        plant.cancel();assert(plant.integral_current()==0);
    }
    // Both adapters implement output-shaft Nm gains despite different vendor P/D scaling.
    for (unsigned j=1; j<=6; ++j) {
        const auto& tuning=joint_profile(j).command;
        ServiceMotion boot(tuning,3);
        assert(boot.position_kp()==(j<=4?20:5) && boot.position_kd()==0 && boot.position_ki()==(j<=4?2:0));
    }
    // Public Kp=3 requests 0.3 Nm for a 0.1 rad output error on every joint.
    for (unsigned j=1; j<=6; ++j) {
        const auto& profile=joint_profile(j);
        ServiceMotionSnapshot command;command.kp=3;command.error=.1F;
        const float iq=position_current_request(command,0,profile.motor.torque_const,profile.gear_ratio);
        assert(near(iq*profile.motor.torque_const*profile.gear_ratio,.3F,.000001F));
    }
    for(float gear:{10.0F,36.0F,72.0F}) for(float kt:{.011F,1.0F})
    for(float direction:{-1.0F,1.0F}) for(float error:{-6.0F,-.1F,.0F,.1F,6.0F}) {
        ServiceMotionSnapshot cmd;cmd.error=error;cmd.velocity=.2F;cmd.torque=-.08F;cmd.kp=20;cmd.kd=.3F;
        constexpr float measured=-.1F, integral=.15F;
        const float expected=position_current_request(cmd,measured,kt,gear)+integral;
        const float native_ff=direction*(cmd.torque+integral*kt*gear);
        const float vbdrive_iq=((cmd.kp/gear)*(direction*error)+
            (cmd.kd/gear)*(direction*(cmd.velocity-measured))+native_ff/gear)/kt;
        const auto local=wrapped_position_target(cmd);
        const float g474_iq=((cmd.kp/gear)*direction*local.angle_error+
            (cmd.kd/gear)*direction*(cmd.velocity-measured)+direction*local.torque/gear+
            direction*integral*kt)/kt;
        const float tolerance=.0001F+std::fabs(expected)*.000002F;
        assert(near(direction*vbdrive_iq,expected,tolerance));
        assert(near(direction*g474_iq,expected,tolerance));
    }
    // The existing G474 wrapped FOC must retain the unwrapped public PD law.
    for(float error:{-6.0F,-3.14F,-.03F,0.0F,.03F,3.14F,6.0F}) {
        ServiceMotionSnapshot command; command.error=error; command.kp=5; command.torque=.2F;
        const auto local=wrapped_position_target(command);
        assert(std::fabs(local.angle_error)<=1.570797F);
        for(float sign:{1.0F,-1.0F}) for(float drift:{-.1F,0.0F,.1F}) {
            const float native_torque=sign*local.torque+command.kp*
                std::remainder(sign*(local.angle_error-drift),6.28318530718F);
            assert(near(sign*native_torque, command.torque+command.kp*(error-drift)));
        }
    }
    CommandSupervisor supervisor; supervisor.initialize(0,1000); supervisor.note_accepted_command(0);
    assert(!supervisor.update(2000,true));
    supervisor.note_accepted_command(2500);
    assert(!supervisor.update(3000)); assert(supervisor.update(3501));
    assert(supervisor.snapshot().accepted_count==2);
    m.hold(.4F,3501);m.update(3501,.4F,l); assert(near(m.snapshot().target,.4F));
    m.cancel(); assert(!m.update(4000,0,l));
    // J1/J4 encoder correction is a persistent 10% weight, not a slower 100% pull.
    assert(kJ4ObserverProfile.fusion_encoder_weight==.1F);
    assert(kJ1ObserverProfile.fusion_encoder_weight==.1F);
    for(unsigned joint:{2U,3U,5U,6U}) assert(joint_profile(joint).observer.fusion_encoder_weight==1);
    constexpr float tick_rad=6.28318530718F/16384;
    for(float weight:{0.0F,.1F,1.0F}) for(int sign:{-1,1}) {
        OutputStateObserver obs({16384,0,static_cast<std::int8_t>(sign),.1F,2,0,weight});
        obs.update(0,0,0);
        for(unsigned t=1;t<=1000;++t) obs.update(t,1000,0);
        const float expected=sign*1000*tick_rad*weight;
        assert(std::fabs(obs.snapshot().fused_angle-expected)<=2.01F*tick_rad);
        const float stable=obs.snapshot().fused_angle;
        for(unsigned t=1001;t<=2000;++t) obs.update(t,1000,0);
        assert(near(obs.snapshot().fused_angle,stable));
        // Encoder wrap and a complete motor turn must not reset the motor anchor.
        OutputStateObserver rotating({16384,0,static_cast<std::int8_t>(sign),.1F,2,0,weight});
        rotating.update(0,16000,0);
        const float start=rotating.snapshot().fused_angle;
        for(unsigned i=1;i<=180;++i) {
            const float motor=static_cast<float>(sign)*i*100*tick_rad;
            rotating.update(i*10,(16000+i*100+1000)%16384,motor);
            const float expected_angle=start+motor+sign*1000*tick_rad*weight;
            assert(std::fabs(rotating.snapshot().fused_angle-expected_angle)<=2.02F*tick_rad);
        }
        rotating.set_encoder_zero(2000);rotating.update(2000,2000,1);
        assert(near(rotating.snapshot().fused_angle,0));
    }
    // Shared observer recovers signed, calibrated angles across the raw encoder wrap.
    for(int sign:{1,-1}) {
        OutputStateObserver obs({16384,0,static_cast<std::int8_t>(sign),.1F,2,0});
        obs.set_calibrated_limits(-2,2);obs.update(0,16380,0);obs.update(10,4,sign*.003F);
        assert(std::fabs(obs.snapshot().fused_angle)<.01F);
        assert(obs.snapshot().fused_velocity*sign>0);
    }
}
