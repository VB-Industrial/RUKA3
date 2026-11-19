#include "foc_trace.hpp"
#include <cassert>
#include <limits>
using namespace ruka3::foc_trace;
int main() {
    DVoltagePulse pulse;
    assert(!pulse.arm(0) && !pulse.arm(.121F));
    assert(!pulse.arm(std::numeric_limits<float>::quiet_NaN()));
    assert(pulse.arm(.06F) && !pulse.arm(.06F));
    for(unsigned i=0;i<256;++i) {
        float vd=100,vq=100;
        const auto flags=pulse.step(vd,vq);
        assert(flags&d_pulse_window);
        const bool on=i>=64 && i<128;
        assert(vd==(on?.06F:0) && vq==0);
        assert(bool(flags&d_pulse_energized)==on);
    }
    assert(!pulse.active());
    float untouched=10,q=20;assert(pulse.step(untouched,q)==0 && untouched==10 && q==20);
    assert(pulse.arm(-.12F));pulse.cancel();assert(!pulse.active());

    Recorder<3> r;
    assert(!r.frame(0));
    assert(!r.arm(static_cast<Profile>(0),false));
    assert(!r.arm(Profile::control,false,0));
    assert(r.arm(Profile::measurement,false));
    assert(!r.clear()); assert(!r.arm(Profile::angle,false));
    for(unsigned i=0;i<3;++i) {
        r.begin(0xfffffff0U+i*100,12,100);
        assert(!r.frame(0));
        r.current()->f(12,-1.25f);
        r.current()->words[10]|=dma_moved;
        r.finish(0xfffffff0U+i*100+25,i==2);
    }
    assert(r.state()==State::frozen && r.count()==3);
    assert(r.frame(0)->words[0]==1 && r.frame(2)->words[0]==3);
    assert(r.frame(1)->words[2]==100); // DWT wrap
    assert(std::bit_cast<float>(r.frame(0)->words[12])==-1.25f);
    assert(r.dma_changed_count==3 && r.pending_count==1 && r.max_cycles==25);
    const auto snapshot=r.frame(0)->words;
    r.begin(300,0,100);r.finish(320,false);
    assert(r.frame(0)->words==snapshot);
    assert(!r.frame(3));
    assert(r.arm(Profile::control,true,2));
    for(unsigned i=0;i<10;++i) {r.begin(500+i*100,0,100);r.finish(510+i*100,false);}
    assert(r.state()==State::capturing && r.count()==3 && !r.frame(0));
    r.stop();
    assert(r.frame(0)->words[0]==9 && r.frame(2)->words[0]==13);
    assert(r.stride()==2 && r.late_count==1);
    assert(r.clear() && r.count()==0 && !r.frame(0));
}
