#pragma once
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace ruka3::foc_trace {
inline constexpr std::uint32_t magic = 0x46544331U;
inline constexpr std::uint32_t schema = 1;
inline constexpr std::size_t words_per_frame = 32;
enum class Profile : std::uint32_t { measurement = 1, control = 2, angle = 3 };
enum class State : std::uint32_t { idle, capturing, frozen };
enum Flag : std::uint32_t {
    pwm_on=1U, mode_shift=1U, d_saturated=1U<<4, q_saturated=1U<<5,
    vector_limited=1U<<6, pwm_clipped=1U<<7, dma_moved=1U<<8,
    adc_overrun=1U<<9, dma_error=1U<<10, update_pending=1U<<11,
    late_entry=1U<<12, current_loop_ran=1U<<13, cordic_min=1U<<14, pwm_moe_end=1U<<15, observer_valid=1U<<16, d_pulse_window=1U<<17, d_pulse_energized=1U<<18,
};
struct Frame {
    std::array<std::uint32_t, words_per_frame> words{};
    void f(std::size_t i, float value) { words[i]=std::bit_cast<std::uint32_t>(value); }
};
static_assert(sizeof(Frame)==128);

// Diagnostic only: 4 ms baseline, 4 ms Vd pulse, 8 ms zero at 16 kHz.
// Arm/cancel from main under the same IRQ lock as the recorder.
class DVoltagePulse {
public:
    static constexpr unsigned pre_cycles=64,pulse_cycles=64,total_cycles=256;
    bool arm(float volts) {
        if(active_ || !std::isfinite(volts) || volts==0 || std::fabs(volts)>0.12F) return false;
        voltage_=volts;cycle_=0;active_=true;return true;
    }
    bool active() const { return active_; }
    float voltage() const { return active_?voltage_:0.0F; }
    void cancel() { active_=false; }
    std::uint32_t step(float& vd,float& vq) {
        if(!active_) return 0;
        const bool energized=cycle_>=pre_cycles && cycle_<pre_cycles+pulse_cycles;
        vd=energized?voltage_:0.0F;vq=0.0F;
        if(++cycle_==total_cycles) active_=false;
        return d_pulse_window|(energized?d_pulse_energized:0U);
    }
private:
    float voltage_{};
    unsigned cycle_{};
    bool active_{};
};

// One ISR producer. Main-loop arm/stop/info must be protected against IRQs;
// frozen frame reads need no lock, since arm requests are serialized in main.
template<std::size_t Capacity> class Recorder {
public:
    static_assert(Capacity>0);
    static constexpr std::size_t capacity=Capacity;
    State state() const { return state_; }
    Profile profile() const { return profile_; }
    Frame* current() { return writing_; }
    bool arm(Profile profile, bool ring, std::uint32_t decimation=1) {
        if (state_==State::capturing || decimation==0 || static_cast<unsigned>(profile)<1 || static_cast<unsigned>(profile)>3) return false;
        profile_=profile; ring_=ring; stride_=decimation; head_=count_=capture_periods_=0;
        late_count=pending_count=dma_changed_count=overrun_observations=dma_error_observations=0;
        max_cycles=max_interval=0; ++capture_id; writing_=nullptr; state_=State::capturing; return true;
    }
    void stop() { writing_=nullptr; state_=State::frozen; }
    bool clear() {
        if (state_==State::capturing) return false;
        count_=head_=0; writing_=nullptr; state_=State::idle; return true;
    }
    void begin(std::uint32_t cycles, std::uint32_t timer_count, std::uint32_t expected_cycles) {
        ++period_; start_=cycles;
        const auto interval=have_previous_?cycles-previous_:0; previous_=cycles; have_previous_=true;
        writing_=nullptr;
        if (state_!=State::capturing) return;
        if(interval>max_interval) max_interval=interval;
        const bool late=interval>expected_cycles+expected_cycles/2;
        if(late) ++late_count;
        if(capture_periods_++%stride_!=0) return;
        writing_=&frames_[head_]; writing_->words.fill(0);
        writing_->words[0]=period_; writing_->words[1]=cycles; writing_->words[2]=interval;
        writing_->words[5]=timer_count;
        if(late) writing_->words[10]|=Flag::late_entry;
    }
    void finish(std::uint32_t cycles, bool pending) {
        if(state_!=State::capturing) return;
        const auto elapsed=cycles-start_; if(elapsed>max_cycles) max_cycles=elapsed;
        if(pending) ++pending_count;
        if(!writing_) return;
        auto& w=writing_->words; w[3]=elapsed;w[4]=max_cycles;
        if(pending) w[10]|=Flag::update_pending;
        if(w[10]&Flag::dma_moved) ++dma_changed_count;
        if(w[10]&Flag::adc_overrun) ++overrun_observations;
        if(w[10]&Flag::dma_error) ++dma_error_observations;
        head_=(head_+1)%Capacity; if(count_<Capacity) ++count_;
        writing_=nullptr;
        if(!ring_ && count_==Capacity) state_=State::frozen;
    }
    const Frame* frame(std::size_t index) const {
        if(state_!=State::frozen || index>=count_) return nullptr;
        return &frames_[(head_+Capacity-count_+index)%Capacity];
    }
    std::uint32_t count() const { return count_; }
    std::uint32_t period() const { return period_; }
    std::uint32_t stride() const { return stride_; }
    std::uint32_t capture_id{},max_cycles{},max_interval{},late_count{},pending_count{},
        dma_changed_count{},overrun_observations{},dma_error_observations{};
private:
    std::array<Frame,Capacity> frames_{};
    volatile State state_{State::idle};
    Profile profile_{Profile::measurement};
    std::uint32_t head_{},count_{},period_{},capture_periods_{},start_{},previous_{},stride_{1};
    bool ring_{},have_previous_{};
    Frame* writing_{};
};
} // namespace ruka3::foc_trace
