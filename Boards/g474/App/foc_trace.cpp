#include "foc_trace_hw.hpp"
#ifdef RUKA3_FOC_TRACE
namespace ruka3::foc_trace {
Recorder<RUKA3_FOC_TRACE_FRAMES> recorder;
std::uint32_t selected_page{};
DVoltagePulse d_pulse;
std::uint32_t previous_pwm[3]{};
static std::uint32_t latched_pwm[3]{};
static std::uint32_t irq_start{};
}
extern "C" void ruka3_foc_trace_init() {
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk; // Do not reset a shared cycle clock.
}
extern "C" void ruka3_foc_trace_irq_begin() {
    using namespace ruka3::foc_trace;
    const auto cycles=DWT->CYCCNT;irq_start=cycles;const auto count=TIM1->CNT;
    for(unsigned i=0;i<3;++i) previous_pwm[i]=latched_pwm[i];
    latched_pwm[0]=TIM1->CCR1;latched_pwm[1]=TIM1->CCR2;latched_pwm[2]=TIM1->CCR3;
    // With no missed update, preloaded values read at this boundary have just latched.
    recorder.begin(cycles,count,SystemCoreClock/16000U);
}
extern "C" void ruka3_foc_trace_irq_end() {
    using namespace ruka3::foc_trace;
    const bool active=recorder.state()==State::capturing;
    auto* record=recorder.current();
    if(record && (TIM1->BDTR&TIM_BDTR_MOE)) record->words[10]|=pwm_moe_end;
    recorder.finish(DWT->CYCCNT,(TIM1->SR&TIM_SR_UIF)!=0);
    if(active) {
        // Include ring bookkeeping; only these final stores/return are outside the measured body.
        const auto elapsed=DWT->CYCCNT-irq_start;
        if(elapsed>recorder.max_cycles) recorder.max_cycles=elapsed;
        if(record) {record->words[3]=elapsed;record->words[4]=recorder.max_cycles;}
    }
}
#endif
