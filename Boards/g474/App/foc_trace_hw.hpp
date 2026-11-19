#pragma once
#ifdef RUKA3_FOC_TRACE
#include "foc_trace.hpp"
#include "foc_trace_hooks.h"
#include "stm32g4xx_hal.h"
#ifndef RUKA3_FOC_TRACE_FRAMES
#define RUKA3_FOC_TRACE_FRAMES 256
#endif
#ifndef RUKA3_FOC_TRACE_DECIMATION
#define RUKA3_FOC_TRACE_DECIMATION 1
#endif
namespace ruka3::foc_trace {
extern Recorder<RUKA3_FOC_TRACE_FRAMES> recorder;
extern std::uint32_t selected_page;
extern DVoltagePulse d_pulse;
extern std::uint32_t previous_pwm[3];
inline Frame* frame(Profile p) { return recorder.profile()==p?recorder.current():nullptr; }
inline std::uint32_t pack16(std::uint32_t a,std::uint32_t b) { return (a&0xffffU)|((b&0xffffU)<<16U); }
// Reads flags only. Do not acknowledge DMA/ADC flags or change conversion timing.
inline void sample_begin() {
    if(auto* r=recorder.current()) {
        auto& w=r->words;w[6]=DMA1_Channel1->CNDTR;w[8]=ADC1->ISR;w[9]=DMA1->ISR;
        w[11]=DWT->CYCCNT-w[1];
        if(w[8]&ADC_ISR_OVR) w[10]|=adc_overrun;
        if(w[9]&DMA_ISR_TEIF1) w[10]|=dma_error;
        if(recorder.profile()==Profile::measurement) {
            w[28]=pack16(previous_pwm[0],previous_pwm[1]);
            w[29]=pack16(previous_pwm[2],TIM1->CNT);
        }
    }
}
inline std::uint32_t adc_word(unsigned rank,std::uint32_t value) {
    if(auto* r=frame(Profile::measurement)) r->words[12+rank]=value;
    return value;
}
inline void sample_end() {
    if(auto* r=recorder.current()) {
        r->words[7]=DMA1_Channel1->CNDTR;
        if(r->words[6]!=r->words[7]) r->words[10]|=dma_moved;
    }
}
}
#endif
