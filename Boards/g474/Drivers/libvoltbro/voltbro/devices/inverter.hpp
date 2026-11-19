#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_conf.h"
#ifdef HAL_ADC_MODULE_ENABLED
#include "main.h"
#include <cstdint>
#include <array>
#include "phase_current_sense.hpp"
#ifdef RUKA3_FOC_TRACE
#include "foc_trace_hw.hpp"
#endif


#ifdef MONITOR
static volatile float I_A_glob, I_B_glob, I_C_glob, busV_glob;
#endif

class BaseInverter {
protected:
    float I_A_offset = 0;
    float I_B_offset = 0;
    float I_C_offset = 0;
    float I_A = 0;
    float I_B = 0;
    float I_C = 0;
    float busV = 0;

    bool is_started = false;
public:
    float get_A() const { return I_A; }
    float get_B() const { return I_B; }
    float get_C() const { return I_C; }
    float get_busV() const { return busV; }
    bool has_started() const { return is_started; }

    virtual void start() = 0;
    virtual void update() = 0;
    virtual ~BaseInverter() = default;
};


class Inverter: public BaseInverter {
private:
    volatile uint32_t ADC_buffer[4] = {};
    ADC_HandleTypeDef* hadc;
    const ruka3::PhaseCurrentSense& sense;

public:
    Inverter(ADC_HandleTypeDef* hadc, const ruka3::PhaseCurrentSense& sense): hadc(hadc), sense(sense) {}

    std::array<std::uint32_t, 4> raw_adc() const {
        return {ADC_buffer[0], ADC_buffer[1], ADC_buffer[2], ADC_buffer[3]};
    }
    std::array<float, 3> offset_volts() const { return {I_A_offset, I_B_offset, I_C_offset}; }

    void start() override {
        if (has_started()) {
            return;
        }

        // apply factory calibration
        HAL_IMPORTANT(HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED))
        // const_cast to allow HAL to work
        HAL_IMPORTANT(HAL_ADC_Start_DMA(hadc, const_cast<uint32_t *>(ADC_buffer), 4))
        // The control ISR reads this circular buffer directly; empty HT/TC
        // callbacks would otherwise interrupt it twice per four-rank scan.
        __HAL_DMA_DISABLE_IT(hadc->DMA_Handle, DMA_IT_HT | DMA_IT_TC);
        // Record offset;
        for (unsigned i = 0; i < sense.offset_samples; i++) {
            HAL_Delay(1);
            I_A_offset += sense.adc_voltage(ADC_buffer[1]);
            I_B_offset += sense.adc_voltage(ADC_buffer[2]);
            I_C_offset += sense.adc_voltage(ADC_buffer[3]);
        }
        I_A_offset /= sense.offset_samples;
        I_B_offset /= sense.offset_samples;
        I_C_offset /= sense.offset_samples;

        is_started = true;
    }

    void update() override {
#ifdef RUKA3_FOC_TRACE
        namespace trace=ruka3::foc_trace;
        trace::sample_begin();
        // Preserve original volatile-read order A/B/C/bus; do not latch/fix DMA.
        I_A = sense.phase_current(trace::adc_word(1,ADC_buffer[1]), I_A_offset, 0);
        I_B = sense.phase_current(trace::adc_word(2,ADC_buffer[2]), I_B_offset, 1);
        I_C = sense.phase_current(trace::adc_word(3,ADC_buffer[3]), I_C_offset, 2);
        busV = sense.bus_voltage_gain * sense.adc_voltage(trace::adc_word(0,ADC_buffer[0]));
        trace::sample_end();
        if(auto* r=trace::frame(trace::Profile::measurement)) {
            r->f(16,I_A);r->f(17,I_B);r->f(18,I_C);r->f(19,I_A+I_B+I_C);
            r->f(24,I_A_offset);r->f(25,I_B_offset);r->f(26,I_C_offset);r->f(27,busV);
        }
#else
        I_A = sense.phase_current(ADC_buffer[1], I_A_offset, 0);
        I_B = sense.phase_current(ADC_buffer[2], I_B_offset, 1);
        I_C = sense.phase_current(ADC_buffer[3], I_C_offset, 2);
        busV = sense.bus_voltage_gain * sense.adc_voltage(ADC_buffer[0]);
#endif
        #ifdef MONITOR
            I_A_glob = I_A;
            I_B_glob = I_B;
            I_C_glob = I_C;
            busV_glob = busV;
        #endif
    }
};

#endif
#endif
