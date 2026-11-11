#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "foc.hpp"

#include "arm_math.h"
#include "main.h"
#include <numeric>

#include "voltbro/math/transform.hpp"


#ifdef DEBUG
static volatile float current_target_angle = 0;
static volatile encoder_data current_encoder_value = 0;
#endif

void FOC::set_windings_calibration(float target_angle) {
    #ifdef DEBUG
    current_target_angle = target_angle;
    #endif
    float s = arm_sin_f32(target_angle);
    float c = arm_cos_f32(target_angle);

    float DVA, DVB, DVC;

    //inverse dq0 transform on voltages
    abc(s, c, -drive_info.calibration_voltage, 0, &DVA, &DVB, &DVC);

    DQs[0] = 1000 + (int16_t)(1000.0f*DVA);
    DQs[1] = 1000 + (int16_t)(1000.0f*DVB);
    DQs[2] = 1000 + (int16_t)(1000.0f*DVC);

    set_pwm();
}

template <class Container, size_t N>
class CircularArray {
private:
    Container* container_ptr;

    size_t normalize_index(int idx) const {
        int normalized = idx % static_cast<int>(N);
        if (normalized < 0) {
            normalized += N;
        }
        return static_cast<size_t>(normalized);
    }
public:
    CircularArray(Container* container_ptr): container_ptr(container_ptr) {}

    typename Container::value_type& operator[](int idx) {
        return (*container_ptr)[normalize_index(idx)];
    }

    const typename Container::value_type& operator[](int idx) const {
        return (*container_ptr)[normalize_index(idx)];
    }

    const Container* get_container() const {
        return container_ptr;
     }
};

// NOTE: caller has to GUARANTEE that calculations_buffer is AT LEAST (CALIBRATION_BUFF_SIZE + 1) * sizeof(int) bytes long
bool FOC::calibrate(CalibrationData& calibration_data, std::byte* additional_buffer, size_t buffer_size) {
    // Calibration measures the encoder itself, not a previously corrected angle.
    lookup_table = nullptr;
    volatile struct CalibrationStats {
        int soft_gaps_n = 0;
        int hard_gaps_n = 0;
        float average_fwd_bwd_discrepancy = 0;
        float max_fwd_bwd_discrepancy = 0;
        float average_electric_discrepancy = 0;
    } processing_stats{};

    auto zero_pwm = [this]() {
        DQs[0] = 0;
        DQs[1] = 0;
        DQs[2] = 0;
        set_pwm();
     };
    auto set_electric_angle = [this](float angle, uint32_t delay=20) {
        set_windings_calibration(angle);
        HAL_Delay(delay);
        for (int i = 0; i < 5; i++) {
            update_angle();
        }
    };

//#pragma region Electrical Offset
    const int ppairs = drive_info.common.ppairs;
    const int samples_count = ppairs * 2;
#ifdef MONITOR
    int offset_samples[28] = {0};
#else
    int offset_samples[samples_count] = {0};
#endif

    auto get_circular_error = [](int meas, int expected, int cycle_len) {
        int diff = meas - expected;
        while (diff > (cycle_len / 2)) {
            diff -= cycle_len;
        }
        while (diff < (-cycle_len / 2)) {
            diff += cycle_len;
        }
        return diff;
    };
    // Align only to the nearest electrical equilibrium. Do not seek the
    // absolute encoder zero: all following passes start/end at field phase 0.
    auto align_field_zero = [&set_electric_angle]() {
        set_electric_angle(0.0f, 500);
    };

    const int ppair_step_counts = 50;
    const int encoder_steps_per_pair = encoder.CPR / ppairs;
    const float ppair_step = pi2 / ppair_step_counts;
    const int overshoot_steps = 4;

    auto offset_calibration_pass = [&](int* offset_array, int offset) {
        // Undershoot at the start to avoid hysteresis then roll forward a bit
        for (int j = 1; j < overshoot_steps; j++) {
            set_electric_angle(0 - ppair_step * j);
        }
        for (int j = overshoot_steps; j >= 1; j--) {
            set_electric_angle(0 - ppair_step * j);
        }

        for (int i = 0; i < ppairs; i++) {
            const float local_zero_point = pi2 * i;
            set_electric_angle(local_zero_point, 100);

            int expected_value = encoder_steps_per_pair * i;
            int current_value = int(encoder.get_value()) - offset;
            int current_offset = get_circular_error(current_value, expected_value, encoder.CPR);
            offset_array[i] = current_offset;

            for (int j = 0; j < ppair_step_counts; j++) {
                set_electric_angle(local_zero_point + ppair_step * (j + 1));
            }
        }

        // Overshoot at the end to avoid hysteresis then roll back a bit
        for (int j = 1; j < overshoot_steps; j++) {
            set_electric_angle(pi2 * ppairs + ppair_step * j);
        }
        for (int j = overshoot_steps; j >= 1; j--) {
            set_electric_angle(pi2 * ppairs + ppair_step * j);
        }

        for (int i = ppairs - 1; i >= 0; i--) {
            const float local_zero_point = pi2 * (i + 1);
            set_electric_angle(local_zero_point, 100);

            int expected_value = encoder_steps_per_pair * (i + 1);
            int current_value = int(encoder.get_value()) - offset;
            int current_offset = get_circular_error(current_value, expected_value, encoder.CPR);
            offset_array[samples_count - i - 1] = current_offset;

            for (int j = 0; j < ppair_step_counts; j++) {
                set_electric_angle(local_zero_point - ppair_step * (j + 1));
            }
        }

        return motor_calibration::electrical_offset_mean(
            offset_array, samples_count, encoder_steps_per_pair);
    };

    align_field_zero();
    int average_offset = offset_calibration_pass(offset_samples, 0);
    calibration_data.meas_elec_offset = average_offset;

#ifdef MONITOR
    int verification_samples[28] = {0};
#else
    int verification_samples[samples_count] = {0};
#endif
    align_field_zero();
    processing_stats.average_electric_discrepancy = offset_calibration_pass(verification_samples, average_offset);
    for (auto sample : verification_samples) {
        if (abs(motor_calibration::circular_error(sample, 0, encoder_steps_per_pair)) > (encoder_steps_per_pair / 10)) {
            zero_pwm();
            return false; // Do not persist a failed electrical alignment.
        }
    }
//#pragma endregion

    align_field_zero();
    // Pick the equivalent electrical offset nearest this sweep's rotor turn,
    // so the correction curve does not inherit an arbitrary whole-pole offset.
    const float pole_pitch = static_cast<float>(encoder.CPR) / ppairs;
    average_offset += static_cast<int>(std::lround(
        (static_cast<int>(encoder.get_value()) - average_offset) / pole_pitch) * pole_pitch);
    calibration_data.meas_elec_offset = average_offset;

//#pragma region Curve Sampling
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
    // TODO: verify that this is safe, but it works so probably fine?
    __non_const_calib_array_t* fwd_calibration_array = &calibration_data.calibration_array;
    #pragma GCC diagnostic pop

    // No big allocation is done here, just boundary and alignment checks
    auto allocate_object = [&zero_pwm]<typename T>(void* buffer_, size_t buffer_size) {
        size_t available_size = buffer_size;
        void* aligned_buffer = std::align(alignof(T), sizeof(T), buffer_, available_size);
        if (available_size < (sizeof(T))) {
            zero_pwm();
            Error_Handler(); // Not enough space
        }
        T* ptr = new (aligned_buffer) T{};
        std::byte* new_buffer = static_cast<std::byte*>(aligned_buffer) + sizeof(T);
        size_t new_buffer_size = available_size - sizeof(T);
        return std::make_tuple(ptr, new_buffer, new_buffer_size);
    };

    auto [bwd_calibration_array, free_buffer, free_buffer_size] = \
        allocate_object.template operator()<__non_const_calib_array_t>(additional_buffer, buffer_size);

    const int SENTINEL_EMPTY = 0x7FFFFFFF;
    for (auto& array: {fwd_calibration_array, bwd_calibration_array}) {
        array->fill(SENTINEL_EMPTY);
    }

    const float electrical_span = (float)ppairs * pi2;
    const float electric_angle_delta = electrical_span / (float)calibration_data.calibration_array.size();
    float encoder_steps_in_electrical_radian = (float)encoder.CPR / (ppairs * pi2);

    auto sample_point = [&](std::array<int, CALIBRATION_BUFF_SIZE>* arr, float angle_target) {
        set_electric_angle(angle_target, 5);

        int expected_value = (int)roundf(angle_target * encoder_steps_in_electrical_radian);
        int offset_encoder_value = static_cast<int>(encoder.get_value()) - average_offset;
        while (offset_encoder_value < 0) {
            offset_encoder_value += encoder.CPR;
        }
        while (offset_encoder_value >= encoder.CPR) {
            offset_encoder_value -= encoder.CPR;
        }

        size_t idx = motor_calibration::lut_index(encoder.get_value());
        if (idx >= arr->size()) {
            Error_Handler(); // Should never happen if buffer size is correct
        }

        (*arr)[idx] = get_circular_error(offset_encoder_value, expected_value, encoder.CPR);
    };

    // Undershoot at the start to avoid hysteresis then roll forward a bit
    for (int j = 1; j < overshoot_steps; j++) {
        set_electric_angle(0 - ppair_step * j);
    }
    for (int j = overshoot_steps; j >= 1; j--) {
        set_electric_angle(0 - ppair_step * j);
    }
    float electric_angle = 0;
    while (electric_angle < electrical_span) {
        sample_point(fwd_calibration_array, electric_angle);
        electric_angle += electric_angle_delta;
    }

    // Overshoot at the end to avoid hysteresis then roll back a bit
    for (int j = 1; j < overshoot_steps; j++) {
        set_electric_angle(electrical_span + ppair_step * j);
    }
    for (int j = overshoot_steps; j >= 1; j--) {
        set_electric_angle(electrical_span + ppair_step * j);
    }
    electric_angle = electrical_span;
    while (electric_angle > 0) {
        sample_point(bwd_calibration_array, electric_angle);
        electric_angle -= electric_angle_delta;
    }

    zero_pwm();

    // Start post-processing
    CircularArray<__non_const_calib_array_t, CALIBRATION_BUFF_SIZE> final_array(fwd_calibration_array);

    // 1. Merge arrays
    // NOTE: fwd_calibration_array and final_array is the same underlying memory, so be careful!
    //       BUT final_array is CircularArray wrapper, so for calculations we will use it
    int total_discrepancy = 0;
    int discrepancy_counter = 0;
    for (size_t i = 0; i < fwd_calibration_array->size(); i++) {
        int fwd = (*fwd_calibration_array)[i];
        int bwd = (*bwd_calibration_array)[i];

        if (fwd != SENTINEL_EMPTY && bwd != SENTINEL_EMPTY) {
            final_array[i] = (fwd + bwd) / 2;
            discrepancy_counter += 1;
            int current_discrepancy = abs(fwd - bwd);
            total_discrepancy += current_discrepancy;
            if (current_discrepancy > processing_stats.max_fwd_bwd_discrepancy) {
                processing_stats.max_fwd_bwd_discrepancy = current_discrepancy;
            }
        } else if (fwd != SENTINEL_EMPTY) {
            final_array[i] = fwd;
            // Here and further: <'++'/'+='/... expression of 'volatile'-qualified type is deprecated> - C++20
            processing_stats.soft_gaps_n = processing_stats.soft_gaps_n + 1;
        } else if (bwd != SENTINEL_EMPTY) {
            final_array[i] = bwd;
            processing_stats.soft_gaps_n = processing_stats.soft_gaps_n + 1;
        } else {
            final_array[i] = SENTINEL_EMPTY; // Still a gap, mark for later
            processing_stats.hard_gaps_n = processing_stats.hard_gaps_n + 1;
        }
    }
    processing_stats.average_fwd_bwd_discrepancy = \
        discrepancy_counter ? (float)total_discrepancy / discrepancy_counter : 0.0f;

    // 2. Combined gap filling and smoothing
    // Copy raw merged array to temp source buffer
    (*bwd_calibration_array) = *final_array.get_container();
    const CircularArray<calibration_array_t, CALIBRATION_BUFF_SIZE> src(bwd_calibration_array);

    // Offline, zero-phase smoothing of the measured periodic correction.
    // Wider than an individual 8-tick bucket to reject sampling/settling noise;
    // the independent source buffer keeps every output free of feedback.
    constexpr int RADIUS = 16;
    for (std::size_t i = 0; i < CALIBRATION_BUFF_SIZE; ++i) {
        float weighted_sum = 0.0F;
        float weight_sum = 0.0F;
        for (int delta = -RADIUS; delta <= RADIUS; ++delta) {
            const int sample = src[static_cast<int>(i) + delta];
            if (sample == SENTINEL_EMPTY) continue;
            const float weight = RADIUS + 1 - std::abs(delta);
            weighted_sum += weight * sample;
            weight_sum += weight;
        }
        if (weight_sum == 0.0F) return false;
        final_array[i] = static_cast<int>(std::lround(weighted_sum / weight_sum));
    }

//#pragma endregion

    memset(additional_buffer, 0, buffer_size);
    // Use the already aligned table pointer: binding a packed array field to
    // a const reference would create an 8-KB temporary on the MCU stack.
    return motor_calibration::monotonic_lut(*fwd_calibration_array, encoder.CPR);
}

#endif
#endif
