#pragma once
#include "encoder_calibration.hpp"
#include <cstdint>
#include "voltbro/encoders/generic.h"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <utility>

#include "../bldc.h"
#include "fusion_math.hpp"
#include "voltbro/math/regulators/pid.hpp"

#define USE_CALIBRATION_ARRAY
constexpr size_t CALIBRATION_BUFF_SIZE = 2048;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;

struct __attribute__((packed)) CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x99ABCDEF;
    bool was_calibrated;
    bool is_encoder_inverted;
    uint16_t ppair_counter;
    int meas_elec_offset;
    uint32_t type_id = 0;
#ifdef USE_CALIBRATION_ARRAY
    __non_const_calib_array_t calibration_array;
#endif

    CalibrationData() {
        reset();
        type_id = 0;  // to distinguish between uninitialized and reset data
    }

    void reset() {
        type_id = TYPE_ID;
        was_calibrated = false;
        is_encoder_inverted = false;
        ppair_counter = 0;
        meas_elec_offset = 0;
#ifdef USE_CALIBRATION_ARRAY
        calibration_array.fill(0);
#endif
    }
};

struct FOCTarget {
    float torque = 0.0f;
    float angle = 0.0f;
    float velocity = 0.0f;
    // Motor-side torque per output-coordinate error; public gains / gear_ratio.
    float angle_kp = 0.0f;
    float velocity_kp = 0.0f;
};

struct CurrentLoopSnapshot {
    float id;
    float iq;
    float iq_target;
    float vd;
    float vq;
};
enum class CurrentLoopFault : std::uint8_t { none = 0, phase_overcurrent = 1, invalid_bus_voltage = 2 };

struct FiltersConfig {
    float expected_a;
    float g1;
    float g2;
    float g3;
    float I_lpf_coefficient;
};

struct FusionConfig {
    GenericEncoder* gear_encoder = nullptr;
    encoder_data gear_encoder_zero = 0;
    int8_t motor_angle_sign = 1;
    int8_t gear_angle_sign = 1;
    float blend = 0.5f;
    float velocity_blend = 0.2f;
    float velocity_lpf_coefficient = 0.1f;
    const EncoderCalibration* output_calibration = nullptr;
};

struct WrappedKalmanState {
    bool initialized = false;
    float prev_angle = 0.0f;
    float theta_hat = 0.0f;
    float omega_hat = 0.0f;
    float epsilon_hat = 0.0f;
};

/**
 * Field oriented control.
 */
class FOC: public BLDCController  {
protected:
    float T;
    float raw_elec_angle = 0;
    float gear_measurement_angle = 0;
    float elec_angle = 0;
    float I_Q = 0;
    float I_D = 0;
    float V_d = 0;
    float V_q = 0;
    float i_q_set = 0;
    volatile CurrentLoopFault current_fault = CurrentLoopFault::none;
    calibration_array_t* lookup_table = nullptr;
    GenericEncoder& encoder;
    const FiltersConfig filters_config;
    const FusionConfig fusion_config;
    FOCTarget foc_target;
    const float voltage_limit;
    PIDRegulator q_reg;
    PIDRegulator d_reg;
    PIDRegulator control_reg = PIDRegulator();
    WrappedKalmanState motor_kalman_state;
    WrappedKalmanState gear_kalman_state;
    bool fusion_offset_initialized = false;
    bool fusion_velocity_initialized = false;
    float motor_fusion_offset = 0.0f;
    arm_atomic(float) gear_angle = 0.0f;
    arm_atomic(float) gear_velocity = 0.0f;
    arm_atomic(float) fusion_angle = 0.0f;
    arm_atomic(float) fusion_velocity = 0.0f;

    void apply_kalman();
    bool run_wrapped_kalman(float raw_angle, WrappedKalmanState& state, float& predicted_angle, float& predicted_velocity);
    void update_angle();
    void update_gear_angle();
    void update_fusion_angle();
    virtual void update_shaft_angle();

    void set_windings_calibration(float current_angle);
public:
    virtual void calibrate(CalibrationData& calibration_data, std::byte* additional_buffer, size_t buffer_size);
    void apply_calibration(CalibrationData& calibration_data) {
        // Replace config parameters with the ones loaded from EEPROM
        // Very ugly hack, but it will work for now
        encoder.electric_offset = calibration_data.meas_elec_offset;
#ifdef USE_CALIBRATION_ARRAY
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
        // TODO: verify that this is safe, but it works so probably fine?
        lookup_table = &(calibration_data.calibration_array);
        #pragma GCC diagnostic pop
#endif
        encoder.is_inverted = calibration_data.is_encoder_inverted;
    }

    FOC(
        float T,
        FiltersConfig&& filters_config,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        GenericEncoder& encoder,
        BaseInverter& inverter,
        FusionConfig fusion_config = {}
    ):
        BLDCController(
            drive_limits,
            drive_info,
            htim,
            inverter
        ),
        T(T),
        encoder(encoder),
        filters_config(std::move(filters_config)),
        fusion_config(fusion_config),
        voltage_limit(q_config.max_output),
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config))
        {}

    // Corrected rotor angle (offset/LUT applied), before the native observer.
    float get_preobserver_rotor_angle() const { return raw_elec_angle; }

    float get_electric_angle() {
        return elec_angle;
    }
    float get_working_current() {
        return drive_info.current_direction * I_Q;
    }
    CurrentLoopSnapshot current_snapshot() const { return {I_D, I_Q, i_q_set, V_d, V_q}; }
    float get_current_limit() const { return drive_limits.current_limit; }
    SetPointType get_point_type() const { return point_type; }
    CurrentLoopFault get_current_fault() const { return current_fault; }
    bool clear_current_fault() {
        if (is_on() || !(inverter.get_busV() > 0.0F) ||
            std::fabs(inverter.get_A()) > drive_info.phase_current_trip_a ||
            std::fabs(inverter.get_B()) > drive_info.phase_current_trip_a ||
            std::fabs(inverter.get_C()) > drive_info.phase_current_trip_a) return false;
        current_fault = CurrentLoopFault::none;
        return true;
    }
    float get_current_target() const {
        return point_type == SetPointType::CURRENT ? target : 0.0F;
    }
    float get_position_target() const {
        return point_type == SetPointType::UNIVERSAL ? foc_target.angle : get_fusion_angle();
    }
    void reset_control() {
        q_reg.reset();
        d_reg.reset();
        control_reg.reset();
        i_q_set = V_d = V_q = 0.0F;
    }
    HAL_StatusTypeDef start() override {
        if (current_fault != CurrentLoopFault::none) return HAL_ERROR;
        reset_control();
        return BLDCController::start();
    }
    HAL_StatusTypeDef stop() override {
        const auto result = BLDCController::stop();
        reset_control();
        return result;
    }
    void set_pwm() override {
        for (auto& value : DQs) value = std::min(value, drive_info.max_pwm_ticks);
        BLDCController::set_pwm();
    }
    // Current target is native joint-oriented amperes; only the q-axis sign
    // is applied inside the loop. position_direction belongs to the caller.
    void set_current_point(float amps) {
        CRITICAL_SECTION({
            if (point_type != SetPointType::CURRENT) reset_control();
            target = amps;
            point_type = SetPointType::CURRENT;
        })
    }
    float get_gear_angle() const {
        return gear_angle;
    }
    float get_gear_measurement_angle() const { return gear_measurement_angle; }
    bool has_output_calibration() const { return fusion_config.output_calibration != nullptr && fusion_config.output_calibration->has_stored_data(); }
    void reset_output_tracking() {
        CRITICAL_SECTION({
            gear_kalman_state = {};
            fusion_offset_initialized = false;
            fusion_velocity_initialized = false;
        })
    }
    float get_gear_velocity() const {
        return gear_velocity;
    }
    float get_fusion_angle() const {
        return fusion_angle;
    }
    float get_fusion_velocity() const {
        return fusion_velocity;
    }
    float get_aligned_motor_angle() const {
        return voltbro::foc::wrap_pi(BLDCController::get_angle() + motor_fusion_offset);
    }
    int get_encoder_electric_offset() const {
        return encoder.electric_offset;
    }
    void set_encoder_electric_offset(int new_offset) {
        encoder.electric_offset = new_offset;
        motor_kalman_state = WrappedKalmanState{};
        fusion_offset_initialized = false;
        fusion_velocity_initialized = false;
        motor_fusion_offset = 0.0f;
    }
    void hold_open_loop_calibration_angle(float target_angle) {
        set_windings_calibration(target_angle);
    }
    void disable_open_loop_drive() {
        DQs[0] = 0;
        DQs[1] = 0;
        DQs[2] = 0;
        set_pwm();
    }

    void set_foc_point(FOCTarget&& target) {
        CRITICAL_SECTION({
            if (point_type != SetPointType::UNIVERSAL) reset_control();
            foc_target = std::move(target);
            point_type = SetPointType::UNIVERSAL;
        })
    }
    void update_q_config(PIDConfig&& new_config) {
        q_reg.update_config(std::move(new_config));
    }
    void update_d_config(PIDConfig&& new_config) {
        d_reg.update_config(std::move(new_config));
    }
    void update_control_config(PIDConfig&& new_config) {
        control_reg.update_config(std::move(new_config));
    }
    const GenericEncoder& get_encoder() const {
        return encoder;
    }

    HAL_StatusTypeDef init() override {
        fusion_offset_initialized = false;
        fusion_velocity_initialized = false;
        motor_fusion_offset = 0.0f;

        HAL_StatusTypeDef result = BLDCController::init();
        if (result != HAL_OK) {
            return result;
        }

        result = encoder.init();
        if (result != HAL_OK) {
            return result;
        }

        if (fusion_config.gear_encoder != nullptr) {
            return fusion_config.gear_encoder->init();
        }

        return HAL_OK;
    }
    void update() override;
    virtual void update_sensors();
};

#endif
#endif
