#pragma once
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
    float angle_kp = 0.0f;
    float velocity_kp = 0.0f;
};

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
    float elec_angle = 0;
    float I_Q = 0;
    calibration_array_t* lookup_table = nullptr;
    GenericEncoder& encoder;
    const FiltersConfig filters_config;
    const FusionConfig fusion_config;
    FOCTarget foc_target;
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
        const_cast<int&>(encoder.electric_offset) = calibration_data.meas_elec_offset;
#ifdef USE_CALIBRATION_ARRAY
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
        // TODO: verify that this is safe, but it works so probably fine?
        lookup_table = &(calibration_data.calibration_array);
        #pragma GCC diagnostic pop
#endif
        const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;
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
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config))
        {}

    float get_electric_angle() {
        return elec_angle;
    }
    float get_working_current() {
        return I_Q;
    }
    float get_gear_angle() const {
        return gear_angle;
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
        const_cast<int&>(encoder.electric_offset) = new_offset;
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
        point_type = SetPointType::UNIVERSAL;
        foc_target = std::move(target);
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
