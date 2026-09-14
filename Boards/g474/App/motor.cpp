#include "app.h"

#include "tim.h"
#include "spi.h"
#include "adc.h"

#include "voltbro/devices/gpio.hpp"
#include "voltbro/devices/inverter.hpp"
#include "voltbro/encoders/ASxxxx/AS5048A.hpp"

static constexpr float T = 0.00005f;
static constexpr float kTorqueConst = 0.011f;
static constexpr float kMaxCurrent = 22.0f;
static constexpr float kMaxTorque = 10.0f;
static constexpr float kStallCurrent = 22.0f;
static constexpr float kStallTimeout = 3.0f;
static constexpr float kStallTolerance = 0.2f;
static constexpr float kCalibrationVoltage = 1.0f;
static constexpr float kFilterExpectedA = 0.0f;
static constexpr float kFilterG1 = 0.015700989410003974f;
static constexpr float kFilterG2 = 3.925227776360174f;
static constexpr float kFilterG3 = 387.54711795263574f;
static constexpr float kCurrentLpf = 0.0925f;
static constexpr float kCurrentPidKp = 2.0f;
static constexpr float kCurrentPidKi = 600.0f;
static constexpr float kCurrentPidKd = 0.0f;
static constexpr float kMaxVoltage = 50.0f;

static GenericEncoder& motor_encoder_instance() {
    static AS5048A encoder(
        GpioPin(SPI3_CS0_GPIO_Port, SPI3_CS0_Pin),
        &hspi3,
        motor_encoder_sign < 0,
        electric_offset
    );
    return encoder;
}

GenericEncoder& get_gear_encoder() {
    static AS5048A encoder(
        GpioPin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin),
        &hspi1,
        gear_encoder_sign < 0,
        gear_encoder_zero
    );
    return encoder;
}

FOC& get_motor() {
    static Inverter inverter(&hadc1);
    static FOC motor_controller(
        T,
        FiltersConfig{
            .expected_a = kFilterExpectedA,
            .g1 = kFilterG1,
            .g2 = kFilterG2,
            .g3 = kFilterG3,
            .I_lpf_coefficient = kCurrentLpf,
        },
        PIDConfig{
            .kp = kCurrentPidKp,
            .ki = kCurrentPidKi,
            .kd = kCurrentPidKd,
            .max_output = kMaxVoltage,
            .min_output = -kMaxVoltage,
        },
        PIDConfig{
            .kp = kCurrentPidKp,
            .ki = kCurrentPidKi,
            .kd = kCurrentPidKd,
            .max_output = kMaxVoltage,
            .min_output = -kMaxVoltage,
        },
        DriveLimits{
            .current_limit = kMaxCurrent,
            .user_current_limit = kMaxCurrent,
            .user_torque_limit = kMaxTorque,
            .user_position_lower_limit = (upper_limit == 0.0f && lower_limit == 0.0f) ? -FLOAT_INF : lower_limit,
            .user_position_upper_limit = (upper_limit == 0.0f && lower_limit == 0.0f) ? FLOAT_INF : upper_limit,
        },
        DriveInfo{
            .torque_const = kTorqueConst,
            .max_current = kMaxCurrent,
            .max_torque = kMaxTorque,
            .stall_current = kStallCurrent,
            .stall_timeout = kStallTimeout,
            .stall_tolerance = kStallTolerance,
            .calibration_voltage = kCalibrationVoltage,
            .l_pins = std::array<GpioPin, 3>{
                GpioPin(INLA_GPIO_Port, INLA_Pin),
                GpioPin(INLB_GPIO_Port, INLB_Pin),
                GpioPin(INLC_GPIO_Port, INLC_Pin),
            },
            .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
            .common = CommonDriverConfig{
                .ppairs = 14,
                .gear_ratio = gear_ratio,
                .user_angle_offset = 0.0f,
            },
        },
        &htim1,
        motor_encoder_instance(),
        inverter,
        FusionConfig{
            .gear_encoder = &get_gear_encoder(),
            .gear_encoder_zero = gear_encoder_zero,
            .motor_angle_sign = static_cast<int8_t>(motor_encoder_sign),
            .gear_angle_sign = static_cast<int8_t>(gear_encoder_sign),
            .blend = 0.5f,
            .velocity_blend = 0.15f,
            .velocity_lpf_coefficient = 0.05f,
        }
    );
    return motor_controller;
}
