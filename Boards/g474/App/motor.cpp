#include "app.h"
#include "robot_config.h"

#include "tim.h"
#include "spi.h"
#include "adc.h"

#include "voltbro/devices/gpio.hpp"
#include "voltbro/devices/inverter.hpp"
#include "voltbro/encoders/ASxxxx/AS5048A.hpp"

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
    constexpr auto& profile = active_joint_profile;
    constexpr auto& motor_profile = profile.motor;
    static Inverter inverter(&hadc1, ruka3::kG474CurrentSense);
    static FOC motor_controller(
        motor_profile.control_period_s,
        FiltersConfig{
            .expected_a = motor_profile.filter_expected_a,
            .g1 = motor_profile.filter_g1,
            .g2 = motor_profile.filter_g2,
            .g3 = motor_profile.filter_g3,
            .I_lpf_coefficient = motor_profile.current_lpf,
        },
        PIDConfig{
            .kp = motor_profile.current_pid_kp,
            .ki = motor_profile.current_pid_ki,
            .kd = motor_profile.current_pid_kd,
            .max_output = ruka3::kG474CommissioningVoltageLimitV,
            .min_output = -ruka3::kG474CommissioningVoltageLimitV,
        },
        PIDConfig{
            .kp = motor_profile.current_pid_kp,
            .ki = motor_profile.current_pid_ki,
            .kd = motor_profile.current_pid_kd,
            .max_output = ruka3::kG474CommissioningVoltageLimitV,
            .min_output = -ruka3::kG474CommissioningVoltageLimitV,
        },
        DriveLimits{
            .current_limit = ruka3::kG474CommissioningCurrentLimitA,
            .user_current_limit = ruka3::kG474CommissioningCurrentLimitA,
            .user_torque_limit = motor_profile.torque_limit,
            .user_position_lower_limit = -FLOAT_INF,
            .user_position_upper_limit = FLOAT_INF,
        },
        DriveInfo{
            .torque_const = motor_profile.torque_const,
            .max_current = motor_profile.current_limit,
            .max_torque = motor_profile.torque_limit,
            .stall_current = motor_profile.stall_current,
            .stall_timeout = motor_profile.stall_timeout_s,
            .stall_tolerance = motor_profile.stall_tolerance,
            .calibration_voltage = motor_profile.calibration_voltage,
            .current_direction = profile.motor_command_sign,
            .modulation_voltage_ratio = ruka3::kG474ModulationVoltageRatio,
            .max_pwm_ticks = RUKA3_G474_MAX_PWM_TICKS,
            .phase_current_trip_a = ruka3::kG474PhaseOvercurrentTripA,
            .l_pins = std::array<GpioPin, 3>{
                GpioPin(INLA_GPIO_Port, INLA_Pin),
                GpioPin(INLB_GPIO_Port, INLB_Pin),
                GpioPin(INLC_GPIO_Port, INLC_Pin),
            },
            .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
            .common = CommonDriverConfig{
                .ppairs = motor_profile.pole_pairs,
                .gear_ratio = static_cast<uint8_t>(profile.gear_ratio),
                .user_angle_offset = profile.position_offset,
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
            .output_calibration = &get_output_calibration_core(),
        }
    );
    return motor_controller;
}
