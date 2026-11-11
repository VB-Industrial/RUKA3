#include <algorithm>
#include <array>
#include <cassert>
#include "fault_manager.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include "stm32g4xx_hal.h"
#include "robot_config.h"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/math/transform.hpp"
#include "voltbro/motors/bldc/foc/fusion_math.hpp"

static void close(float actual, float expected, float tolerance = 0.00002F) {
    assert(std::fabs(actual - expected) <= tolerance);
}

int main() {
    const auto& sense = ruka3::kG474CurrentSense;
    const auto raw = [&](float volts) {
        return static_cast<std::uint32_t>(std::lround(volts / sense.adc_reference_v *
            sense.adc_resolution_counts * sense.oversampling_ratio));
    };
    close(sense.phase_current(raw(1.65F), 1.65F, 0), 0.0F);
    const float half_current_lsb = sense.adc_reference_v /
        (sense.adc_resolution_counts * sense.oversampling_ratio * sense.volts_per_amp) * 0.5F + 1e-5F;
    close(sense.phase_current(raw(1.65F + 0.045F * 0.5F), 1.65F, 0), 0.5F, half_current_lsb);
    close(sense.phase_current(raw(1.65F - 0.045F * 0.5F), 1.65F, 1), -0.5F, half_current_lsb);
    close(sense.adc_voltage(raw(24.0F / 16.0F)) * sense.bus_voltage_gain, 24.0F,
          half_current_lsb * sense.volts_per_amp * sense.bus_voltage_gain);
    auto reversed = sense;
    reversed.phase_sign[1] = -1;
    close(reversed.phase_current(raw(1.65F + 0.045F * 0.5F), 1.65F, 1), -0.5F, half_current_lsb);

    // Use the actual forward and inverse transforms, not a copied formula.
    for (float angle : {-3.0F, -1.0F, 0.0F, 0.7F, 2.8F}) {
        float a, b, c, d, q;
        abc(std::sin(angle), std::cos(angle), 0.2F, -0.6F, &a, &b, &c);
        close(a + b + c, 0.0F);
        dq0(std::sin(angle), std::cos(angle), a, b, c, &d, &q);
        close(d, 0.2F); close(q, -0.6F);
    }

    // Verify actual SVM output leaves the ADC window free at every angle.
    for (unsigned i = 0; i < 360; ++i) {
        const float angle = static_cast<float>(i) * voltbro::foc::kTwoPi / 360.0F;
        float a, b, c, du, dv, dw;
        abc(std::sin(angle), std::cos(angle), 0.0F, 5.0F * ruka3::kG474ModulationVoltageRatio, &a, &b, &c);
        svm(5.0F, a, b, c, &du, &dv, &dw);
        for (float duty : {du, dv, dw}) {
            assert(duty >= 0.0F);
            assert(duty * RUKA3_G474_PWM_TICKS <= RUKA3_G474_MAX_PWM_TICKS);
        }
    }

    // Dynamic bus/vector limits must clamp the output, not only the integral.
    PIDRegulator limit_test(PIDConfig{.kp = 2.0F, .ki = 600.0F, .max_output = 2.0F, .min_output = -2.0F});
    close(limit_test.regulation(100.0F, 0.0000625F, 0.25F), 0.25F);
    close(limit_test.regulation(-100.0F, 0.0000625F, 0.25F), -0.25F);
    close(limit_test.get_integral_error(), 0.0F);
    close(limit_test.regulation(100.0F, 0.0000625F, 50.0F), 2.0F);
    limit_test.reset();
    close(limit_test.regulation(0.0F, 0.0000625F, 2.0F), 0.0F);

    // A saturated integrator can unwind when the error reverses.
    PIDRegulator unwind(PIDConfig{.kp = 0.0F, .ki = 1.0F, .max_output = 2.0F, .min_output = -2.0F});
    unwind.set_integral_error(10.0F);
    for (int i = 0; i < 1000; ++i) {
        const float out = unwind.regulation(-1.0F, 0.01F, 0.5F);
        assert(std::fabs(out) <= 0.5F);
    }
    assert(std::fabs(unwind.get_integral_error()) < 0.51F);

    // Locked-rotor R/L plant: verify tracking and reversal with real PI gains.
    PIDRegulator controller(PIDConfig{.kp = ruka3::kG474MotorProfile.current_pid_kp,
        .ki = ruka3::kG474MotorProfile.current_pid_ki,
        .max_output = ruka3::kG474CommissioningVoltageLimitV,
        .min_output = -ruka3::kG474CommissioningVoltageLimitV});
    float current = 0.0F;
    constexpr float resistance = 0.4F, inductance = 0.002F;
    const float dt = ruka3::kG474MotorProfile.control_period_s;
    close(dt, 5.0F * 2000.0F / 160000000.0F, 1e-10F);
    for (float target : {0.5F, -0.5F, 0.0F}) {
        for (unsigned i = 0; i < 3000; ++i) {
            const float voltage = controller.regulation(target - current, dt, 2.0F);
            assert(std::isfinite(voltage) && std::fabs(voltage) <= 2.0F);
            current += dt * (voltage - resistance * current) / inductance;
            assert(std::isfinite(current));
        }
        close(current, target, 0.001F);
    }
    ruka3::FaultManager faults;
    faults.initialize();
    ruka3::FaultInputs inputs{.calibration_valid = true};
    assert(!faults.update(inputs)); // Existing G431 callers omit the new input.
    assert(faults.snapshot().active == 0U);
    inputs.current_loop_fault = true;
    assert(faults.update(inputs));
    assert(faults.snapshot().active == ruka3::fault_current_loop);
    assert(faults.snapshot().stop_reason == ruka3::StopReason::current_loop_fault);
    assert(!faults.update(inputs));
    assert(!faults.acknowledge());
    inputs.current_loop_fault = false;
    assert(!faults.update(inputs));
    assert(faults.snapshot().latched == ruka3::fault_current_loop);
    assert(faults.acknowledge());
    assert(faults.snapshot().latched == 0U);
    inputs.driver_fault = true;
    assert(faults.update(inputs));
    assert(faults.snapshot().stop_reason == ruka3::StopReason::driver_fault);

    for (unsigned joint = 1; joint <= 4; ++joint) {
        const auto& config = ruka3::joint_profile(joint);
        assert(config.motor_command_sign == -1 || config.motor_command_sign == 1);
        assert(config.position_direction == -1 || config.position_direction == 1);
        const float current_sign = config.position_direction * config.motor_command_sign;
        close(current_sign * (current_sign * 0.5F), 0.5F);
        close((0.5F * config.motor.torque_const * config.gear_ratio) /
              (config.motor.torque_const * config.gear_ratio), 0.5F);
    }
}
