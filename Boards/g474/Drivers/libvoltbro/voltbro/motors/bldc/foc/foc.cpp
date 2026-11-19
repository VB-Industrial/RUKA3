#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "foc.hpp"
#include "fusion_math.hpp"
#ifdef RUKA3_FOC_TRACE
#include "foc_trace_hw.hpp"
#endif

#include "arm_math.h"
#include "stm32g4xx_ll_cordic.h"

#include "voltbro/math/transform.hpp"

#ifdef FOC_PROFILE
struct FOCProfile {
    volatile uint32_t sensors = 0;
    volatile uint32_t currents = 0;
    volatile uint32_t outer_loop = 0;
    volatile uint32_t pwm = 0;
    volatile uint32_t total = 0;
};
static volatile FOCProfile foc_profile;

struct SensorsProfile {
    volatile uint32_t inverter = 0;
    volatile uint32_t angle = 0;
    volatile uint32_t kalman = 0;
    volatile uint32_t total = 0;
};
static volatile SensorsProfile sensors_profile;

struct KalmanProfile {
    volatile uint32_t start = 0;
    volatile uint32_t mid = 0;
    volatile uint32_t end = 0;
    volatile uint32_t total = 0;
};
static volatile KalmanProfile kalman_profile;
#endif

#if defined(MONITOR)
volatile float i_q_error, i_d_error;
float d_response, q_response;
volatile float control_error_glob = 0;
volatile float controller_response_glob = 0;
volatile float value_foc_p = 0;
volatile float value_foc_v = 0;
volatile float value_foc_p_kp = 0;
volatile float value_foc_v_kp = 0;
volatile float value_foc_t = 0;
volatile encoder_data raw_value = 0;
volatile float monitor_dtc_u = 0;
volatile float monitor_dtc_v = 0;
volatile float monitor_dtc_w = 0;
volatile float monitor_motor_target = 0;
volatile int monitor_motor_point_type = 0;
#endif

namespace {
float wrap_0_pi2(float angle) {
    angle = mfmod(angle, pi2);
    if (angle < 0.0f) {
        angle += pi2;
    }
    return angle;
}
}

void FOC::update_angle() {
    encoder.update_value();

    #ifndef MONITOR
    encoder_data raw_value;
    #endif
    raw_value = encoder.get_value();
    int offset_value = (int)raw_value - encoder.electric_offset;
    if (lookup_table != nullptr) {
        offset_value -= (*lookup_table)[raw_value >> 3];
    }
    static const float cpr_offset = (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    offset_value -= cpr_offset;
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    raw_elec_angle = offset_value * (pi2 / (float)encoder.CPR);
#ifdef RUKA3_FOC_TRACE
    if(auto* r=ruka3::foc_trace::frame(ruka3::foc_trace::Profile::angle)) {
        r->words[12]=raw_value;r->f(13,raw_elec_angle);
        r->words[27]=static_cast<std::uint32_t>(encoder.electric_offset);
        r->words[28]=lookup_table?static_cast<std::uint32_t>((*lookup_table)[raw_value>>3]):0;
        r->words[25]=encoder.CPR;r->words[26]=drive_info.common.ppairs;
        r->words[29]=(encoder.is_inverted?1U:0U)|(lookup_table?2U:0U);
    }
#endif
}

bool FOC::run_wrapped_kalman(float raw_angle, WrappedKalmanState& state, float& predicted_angle, float& predicted_velocity) {
    raw_angle = wrap_0_pi2(raw_angle);

    if (!state.initialized) {
        state.initialized = true;
        state.prev_angle = raw_angle;
        state.theta_hat = raw_angle;
        predicted_angle = raw_angle;
        predicted_velocity = 0.0f;
        return false;
    }

    float travel = raw_angle - state.prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }

//#pragma region KALMAN_PAPER
    /*
     * Source: "A digital speed filter for motion control drives
     *          with a low resolution position encoder",
     * AUTOMATIKA, 44(2003),
     * A. Bellini, S. Bifaretti, S. Constantini
     */
    // TODO: get acceleration from inverter?
    // (11)
    float nTh = state.theta_hat + state.omega_hat * T + (state.epsilon_hat + filters_config.expected_a) * (T * T) / 2.0f;
    float nW = state.omega_hat + (state.epsilon_hat + filters_config.expected_a) * T;
    float nE = state.epsilon_hat;

    nTh = wrap_0_pi2(nTh);

#ifdef RUKA3_FOC_TRACE
    const float trace_previous=state.prev_angle;
#endif
    // (19)
    state.theta_hat = nTh + filters_config.g1 * travel;
    state.omega_hat = nW + filters_config.g2 * travel;
    state.epsilon_hat = nE + filters_config.g3 * travel;
//#pragma endregion KALMAN_PAPER
#ifdef RUKA3_FOC_TRACE
    if(&state==&motor_kalman_state) {
        if(auto* r=ruka3::foc_trace::frame(ruka3::foc_trace::Profile::angle)) {
            r->words[10]|=ruka3::foc_trace::observer_valid;
            r->f(14,nTh);r->f(15,state.theta_hat);r->f(16,state.omega_hat);
            r->f(17,state.epsilon_hat);r->f(18,nW);r->f(20,travel);r->f(21,trace_previous);
            r->f(22,T);r->f(30,filters_config.g1);r->f(31,filters_config.g2);
        }
    }
#endif
    predicted_angle = nTh;
    predicted_velocity = nW;
    state.prev_angle = nTh;
    return true;
}

void FOC::apply_kalman() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    float predicted_angle = raw_elec_angle;
    float predicted_velocity = 0.0f;
    run_wrapped_kalman(raw_elec_angle, motor_kalman_state, predicted_angle, predicted_velocity);
#ifdef FOC_PROFILE
    kalman_profile.start = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif

    const float ab = pi2 / (float)drive_info.common.ppairs;
    elec_angle = (float)drive_info.common.ppairs * mfmod(predicted_angle, ab);
    shaft_velocity = predicted_velocity / drive_info.common.gear_ratio;

#ifdef FOC_PROFILE
    kalman_profile.mid = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
    kalman_profile.end = DWT->CYCCNT - t_start;
    kalman_profile.total = DWT->CYCCNT - start_total;
#endif
}

void FOC::update_gear_angle() {
    if (fusion_config.gear_encoder == nullptr) {
        gear_angle = voltbro::foc::wrap_pi(
            static_cast<float>(fusion_config.gear_angle_sign) * BLDCController::get_angle()
        );
        gear_velocity = static_cast<float>(fusion_config.gear_angle_sign) * BLDCController::get_velocity();
        return;
    }

    fusion_config.gear_encoder->update_value();
    const int32_t current_ticks = static_cast<int32_t>(fusion_config.gear_encoder->get_value());
    const int32_t cpr = static_cast<int32_t>(fusion_config.gear_encoder->CPR);
    int32_t zero_ticks = static_cast<int32_t>(fusion_config.gear_encoder_zero);
    if (fusion_config.gear_angle_sign < 0) {
        // The encoder driver already flips measured ticks, so the configured zero
        // has to be moved into that same frame before angle reconstruction.
        zero_ticks = cpr - zero_ticks;
    }
    const auto* calibration = fusion_config.output_calibration;
    const std::uint16_t physical_raw = fusion_config.gear_angle_sign < 0 ? cpr - current_ticks : current_ticks;
    std::int32_t corrected{};
    const float raw_angle = calibration != nullptr && calibration->calibrated_position_ticks(physical_raw, corrected) ?
        voltbro::foc::wrap_pi(fusion_config.gear_angle_sign * voltbro::foc::kTwoPi * corrected / (cpr + 1)) :
        voltbro::foc::gear_angle_from_ticks(current_ticks, zero_ticks, cpr);
    gear_measurement_angle = raw_angle;

    float predicted_angle = raw_angle;
    float predicted_velocity = 0.0f;
    run_wrapped_kalman(raw_angle, gear_kalman_state, predicted_angle, predicted_velocity);

    gear_angle = voltbro::foc::wrap_pi(predicted_angle);
    gear_velocity = predicted_velocity;
}

void FOC::update_fusion_angle() {
    const float raw_motor_angle = voltbro::foc::wrap_pi(BLDCController::get_angle());
    const float motor_velocity = BLDCController::get_velocity();

    if (!fusion_offset_initialized) {
        // Align motor-side estimate to the gear encoder reference before fusion.
        motor_fusion_offset = voltbro::foc::wrap_pi(gear_angle - raw_motor_angle);
        fusion_offset_initialized = true;
    }

    const float motor_angle = voltbro::foc::wrap_pi(raw_motor_angle + motor_fusion_offset);
    fusion_angle = voltbro::foc::fuse_angle(motor_angle, gear_angle, fusion_config.blend);

    const float raw_fusion_velocity = voltbro::foc::fuse_velocity(
        motor_velocity,
        gear_velocity,
        fusion_config.velocity_blend
    );
    const float velocity_lpf = std::clamp(fusion_config.velocity_lpf_coefficient, 0.0f, 1.0f);
    if (!fusion_velocity_initialized) {
        fusion_velocity = raw_fusion_velocity;
        fusion_velocity_initialized = true;
    } else {
        fusion_velocity = fusion_velocity + velocity_lpf * (raw_fusion_velocity - fusion_velocity);
    }
}

void FOC::update_shaft_angle() {
    static float prev_elec_angle = elec_angle;
    static int32_t elec_turns = 0;
    float filtered_travel = elec_angle - prev_elec_angle;
    prev_elec_angle = elec_angle;
    if (filtered_travel < -PI) {
        elec_turns += 1;
    } else if (filtered_travel > PI) {
        elec_turns -= 1;
    }
    float elec_unwrapped = (float)elec_turns * pi2 + elec_angle;
    shaft_angle = elec_unwrapped / ( (float)drive_info.common.ppairs * drive_info.common.gear_ratio );
}

void FOC::update_sensors() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    inverter.update();
#ifdef FOC_PROFILE
    sensors_profile.inverter = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    update_angle();
#ifdef FOC_PROFILE
    sensors_profile.angle = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    apply_kalman();
    update_shaft_angle();
    update_gear_angle();
    update_fusion_angle();
#ifdef FOC_PROFILE
    sensors_profile.kalman = DWT->CYCCNT - t_start;
    sensors_profile.total = DWT->CYCCNT - start_total;
#endif
}


void FOC::update() {
#ifdef RUKA3_FOC_TRACE
    namespace trace=ruka3::foc_trace;
    if(auto* r=trace::recorder.current()) {
        r->words[10]|=(is_on()?trace::pwm_on:0U)|(static_cast<std::uint32_t>(point_type)<<1U);
    }
    if(auto* r=trace::frame(trace::Profile::control)) {
        for(unsigned i=12;i<29;++i) r->f(i,std::numeric_limits<float>::quiet_NaN());
    }
#endif
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    update_sensors();
#ifdef FOC_PROFILE
    foc_profile.sensors = DWT->CYCCNT - t_start;
#endif

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
#ifdef RUKA3_FOC_TRACE
    const std::uint32_t trace_cos=LL_CORDIC_ReadData(CORDIC);
    const std::uint32_t trace_sin=LL_CORDIC_ReadData(CORDIC);
    struct {int32_t cosOutput;int32_t sinOutput;} elec_angles_q31{
        -(int32_t)trace_cos,-(int32_t)trace_sin}; // Original boundary arithmetic intentionally retained.
    if(auto* r=trace::recorder.current()) {
        if(trace_cos==0x80000000U || trace_sin==0x80000000U) r->words[10]|=trace::cordic_min;
        if(trace::recorder.profile()==trace::Profile::angle) {
            r->words[23]=trace_cos;r->words[24]=trace_sin;r->f(19,elec_angle);
        } else if(trace::recorder.profile()==trace::Profile::measurement) {
            r->words[30]=encoder.get_value();r->f(31,elec_angle);
        }
    }
#else
    struct {
        int32_t cosOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
        int32_t sinOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine
    } elec_angles_q31;
#endif
    struct {
        float c;
        float s;
    } elec_angles;
    //arm_q31_to_float(&elec_angles_q31.cosOutput, &elec_angles.c, 2);

    elec_angles.c = (float32_t)elec_angles_q31.cosOutput / 2147483648.0f;  // convert to float from q31
    elec_angles.s = (float32_t)elec_angles_q31.sinOutput / 2147483648.0f;  // convert to float from q31

    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    // LPF for motor current
    float tempD, tempQ;
    // dq0 transform on currents
    dq0(elec_angles.s, elec_angles.c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
    const float diff_D = I_D - tempD;
    const float diff_Q = I_Q - tempQ;

    I_D = I_D - (filters_config.I_lpf_coefficient * diff_D);
    I_Q = I_Q - (filters_config.I_lpf_coefficient * diff_Q);
#ifdef RUKA3_FOC_TRACE
    if(auto* r=trace::frame(trace::Profile::measurement)) {
        r->f(20,tempD);r->f(21,tempQ);r->f(22,I_D);r->f(23,I_Q);
    }
    if(auto* r=trace::frame(trace::Profile::control)) {r->f(17,I_D);r->f(18,I_Q);}
#endif
    #ifdef FOC_PROFILE
    foc_profile.currents = DWT->CYCCNT - t_start;
    #endif

    const float gear_ratio_f = static_cast<float>(drive_info.common.gear_ratio);
    const float busV = inverter.get_busV();
#if defined(MONITOR)
    monitor_motor_target = target;
    monitor_motor_point_type = static_cast<int>(point_type);
#endif

    shaft_torque = drive_info.current_direction * I_Q * drive_info.torque_const * gear_ratio_f;

    // Keep measuring while disarmed, but never integrate errors or write PWM.
    if (!is_on()) {
#ifdef RUKA3_FOC_TRACE
        trace::d_pulse.cancel();
#endif
        return;
    }
    if (!(busV > 0.0F)) {
        current_fault = CurrentLoopFault::invalid_bus_voltage;
        stop();
        return;
    }
    if (std::fabs(inverter.get_A()) > drive_info.phase_current_trip_a ||
        std::fabs(inverter.get_B()) > drive_info.phase_current_trip_a ||
        std::fabs(inverter.get_C()) > drive_info.phase_current_trip_a) {
        current_fault = CurrentLoopFault::phase_overcurrent;
        stop();
        return;
    }
    const float available_voltage = std::min(voltage_limit, busV * drive_info.modulation_voltage_ratio);

#ifdef RUKA3_FOC_TRACE
    if(point_type!=SetPointType::VOLTAGE || target!=0.0F) trace::d_pulse.cancel();
#endif
    if (point_type == SetPointType::VOLTAGE) {
        V_d = 0;
        V_q = target;
#ifdef RUKA3_FOC_TRACE
        const auto pulse_flags=trace::d_pulse.step(V_d,V_q);
        if(auto* r=trace::recorder.current()) r->words[10]|=pulse_flags;
#endif
    }
    else {
        #ifndef MONITOR
        float i_d_error, i_q_error, d_response, q_response;
        #endif

        i_d_error = -I_D;
        d_response = d_reg.regulation(i_d_error, T, available_voltage);
        V_d = d_response;

        i_q_set = 0.0f;
        #ifdef FOC_PROFILE
        t_start = DWT->CYCCNT;
        #endif
        if (point_type == SetPointType::UNIVERSAL) {
            #ifdef MONITOR
            value_foc_p = foc_target.angle;
            value_foc_v = foc_target.velocity;
            value_foc_p_kp = foc_target.angle_kp;
            value_foc_v_kp = foc_target.velocity_kp;
            value_foc_t = foc_target.torque;
            #endif
            // Adapter already converted P/D to motor-side gains, as in VBDrive.
            // Feed-forward torque retains output-shaft units.
            i_q_set = 1.0f / drive_info.torque_const * (
                foc_target.angle_kp * std::remainder(foc_target.angle - get_fusion_angle(), pi2) +
                foc_target.velocity_kp * (foc_target.velocity - get_fusion_velocity()) +
                foc_target.torque / gear_ratio_f
            );
        }
        else if (point_type == SetPointType::CURRENT) {
            i_q_set = target;
        }
        else if (point_type == SetPointType::TORQUE) {
            i_q_set = target / drive_info.torque_const / gear_ratio_f;
        }
        else {
            float control_error = 0;
            if (point_type == SetPointType::POSITION) {
                control_error = target - shaft_angle;
            }
            else if (point_type == SetPointType::VELOCITY) {
                control_error = target - shaft_velocity;
            }
            float controller_response = control_reg.regulation(control_error, T, false);
            #ifdef MONITOR
            control_error_glob = control_error;
            controller_response_glob = controller_response;
            #endif
            i_q_set = controller_response;
        }

#ifdef RUKA3_FOC_TRACE
        if(auto* r=trace::frame(trace::Profile::control)) r->f(13,i_q_set*drive_info.current_direction);
#endif
        const float abs_max_current_from_torque = (drive_info.max_torque / drive_info.torque_const / gear_ratio_f);
        if (fabs(i_q_set) > fabs(abs_max_current_from_torque)) {
            i_q_set = copysign(abs_max_current_from_torque, i_q_set);
        }
        if (
            drive_limits.current_limit > 0.0f &&
            (fabs(i_q_set) > fabs(drive_limits.current_limit))
        ) {
            i_q_set = copysign(drive_limits.current_limit, i_q_set);
        }
        // absolute limit on currents defined by the hardware safe operation region
        if (fabs(i_q_set) > drive_info.max_current) {
            i_q_set = copysign(drive_info.max_current, i_q_set);
        }

        i_q_set *= drive_info.current_direction;

        i_q_error = i_q_set - I_Q;
        // Give Id priority and keep the voltage vector inside the SVM circle.
        const float q_voltage_limit = std::sqrt(std::max(0.0F,
            available_voltage * available_voltage - V_d * V_d));
        q_response = q_reg.regulation(i_q_error, T, q_voltage_limit);
        V_q = q_response;
#ifdef RUKA3_FOC_TRACE
        if(auto* r=trace::frame(trace::Profile::control)) {
            r->words[10]|=trace::current_loop_ran;
            if(d_reg.trace().saturated) r->words[10]|=trace::d_saturated;
            if(q_reg.trace().saturated) r->words[10]|=trace::q_saturated;
            r->f(12,0);r->f(14,i_q_set);r->f(15,i_d_error);r->f(16,i_q_error);
            r->f(19,d_reg.trace().raw);r->f(20,q_reg.trace().raw);
            r->f(21,V_d);r->f(22,V_q);r->f(25,available_voltage);r->f(26,q_voltage_limit);
            r->f(27,d_reg.get_integral_error());r->f(28,q_reg.get_integral_error());
        }
#endif
        #ifdef FOC_PROFILE
        foc_profile.outer_loop = DWT->CYCCNT - t_start;
        #endif
    }

#ifdef RUKA3_FOC_TRACE
    if(auto* r=trace::frame(trace::Profile::control)) {
        if(V_d*V_d+V_q*V_q>available_voltage*available_voltage) r->words[10]|=trace::vector_limited;
        r->f(25,available_voltage);
    }
#endif
    #if defined(MONITOR)
    float V_d_local = V_d;
    float V_q_local = V_q;
    limit_norm(&V_d_local, &V_q_local, available_voltage);
    V_d = V_d_local;
    V_q = V_q_local;
    #else
    limit_norm(&V_d, &V_q, available_voltage);
    #endif

    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    // inverse dq0 transform on voltages
    #if defined(MONITOR)
    abc(elec_angles.s, elec_angles.c, V_d_local, V_q_local, &v_u, &v_v, &v_w);
    #else
    abc(elec_angles.s, elec_angles.c, V_d, V_q, &v_u, &v_v, &v_w);
    #endif
    // space vector modulation
    svm(busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);
#if defined(MONITOR)
    monitor_dtc_u = dtc_u;
    monitor_dtc_v = dtc_v;
    monitor_dtc_w = dtc_w;
#endif

    DQs[0] = (uint16_t)(float(full_pwm + 1) * dtc_u);
    DQs[1] = (uint16_t)(float(full_pwm + 1) * dtc_v);
    DQs[2] = (uint16_t)(float(full_pwm + 1) * dtc_w);

#ifdef RUKA3_FOC_TRACE
    if(auto* r=trace::frame(trace::Profile::control)) {
        if(DQs[0]>drive_info.max_pwm_ticks || DQs[1]>drive_info.max_pwm_ticks || DQs[2]>drive_info.max_pwm_ticks) r->words[10]|=trace::pwm_clipped;
        r->f(23,V_d);r->f(24,V_q);r->f(31,busV);
    }
#endif
    set_pwm();
#ifdef RUKA3_FOC_TRACE
    if(auto* r=trace::frame(trace::Profile::control)) {
        r->words[29]=trace::pack16(DQs[0],DQs[1]);r->words[30]=DQs[2];
    }
#endif
    #ifdef FOC_PROFILE
    foc_profile.pwm = DWT->CYCCNT - t_start;
    #endif

#ifdef FOC_PROFILE
    foc_profile.total = DWT->CYCCNT - start_total;
#endif
}


#endif
#endif
