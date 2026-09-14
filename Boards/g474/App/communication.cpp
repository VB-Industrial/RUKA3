#include "app.h"
#include "fdcan.h"

#include <cyphal/cyphal.h>
#include <cyphal/allocators/o1/o1_allocator.h>
#include <cyphal/providers/G4CAN.h>
#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>

#include <voltbro/utils.hpp>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>

//VB part
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/torque/Scalar_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/si/unit/voltage/Scalar_1_0.h>

//MOVEIT2 part
#include "robot_config.h"
#include "uavcan/primitive/scalar/Integer32_1_0.h"
#include "uavcan/primitive/array/Real16_1_0.h"
#include "voltbro/foc/command_1_0.h"
#include "voltbro/foc/state_simple_1_0.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>

#include <uavcan/node/GetInfo_1_0.h>
#include <cstdint>
#include <cmath>

//moveit2 part
#ifdef MOVEIT

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, voltbro_foc_command_1_0)
TYPE_ALIAS(FocState, voltbro_foc_state_simple_1_0)

TYPE_ALIAS(Angle, uavcan_si_unit_angle_Scalar_1_0)
TYPE_ALIAS(AngularVelocity, uavcan_si_unit_angular_velocity_Scalar_1_0)
TYPE_ALIAS(Torque, uavcan_si_unit_torque_Scalar_1_0)
TYPE_ALIAS(Voltage, uavcan_si_unit_voltage_Scalar_1_0)

static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION; 

UtilityConfig utilities(micros_64, Error_Handler);

static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

const int NUMBER_OF_REGISTERS = 6;

static float angle_in;
static float vel_in;
static float torque_in;
static float position_feedback_gain_in = 0.5f;
static float velocity_feedback_gain_in = 0.02f;
static float torque_feedback_gain_in = 0.5f;

static inline float apply_dh_rotation(float value) {
    return static_cast<float>(DH_rotation) * value;
}

static inline float wrap_pi(float angle) {
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

static bool parse_register_numeric(const uavcan_register_Value_1_0& value, float& parsed) {
    if (value._tag_ == 12 && value.real64.value.count > 0) {
        parsed = static_cast<float>(value.real64.value.elements[0]);
        return true;
    }
    if (value._tag_ == 13 && value.real32.value.count > 0) {
        parsed = value.real32.value.elements[0];
        return true;
    }
    if (value._tag_ == 14 && value.real16.value.count > 0) {
        parsed = static_cast<float>(value.real16.value.elements[0]);
        return true;
    }
    if (value._tag_ == 4 && value.integer64.value.count > 0) {
        parsed = static_cast<float>(value.integer64.value.elements[0]);
        return true;
    }
    if (value._tag_ == 5 && value.integer32.value.count > 0) {
        parsed = static_cast<float>(value.integer32.value.elements[0]);
        return true;
    }
    if (value._tag_ == 6 && value.integer16.value.count > 0) {
        parsed = static_cast<float>(value.integer16.value.elements[0]);
        return true;
    }
    if (value._tag_ == 7 && value.integer8.value.count > 0) {
        parsed = static_cast<float>(value.integer8.value.elements[0]);
        return true;
    }
    if (value._tag_ == 8 && value.natural64.value.count > 0) {
        parsed = static_cast<float>(value.natural64.value.elements[0]);
        return true;
    }
    if (value._tag_ == 9 && value.natural32.value.count > 0) {
        parsed = static_cast<float>(value.natural32.value.elements[0]);
        return true;
    }
    if (value._tag_ == 10 && value.natural16.value.count > 0) {
        parsed = static_cast<float>(value.natural16.value.elements[0]);
        return true;
    }
    if (value._tag_ == 11 && value.natural8.value.count > 0) {
        parsed = static_cast<float>(value.natural8.value.elements[0]);
        return true;
    }
    return false;
}

void heartbeat() {
    static uint32_t uptime = 0;
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
        .uptime = uptime,
        .health = {CYPHAL_HEALTH_STATUS},
        .mode = {CYPHAL_MODE},
        .vendor_specific_status_code = 0
    };
    uptime += 1;

    if (_is_cyphal_on) {
        cyphal_interface->send_msg<HBeat>(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
        );
    }
}


class JSSub: public AbstractSubscription<JS_msg> {
public:
	JSSub(InterfacePtr interface, CanardPortID):
    AbstractSubscription<JS_msg>(interface, JS_SUB_PORT_ID) {};
    void handler(const voltbro_foc_command_1_0& js_in, CanardRxTransfer*) override
    {
        FOC& motor = get_motor();
        if constexpr (!js_control_enabled) {
            motor.set_voltage_point(0.0f);
            return;
        }
        angle_in = apply_dh_rotation(js_in.angle.radian);
        vel_in = apply_dh_rotation(js_in.velocity.radian_per_second);
        torque_in = apply_dh_rotation(js_in._torque.newton_meter);
        position_feedback_gain_in = js_in.angle_kp.value;
        velocity_feedback_gain_in = js_in.velocity_kp.value;

        const float current_angle = motor.get_fusion_angle();
        const float current_velocity = motor.get_fusion_velocity();
        const float position_error = wrap_pi(angle_in - current_angle);
        const float velocity_error = vel_in - current_velocity;

        constexpr float kAcquireThreshold = 0.5f;
        constexpr float kAcquireVoltage = 0.6f;
        constexpr float kTrackMaxVoltage = 11.5f;
        constexpr float kMaxVelocityFeedbackGain = 1.5f;

        float voltage_command = 0.0f;
        if (std::fabs(position_error) > kAcquireThreshold) {
            voltage_command = (position_error > 0.0f) ? kAcquireVoltage : -kAcquireVoltage;
        } else {
            const float kp = position_feedback_gain_in;
            const float kd = std::clamp(velocity_feedback_gain_in, 0.0f, kMaxVelocityFeedbackGain);
            const float ke = torque_feedback_gain_in;
            voltage_command = kp * position_error + kd * velocity_error + ke * torque_in;
        }
        voltage_command = static_cast<float>(motor_command_sign) *
            std::clamp(voltage_command, -kTrackMaxVoltage, kTrackMaxVoltage);
        motor.set_voltage_point(voltage_command);
    }
};


class RegisterListReader : public AbstractSubscription<RegisterListRequest> {
public:
    RegisterListReader(InterfacePtr interface): AbstractSubscription<RegisterListRequest>(
        interface,
        uavcan_register_List_1_0_FIXED_PORT_ID_,
        CanardTransferKindRequest
    ) {};
    void handler(const RegisterListRequest::Type&, CanardRxTransfer*) override;
};

RegisterListReader* reg_list_service;

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<NUMBER_OF_REGISTERS>> registers_handler;

ReservedObject<JSSub> js_sub;


static inline bool register_value_is_empty(const uavcan_register_Value_1_0& value) {
    return value._tag_ == 0;
}

static inline bool parse_register_i32(const uavcan_register_Value_1_0& value, int32_t& out) {
    if (value._tag_ == 5 && value.integer32.value.count > 0) {
        out = value.integer32.value.elements[0];
        return true;
    }
    if (value._tag_ == 10 && value.natural16.value.count > 0) {
        out = value.natural16.value.elements[0];
        return true;
    }
    return false;
}

static inline void fill_register_i32(uavcan_register_Value_1_0& value, int32_t data) {
    value._tag_ = 5;
    value.integer32.value.elements[0] = data;
    value.integer32.value.count = 1;
}

void set_pos_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    FOC& motor = get_motor();
    constexpr float kPosSetKp = 0.002f;
    constexpr float kPosSetMaxVoltage = 1.2f;
    constexpr int32_t kPosSetDeadbandTicks = 8;

    const auto clamp_voltage = [](float value) {
        if (value > kPosSetMaxVoltage) {
            return kPosSetMaxVoltage;
        }
        if (value < -kPosSetMaxVoltage) {
            return -kPosSetMaxVoltage;
        }
        return value;
    };

    if (register_value_is_empty(v_in)) {
        fill_register_i32(v_out, static_cast<int32_t>(get_gear_encoder().get_value()));
    } else {
        int32_t target_ticks = 0;
        if (parse_register_i32(v_in, target_ticks)) {
            while (true) {
                const int32_t current_ticks = static_cast<int32_t>(get_gear_encoder().get_value());
                const int32_t error_ticks = target_ticks - current_ticks;
                if (error_ticks <= kPosSetDeadbandTicks && error_ticks >= -kPosSetDeadbandTicks) {
                    motor.set_voltage_point(0.0f);
                    fill_register_i32(v_out, current_ticks);
                    break;
                }
                motor.set_voltage_point(clamp_voltage(static_cast<float>(error_ticks) * kPosSetKp));
                HAL_Delay(1);
            }
        } else {
            fill_register_i32(v_out, static_cast<int32_t>(get_gear_encoder().get_value()));
        }
    }

    response.persistent = true;
    response._mutable = true;
}


void move_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    FOC& motor = get_motor();
    float requested_voltage = 0.0f;
    float applied_voltage = 0.0f;
    const bool has_numeric_value = parse_register_numeric(v_in, requested_voltage);

    if (has_numeric_value) {
        applied_voltage = static_cast<float>(motor_command_sign) * requested_voltage;
        motor.set_voltage_point(applied_voltage);
    }

    response.persistent = true;
    response._mutable = true;
    v_out._tag_ = 13;
    v_out.real32.value.elements[0] = applied_voltage;
    v_out.real32.value.count = 1;
}


void pos_get_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    const FOC& motor = get_motor();
    response.persistent = true;
    response._mutable = true;
    (void)v_in;
    v_out._tag_ = 13;
    v_out.real32.value.elements[0] = static_cast<float>(motor.get_encoder().get_value());
    v_out.real32.value.elements[1] = static_cast<float>(get_gear_encoder().get_value());
    v_out.real32.value.elements[2] = motor.get_fusion_angle();
    v_out.real32.value.count = 3;
}


void fus_dbg_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    const FOC& motor = get_motor();
    response.persistent = true;
    response._mutable = true;
    (void)v_in;
    v_out._tag_ = 13;
    v_out.real32.value.elements[0] = motor.get_aligned_motor_angle();
    v_out.real32.value.elements[1] = motor.get_gear_angle();
    v_out.real32.value.elements[2] = motor.get_fusion_angle();
    v_out.real32.value.count = 3;
}


void get_pd_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    (void)v_in;
    response.persistent = true;
    response._mutable = true;
    v_out._tag_ = 13;
    v_out.real32.value.elements[0] = position_feedback_gain_in;
    v_out.real32.value.elements[1] = std::clamp(velocity_feedback_gain_in, 0.0f, 1.5f);
    v_out.real32.value.count = 2;
}


void set_offset_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    FOC& motor = get_motor();

    if (!register_value_is_empty(v_in)) {
        int32_t requested_offset = 0;
        if (parse_register_i32(v_in, requested_offset)) {
            motor.set_encoder_electric_offset(requested_offset);
        } else {
            float requested_offset_f = 0.0f;
            if (parse_register_numeric(v_in, requested_offset_f)) {
                motor.set_encoder_electric_offset(static_cast<int>(requested_offset_f));
            }
        }
    }

    response.persistent = true;
    response._mutable = true;
    fill_register_i32(v_out, motor.get_encoder_electric_offset());
}


void setup_cyphal_comms() {

    constexpr auto& profile = ruka3::joint_profile(RUKA3_JOINT_INDEX);

    cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        profile.node_id,
        &hfdcan1,
        200,
        utilities
    ));

    node_info_reader.create(
        cyphal_interface,
        "ruka3.g474",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );

    registers_handler.create(
        std::array<RegisterDefinition, NUMBER_OF_REGISTERS>{{
            {"set", set_offset_handler},
            {"set_pos", set_pos_handler},
            {"pos_get", pos_get_handler},
            {"fus_dbg", fus_dbg_handler},
            {"get_PD", get_pd_handler},
            {"move", move_handler},
        }},
        cyphal_interface
    );



    js_sub.create(cyphal_interface, JS_SUB_PORT_ID);
}

void setup_can() {
    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler,
        0
    ))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))

    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))

    _is_cyphal_on = true;
    CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;
    


}

#if defined(DEBUG) || defined(MONITOR)
#define IS_GLOBAL_CONTROL_VARIABLES
volatile static float send_pos = 0;
volatile static float send_vel = 0;
#endif

void send_JS()
{
	static CanardTransferID int_transfer_id = 0;
    FOC& motor = get_motor();

    #if defined(DEBUG) || defined(MONITOR)
    send_pos = apply_dh_rotation(motor.get_fusion_angle());
    send_vel = apply_dh_rotation(motor.get_fusion_velocity());
    #endif

    voltbro_foc_state_simple_1_0 js_msg{};
    js_msg.timestamp.microsecond = micros_64();
    js_msg.angle.radian = apply_dh_rotation(motor.get_fusion_angle());
    js_msg.velocity.radian_per_second = apply_dh_rotation(motor.get_fusion_velocity());
    js_msg._torque.newton_meter = apply_dh_rotation(motor.get_torque());
    js_msg.current.ampere = 0.0f;
    js_msg.bus_voltage.volt = motor.get_voltage();
    js_msg.mcu_temp.kelvin = 0.0f;
    js_msg.stator_temp.kelvin = 0.0f;
    js_msg.has_fault.value = false;

    cyphal_interface->send_msg<FocState>(
		&js_msg,
		AGENT_JS_SUB_PORT,
		&int_transfer_id
	);
}
#endif


static millis odom_report_time = 0;


#ifdef MOVEIT
void comms_loop(millis cur_millis) {
    cyphal_interface->loop();
      EACH_N(cur_millis, odom_report_time, 5, {
        send_JS();
    })
}
#endif
