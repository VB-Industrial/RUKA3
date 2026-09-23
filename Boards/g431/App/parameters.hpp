#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <variant>

enum class ParameterType : uint8_t { REAL32, REAL32_ARRAY, NATURAL32, INTEGER32, BIT, STRING };

enum class ParameterId : uint8_t {
    NODE_ID, ARM, BOOTLOADER, CMD_ERRORS, MODEL, REVISION,
    BUS_VOLTAGE, BUS_CURRENT, TEMP_MCU, TEMP_STATOR, IS_FAULT,
    GET_ANGLE, CALIB_VALID, CALIB_INVERTED, MOTOR_CALIB, DRIVER_WAKE, ZERO_SET, SET_LIMIT, MOVE, POS_SET,
    CAN_ERR_TOTAL, CAN_RX_ERR, CAN_TX_ERR, CAN_ERR_LOG, CAN_ARB_LEC,
    CAN_DATA_LEC, CAN_WARNING, CAN_PASSIVE, CAN_BUS_OFF,
    CMD_COUNT, CMD_AGE_MS, CMD_TIMEOUT,
    FAULT_ACTIVE, FAULT_LATCHED, FAULT_LEVEL, STOP_REASON, NET_STATE, FAULT_ACK,
    JOINT_STATE,
};

struct ParameterDefinition {
    ParameterId id;
    std::string_view name;
    ParameterType type;
    bool is_mutable;
    bool is_persistent;
};

using AngleDiagnostics = std::array<float, 34>;
using ParameterValue = std::variant<uint32_t, int32_t, float, bool, std::string_view, AngleDiagnostics>;
enum class ParameterWriteResult : uint8_t { OK, READ_ONLY, INVALID, UNAVAILABLE };

inline constexpr std::array<ParameterDefinition, 39> PARAMETER_CATALOG{{
    {ParameterId::NODE_ID,         "get_node_id",   ParameterType::NATURAL32, false, false},
    {ParameterId::ARM,             "set_arm",       ParameterType::BIT,       true,  false},
    {ParameterId::BOOTLOADER,      "set_bootloader",ParameterType::BIT,       true,  false},
    {ParameterId::CMD_ERRORS,      "get_cmd_errors",ParameterType::NATURAL32, false, false},
    {ParameterId::MODEL,           "get_model",     ParameterType::STRING,    false, false},
    {ParameterId::REVISION,        "get_fw_rev",    ParameterType::STRING,    false, false},
    {ParameterId::BUS_VOLTAGE,     "get_bus_voltage",ParameterType::REAL32,   false, false},
    {ParameterId::BUS_CURRENT,     "get_bus_current",ParameterType::REAL32,   false, false},
    {ParameterId::TEMP_MCU,        "get_temp_mcu",  ParameterType::REAL32,    false, false},
    {ParameterId::TEMP_STATOR,     "get_temp_stator",ParameterType::REAL32,   false, false},
    {ParameterId::IS_FAULT,        "get_is_fault",  ParameterType::BIT,       false, false},
    // get_pos: shaft/rotor ticks, motor angle/velocity, electric angle,
    // output angle/velocity/residual, fused angle/velocity/offset/error,
    // motor calibration offset, EEPROM zero, gear/corridor, runtime limits,
    // raw endpoints, limit stage/direction, service mode/target/error,
    // soft limits, EEPROM margin, calibration validity, and the raw
    // motor/output disagreement and the configured mechanical backlash.
    {ParameterId::GET_ANGLE,       "get_pos",       ParameterType::REAL32_ARRAY, false, false},
    {ParameterId::CALIB_VALID,     "get_calib_valid",ParameterType::BIT,      false, false},
    {ParameterId::CALIB_INVERTED,  "get_calib_inv", ParameterType::BIT,       false, false},
    {ParameterId::MOTOR_CALIB,     "set_motor_calib",ParameterType::BIT,      true,  false},
    {ParameterId::DRIVER_WAKE,     "set_drv_wake",  ParameterType::BIT,       true,  false},
    // RUKA2-compatible trigger: an empty register read calibrates zero and
    // returns INTEGER32 0/1.  It deliberately has no writable value.
    {ParameterId::ZERO_SET,        "set_zero",      ParameterType::INTEGER32, false, false},
    {ParameterId::SET_LIMIT,       "set_limits",    ParameterType::NATURAL32, true,  false},
    {ParameterId::MOVE,            "set_voltage",   ParameterType::REAL32,    true,  false},
    {ParameterId::POS_SET,         "set_pos",       ParameterType::REAL32,    true,  false},
    {ParameterId::CAN_ERR_TOTAL,   "get_can_err",   ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_RX_ERR,      "get_can_rx_err",ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_TX_ERR,      "get_can_tx_err",ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_ERR_LOG,     "get_can_err_log",ParameterType::NATURAL32,false, false},
    {ParameterId::CAN_ARB_LEC,     "get_can_arb_lec",ParameterType::NATURAL32,false, false},
    {ParameterId::CAN_DATA_LEC,    "get_can_dat_lec",ParameterType::NATURAL32,false, false},
    {ParameterId::CAN_WARNING,     "get_can_warning",ParameterType::BIT,      false, false},
    {ParameterId::CAN_PASSIVE,     "get_can_passive",ParameterType::BIT,      false, false},
    {ParameterId::CAN_BUS_OFF,     "get_can_bus_off",ParameterType::BIT,      false, false},
    {ParameterId::CMD_COUNT,       "get_cmd_count", ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_AGE_MS,      "get_cmd_age_ms",ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_TIMEOUT,     "get_cmd_timeout",ParameterType::BIT,      false, false},
    {ParameterId::FAULT_ACTIVE,    "get_fault_act", ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_LATCHED,   "get_fault_latch",ParameterType::NATURAL32,false, false},
    {ParameterId::FAULT_LEVEL,     "get_fault_level",ParameterType::NATURAL32,false, false},
    {ParameterId::STOP_REASON,     "get_stop_reason",ParameterType::NATURAL32,false, false},
    {ParameterId::NET_STATE,       "get_net_state", ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_ACK,       "set_fault_ack", ParameterType::BIT,       true,  false},
    {ParameterId::JOINT_STATE,     "get_joint_state",ParameterType::NATURAL32,false, false},
}};

consteval bool parameter_catalog_is_valid() {
    for (size_t i = 0; i < PARAMETER_CATALOG.size(); ++i) {
        if (PARAMETER_CATALOG[i].name.empty() || PARAMETER_CATALOG[i].name.size() > 15) return false;
        for (size_t j = i + 1; j < PARAMETER_CATALOG.size(); ++j) {
            if (PARAMETER_CATALOG[i].name == PARAMETER_CATALOG[j].name) return false;
        }
    }
    return true;
}
static_assert(parameter_catalog_is_valid());

inline constexpr std::array<std::string_view, PARAMETER_CATALOG.size()> PARAMETER_NAMES = [] {
    std::array<std::string_view, PARAMETER_CATALOG.size()> names{};
    for (size_t i = 0; i < names.size(); ++i) names[i] = PARAMETER_CATALOG[i].name;
    return names;
}();

bool read_parameter(ParameterId id, ParameterValue& value);
ParameterWriteResult write_runtime_parameter(ParameterId id, const ParameterValue& value);
void record_invalid_command();
inline bool bootloader_reboot_pending = false;
