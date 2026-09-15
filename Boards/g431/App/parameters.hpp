#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <variant>

enum class ParameterType : uint8_t { REAL32, NATURAL32, INTEGER32, BIT, STRING };

enum class ParameterId : uint8_t {
    NODE_ID, IS_ON, BOOTLOADER, CMD_ERRORS, MODEL, REVISION,
    BUS_VOLTAGE, BUS_CURRENT, TEMP_MCU, TEMP_STATOR, IS_FAULT,
    ENCODER_SHAFT, ENCODER_ROTOR, CALIB_VALID, CALIB_OFFSET, CALIB_INVERTED,
    ELECTRIC_ANGLE,
    CAN_ERR_TOTAL, CAN_RX_ERR, CAN_TX_ERR, CAN_ERR_LOG, CAN_ARB_LEC,
    CAN_DATA_LEC, CAN_WARNING, CAN_PASSIVE, CAN_BUS_OFF,
    CMD_COUNT, CMD_AGE_MS, CMD_TIMEOUT,
    FAULT_ACTIVE, FAULT_LATCHED, FAULT_LEVEL, STOP_REASON, NET_STATE, FAULT_ACK,
    JOINT_STATE,
    OUT_ANGLE, OUT_VELOCITY, OUT_RESIDUAL,
};

struct ParameterDefinition {
    ParameterId id;
    std::string_view name;
    ParameterType type;
    bool is_mutable;
    bool is_persistent;
};

using ParameterValue = std::variant<uint32_t, int32_t, float, bool, std::string_view>;
enum class ParameterWriteResult : uint8_t { OK, READ_ONLY, INVALID, UNAVAILABLE };

inline constexpr std::array<ParameterDefinition, 39> PARAMETER_CATALOG{{
    {ParameterId::NODE_ID,         "node_id",       ParameterType::NATURAL32, false, false},
    {ParameterId::IS_ON,           "is_on",         ParameterType::BIT,       true,  false},
    {ParameterId::BOOTLOADER,      "bootloader",    ParameterType::BIT,       true,  false},
    {ParameterId::CMD_ERRORS,      "cmd_errors",    ParameterType::NATURAL32, false, false},
    {ParameterId::MODEL,           "vbdrive_model", ParameterType::STRING,    false, false},
    {ParameterId::REVISION,        "firmware_rev",  ParameterType::STRING,    false, false},
    {ParameterId::BUS_VOLTAGE,     "bus_voltage",   ParameterType::REAL32,    false, false},
    {ParameterId::BUS_CURRENT,     "bus_current",   ParameterType::REAL32,    false, false},
    {ParameterId::TEMP_MCU,        "temp_mcu",      ParameterType::REAL32,    false, false},
    {ParameterId::TEMP_STATOR,     "temp_stator",   ParameterType::REAL32,    false, false},
    {ParameterId::IS_FAULT,        "is_fault",      ParameterType::BIT,       false, false},
    {ParameterId::ENCODER_SHAFT,   "encoder_shaft", ParameterType::NATURAL32, false, false},
    {ParameterId::ENCODER_ROTOR,   "encoder_rotor", ParameterType::NATURAL32, false, false},
    {ParameterId::CALIB_VALID,     "calib_valid",   ParameterType::BIT,       false, false},
    {ParameterId::CALIB_OFFSET,    "calib_offset",  ParameterType::INTEGER32, false, false},
    {ParameterId::CALIB_INVERTED,  "calib_inverted",ParameterType::BIT,       false, false},
    {ParameterId::ELECTRIC_ANGLE,  "electric_angle",ParameterType::REAL32,    false, false},
    {ParameterId::CAN_ERR_TOTAL,   "can_err_total", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_RX_ERR,      "can_rx_err",    ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_TX_ERR,      "can_tx_err",    ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_ERR_LOG,     "can_err_log",   ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_ARB_LEC,     "can_arb_lec",   ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_DATA_LEC,    "can_data_lec",  ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_WARNING,     "can_warning",   ParameterType::BIT,       false, false},
    {ParameterId::CAN_PASSIVE,     "can_passive",   ParameterType::BIT,       false, false},
    {ParameterId::CAN_BUS_OFF,     "can_bus_off",   ParameterType::BIT,       false, false},
    {ParameterId::CMD_COUNT,       "cmd_count",     ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_AGE_MS,      "cmd_age_ms",    ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_TIMEOUT,     "cmd_timeout",   ParameterType::BIT,       false, false},
    {ParameterId::FAULT_ACTIVE,    "fault_active",  ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_LATCHED,   "fault_latched", ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_LEVEL,     "fault_level",   ParameterType::NATURAL32, false, false},
    {ParameterId::STOP_REASON,     "stop_reason",   ParameterType::NATURAL32, false, false},
    {ParameterId::NET_STATE,       "net_state",     ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_ACK,       "fault_ack",     ParameterType::BIT,       true,  false},
    {ParameterId::JOINT_STATE,     "joint_state",   ParameterType::NATURAL32, false, false},
    {ParameterId::OUT_ANGLE,       "out_angle",     ParameterType::REAL32,    false, false},
    {ParameterId::OUT_VELOCITY,    "out_velocity",  ParameterType::REAL32,    false, false},
    {ParameterId::OUT_RESIDUAL,    "out_residual",  ParameterType::REAL32,    false, false},
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
