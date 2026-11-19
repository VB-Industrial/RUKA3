#pragma once
#include <array>
#include <cstdint>
#include <string_view>
#include <variant>

// Same millisecond clock as the board service loop.
std::uint32_t joint_uptime_ms();

enum class ParameterType : std::uint8_t { REAL32, REAL32_ARRAY, NATURAL32, INTEGER32, INTEGER32_ARRAY, BIT, STRING };
enum class ParameterId : std::uint8_t {
    NODE_ID, ARM, BOOTLOADER, RESET, CMD_ERRORS, MODEL, REVISION,
    BUS_VOLTAGE, BUS_CURRENT, TEMP_MCU, TEMP_STATOR, IS_FAULT, GET_ANGLE,
    CALIB_VALID, CALIB_INVERTED, MOTOR_CALIB, DRIVER_WAKE, ZERO_SET, SET_LIMIT, MOVE, POS_SET,
    CAN_ERR_TOTAL, CAN_RX_ERR, CAN_TX_ERR, CAN_ERR_LOG, CAN_ARB_LEC, CAN_DATA_LEC,
    CAN_WARNING, CAN_PASSIVE, CAN_BUS_OFF, CMD_COUNT, CMD_AGE_MS, CMD_TIMEOUT,
    FAULT_ACTIVE, FAULT_LATCHED, FAULT_LEVEL, STOP_REASON, NET_STATE, FAULT_ACK, JOINT_STATE,
    MAN_CAL, MOVE_CAL, CAL_DATA, CAL_DIAG, LUFT_CAL, VERSION, LIMITS, CONTROL_DIAG,
    DRIVER_DIAG, BOOT_DIAG, CYPHAL_DIAG, ENCODER_DIAG, EEPROM_DIAG, FAULT_HISTORY,
    MOTOR_POS, CURRENT, CURRENT_DIAG, EL_OFFSET, ENCODER_CPR, POSITION_KP, POSITION_KD, ACTIVE_GAINS, POSITION_KI, INTEGRAL_LIMIT, INTEGRAL_CURRENT,
#ifdef RUKA3_FOC_TRACE
    FOC_TRACE_COMMAND, FOC_TRACE_INFO, FOC_TRACE_PAGE, FOC_TRACE_DATA, FOC_TRACE_D_PULSE,
#endif
    PROFILE_VELOCITY, PROFILE_ACCELERATION,
};

struct ParameterDefinition {
    ParameterId id;
    std::string_view name;
    ParameterType type;
    bool is_mutable;
    bool is_persistent;
    unsigned count = 1;
};
using AngleDiagnostics = std::array<float, 36>;
using IntegerDiagnostics = std::array<std::int32_t, 32>;
using ParameterValue = std::variant<std::uint32_t, std::int32_t, float, bool, std::string_view,
                                    AngleDiagnostics, IntegerDiagnostics>;
enum class ParameterWriteResult : std::uint8_t { OK, READ_ONLY, INVALID, UNAVAILABLE };

// One List/Access contract for both targets. RUKA2 singleton names are preserved;
// paired position/current accessors use set_/get_.
// TMC-specific hardware is represented by BLDC motor/driver diagnostics.
inline constexpr auto PARAMETER_CATALOG = std::to_array<ParameterDefinition>({
    {ParameterId::MAN_CAL, "man_cal", ParameterType::INTEGER32, true, false},
    {ParameterId::MOVE_CAL, "move_cal", ParameterType::INTEGER32, true, false},
    {ParameterId::CAL_DATA, "cal_data", ParameterType::INTEGER32_ARRAY, false, false, 8},
    {ParameterId::CAL_DIAG, "cal_diag", ParameterType::INTEGER32_ARRAY, false, false, 11},
    {ParameterId::POS_SET, "set_pos", ParameterType::REAL32, true, false},
    {ParameterId::GET_ANGLE, "get_pos", ParameterType::REAL32_ARRAY, false, false, 36},
    {ParameterId::FAULT_ACK, "fail_ack", ParameterType::INTEGER32, true, false},
    {ParameterId::IS_FAULT, "errors", ParameterType::INTEGER32_ARRAY, false, false, 26},
    {ParameterId::MOVE, "move", ParameterType::REAL32, true, false},
    {ParameterId::MOTOR_POS, "tmc_pos_set", ParameterType::INTEGER32, true, false},
    {ParameterId::ZERO_SET, "zero_set", ParameterType::INTEGER32, false, false},
    {ParameterId::LUFT_CAL, "luft_cal", ParameterType::INTEGER32, false, false},
    {ParameterId::ARM, "arm", ParameterType::INTEGER32_ARRAY, true, false, 5},
    {ParameterId::VERSION, "version", ParameterType::INTEGER32_ARRAY, false, false, 4},
    {ParameterId::LIMITS, "limits", ParameterType::REAL32_ARRAY, false, false, 9},
    {ParameterId::CONTROL_DIAG, "control_diag", ParameterType::REAL32_ARRAY, false, false, 8},
    {ParameterId::DRIVER_DIAG, "tmc_diag", ParameterType::INTEGER32_ARRAY, false, false, 11},
    {ParameterId::BOOT_DIAG, "boot_diag", ParameterType::INTEGER32_ARRAY, false, false, 5},
    {ParameterId::CYPHAL_DIAG, "cyphal_diag", ParameterType::INTEGER32_ARRAY, false, false, 6},
    {ParameterId::ENCODER_DIAG, "encoder_diag", ParameterType::REAL32_ARRAY, false, false, 12},
    {ParameterId::EEPROM_DIAG, "eeprom_diag", ParameterType::INTEGER32_ARRAY, false, false, 6},
    {ParameterId::FAULT_HISTORY, "fault_history", ParameterType::INTEGER32_ARRAY, false, false, 6},
    {ParameterId::BOOTLOADER, "bootloader", ParameterType::INTEGER32, true, false},
    {ParameterId::RESET, "reset", ParameterType::INTEGER32, true, false},
    {ParameterId::CURRENT, "set_current", ParameterType::REAL32, true, false},
    {ParameterId::CURRENT_DIAG, "get_current", ParameterType::REAL32_ARRAY, false, false, 25},
    {ParameterId::NODE_ID, "node_id", ParameterType::NATURAL32, false, false},
    {ParameterId::ENCODER_CPR, "enc_cpr", ParameterType::NATURAL32, false, false},
    {ParameterId::MODEL, "model", ParameterType::STRING, false, false},
    {ParameterId::REVISION, "fw_rev", ParameterType::STRING, false, false},
    {ParameterId::BUS_VOLTAGE, "bus_voltage", ParameterType::REAL32, false, false},
    {ParameterId::BUS_CURRENT, "bus_current", ParameterType::REAL32, false, false},
    {ParameterId::TEMP_MCU, "temp_mcu", ParameterType::REAL32, false, false},
    {ParameterId::TEMP_STATOR, "temp_stator", ParameterType::REAL32, false, false},
    {ParameterId::CALIB_VALID, "calib_valid", ParameterType::BIT, false, false},
    {ParameterId::CALIB_INVERTED, "calib_inv", ParameterType::BIT, false, false},
    {ParameterId::MOTOR_CALIB, "motor_calib", ParameterType::BIT, true, false},
    {ParameterId::DRIVER_WAKE, "drv_wake", ParameterType::BIT, true, false},
    {ParameterId::CAN_ERR_TOTAL, "can_err", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_RX_ERR, "can_rx_err", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_TX_ERR, "can_tx_err", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_ERR_LOG, "can_err_log", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_ARB_LEC, "can_arb_lec", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_DATA_LEC, "can_dat_lec", ParameterType::NATURAL32, false, false},
    {ParameterId::CAN_WARNING, "can_warning", ParameterType::BIT, false, false},
    {ParameterId::CAN_PASSIVE, "can_passive", ParameterType::BIT, false, false},
    {ParameterId::CAN_BUS_OFF, "can_bus_off", ParameterType::BIT, false, false},
    {ParameterId::CMD_ERRORS, "cmd_errors", ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_COUNT, "cmd_count", ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_AGE_MS, "cmd_age_ms", ParameterType::NATURAL32, false, false},
    {ParameterId::CMD_TIMEOUT, "cmd_timeout", ParameterType::BIT, false, false},
    {ParameterId::FAULT_ACTIVE, "fault_act", ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_LATCHED, "fault_latch", ParameterType::NATURAL32, false, false},
    {ParameterId::FAULT_LEVEL, "fault_level", ParameterType::NATURAL32, false, false},
    {ParameterId::STOP_REASON, "stop_reason", ParameterType::NATURAL32, false, false},
    {ParameterId::NET_STATE, "net_state", ParameterType::NATURAL32, false, false},
    {ParameterId::JOINT_STATE, "joint_state", ParameterType::NATURAL32, false, false},
    {ParameterId::POSITION_KP, "kp", ParameterType::REAL32, true, false},
    {ParameterId::POSITION_KD, "kd", ParameterType::REAL32, true, false},
    {ParameterId::ACTIVE_GAINS, "active_gains", ParameterType::REAL32_ARRAY, false, false, 2},
    {ParameterId::POSITION_KI, "ki", ParameterType::REAL32, true, false},
    {ParameterId::INTEGRAL_LIMIT, "i_limit", ParameterType::REAL32, true, false},
    {ParameterId::INTEGRAL_CURRENT, "i_current", ParameterType::REAL32, false, false},
    {ParameterId::PROFILE_VELOCITY, "pos_v", ParameterType::REAL32, true, false},
    {ParameterId::PROFILE_ACCELERATION, "pos_a", ParameterType::REAL32, true, false},
#ifdef RUKA3_FOC_TRACE
    {ParameterId::FOC_TRACE_D_PULSE, "trace_d_pulse", ParameterType::REAL32, true, false},
    {ParameterId::FOC_TRACE_COMMAND, "foc_trace", ParameterType::INTEGER32, true, false},
    {ParameterId::FOC_TRACE_INFO, "trace_info", ParameterType::INTEGER32_ARRAY, false, false, 32},
    {ParameterId::FOC_TRACE_PAGE, "trace_page", ParameterType::INTEGER32, true, false},
    {ParameterId::FOC_TRACE_DATA, "trace_data", ParameterType::INTEGER32_ARRAY, false, false, 32},
#endif
});
consteval bool parameter_catalog_is_valid() {
    for (unsigned i = 0; i < PARAMETER_CATALOG.size(); ++i) {
        if (PARAMETER_CATALOG[i].name.empty() || PARAMETER_CATALOG[i].name.size() > 15) return false;
        for (unsigned j = i + 1; j < PARAMETER_CATALOG.size(); ++j)
            if (PARAMETER_CATALOG[i].name == PARAMETER_CATALOG[j].name || PARAMETER_CATALOG[i].id == PARAMETER_CATALOG[j].id) return false;
    }
    return true;
}
static_assert(parameter_catalog_is_valid());
inline constexpr auto PARAMETER_NAMES = [] {
    std::array<std::string_view, PARAMETER_CATALOG.size()> names{};
    for (unsigned i = 0; i < names.size(); ++i) names[i] = PARAMETER_CATALOG[i].name;
    return names;
}();

bool read_board_parameter(ParameterId, ParameterValue&);
ParameterWriteResult write_board_parameter(ParameterId, const ParameterValue&);
bool read_parameter(ParameterId, ParameterValue&);
ParameterWriteResult write_runtime_parameter(ParameterId, const ParameterValue&);
void update_register_diagnostics(std::uint32_t now);
void record_invalid_command();
inline bool bootloader_reboot_pending = false;
inline bool reset_pending = false;
inline std::uint32_t reset_requested_at = 0;
