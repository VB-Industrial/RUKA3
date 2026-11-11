#pragma once
#include "joint_parameters.hpp"
#include "calibration_service.hpp"
#include <cmath>
#include <limits>
#include <uavcan/_register/Access_1_0.h>

inline bool register_number(const uavcan_register_Value_1_0& input, double& number) {
    switch (input._tag_) {
    case 3: if (input.bit.value.count) { number = input.bit.value.bitpacked[0] & 1U; return true; } break;
    case 4: if (input.integer64.value.count) { number = input.integer64.value.elements[0]; return true; } break;
    case 5: if (input.integer32.value.count) { number = input.integer32.value.elements[0]; return true; } break;
    case 6: if (input.integer16.value.count) { number = input.integer16.value.elements[0]; return true; } break;
    case 7: if (input.integer8.value.count) { number = input.integer8.value.elements[0]; return true; } break;
    case 8: if (input.natural64.value.count) { number = input.natural64.value.elements[0]; return true; } break;
    case 9: if (input.natural32.value.count) { number = input.natural32.value.elements[0]; return true; } break;
    case 10: if (input.natural16.value.count) { number = input.natural16.value.elements[0]; return true; } break;
    case 11: if (input.natural8.value.count) { number = input.natural8.value.elements[0]; return true; } break;
    case 12: if (input.real64.value.count) { number = input.real64.value.elements[0]; return true; } break;
    case 13: if (input.real32.value.count) { number = input.real32.value.elements[0]; return true; } break;
    case 14: if (input.real16.value.count) { number = input.real16.value.elements[0]; return true; } break;
    }
    return false;
}

// One wire codec, including RUKA2's command responses and empty-read triggers.
inline void handle_parameter_register(std::size_t index, const uavcan_register_Value_1_0& input,
    uavcan_register_Value_1_0& output, uavcan_register_Access_Response_1_0& response) {
    const auto& definition = PARAMETER_CATALOG[index];
    response.persistent = definition.is_persistent;
    response._mutable = definition.is_mutable;
    ParameterValue value;
    bool immediate_integer = false;
    if (definition.id == ParameterId::ZERO_SET || definition.id == ParameterId::LUFT_CAL) {
        std::int32_t result = 0;
        if (input._tag_ == 0) {
            if (definition.id == ParameterId::ZERO_SET) {
                calibration_cancel();
                calibration_board_disable();
                result = joint_calibration_instance().set_zero(calibration_board_raw());
                if (result) calibration_board_sync();
            } else {
                (void)calibration_begin_backlash(joint_uptime_ms());
                result = static_cast<int>(joint_calibration_instance().core().state());
            }
        }
        value = result;
        immediate_integer = true;
    } else if (input._tag_ != 0) {
        double numeric{};
        bool valid = definition.is_mutable && register_number(input, numeric) && std::isfinite(numeric);
        ParameterValue requested;
        const bool boolean_command = definition.id == ParameterId::ARM || definition.id == ParameterId::BOOTLOADER ||
            definition.id == ParameterId::RESET || definition.id == ParameterId::FAULT_ACK || definition.type == ParameterType::BIT;
        if (valid && boolean_command) {
            valid = numeric == 0 || numeric == 1;
            requested = numeric == 1;
        } else if (valid && definition.type == ParameterType::REAL32) {
            valid = std::fabs(numeric) <= std::numeric_limits<float>::max();
            if (valid) requested = static_cast<float>(numeric);
        } else if (valid && definition.type == ParameterType::INTEGER32) {
            valid = std::floor(numeric) == numeric && numeric >= INT32_MIN && numeric <= INT32_MAX;
            if (valid) requested = static_cast<std::int32_t>(numeric);
        } else valid = false;
        const bool accepted = valid && write_runtime_parameter(definition.id, requested) == ParameterWriteResult::OK;
        if (!accepted) record_invalid_command();
        if (definition.id == ParameterId::MAN_CAL || definition.id == ParameterId::MOVE_CAL) {
            value = accepted ? std::get<std::int32_t>(requested) + (definition.id == ParameterId::MAN_CAL ? 1 : 0) : 0;
            immediate_integer = true;
        }
    }
    if (!immediate_integer && !read_parameter(definition.id, value)) {
        output = {}; return; // Explicit unavailable register value, never a fabricated zero.
    }
    switch (definition.type) {
    case ParameterType::REAL32:
        output._tag_ = 13; output.real32.value.count = 1;
        output.real32.value.elements[0] = std::get<float>(value); break;
    case ParameterType::REAL32_ARRAY: {
        output._tag_ = 13; output.real32.value.count = definition.count;
        const auto& data = std::get<AngleDiagnostics>(value);
        for (unsigned i = 0; i < definition.count; ++i) output.real32.value.elements[i] = data[i];
        break;
    }
    case ParameterType::INTEGER32:
        output._tag_ = 5; output.integer32.value.count = 1;
        output.integer32.value.elements[0] = std::get<std::int32_t>(value); break;
    case ParameterType::INTEGER32_ARRAY: {
        output._tag_ = 5; output.integer32.value.count = definition.count;
        const auto& data = std::get<IntegerDiagnostics>(value);
        for (unsigned i = 0; i < definition.count; ++i) output.integer32.value.elements[i] = data[i];
        break;
    }
    case ParameterType::NATURAL32:
        output._tag_ = 9; output.natural32.value.count = 1;
        output.natural32.value.elements[0] = std::get<std::uint32_t>(value); break;
    case ParameterType::BIT:
        output._tag_ = 3; output.bit.value.count = 1;
        output.bit.value.bitpacked[0] = std::get<bool>(value); break;
    case ParameterType::STRING: {
        const auto text = std::get<std::string_view>(value);
        output._tag_ = 1; output._string.value.count = text.size();
        for (unsigned i = 0; i < text.size(); ++i) output._string.value.elements[i] = text[i];
        break;
    }
    }
}
