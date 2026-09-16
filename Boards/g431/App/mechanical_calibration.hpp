#pragma once

#include <cstdint>
#include <cstring>

#include <voltbro/eeprom/eeprom.hpp>

#include "state_manager.hpp"

namespace ruka3 {

struct MechanicalCalibrationSnapshot {
    std::uint16_t encoder_zero{};
    float lower_limit{};
    float upper_limit{};
    std::uint16_t limit_a_raw{};
    std::uint16_t limit_b_raw{};
    std::uint8_t limit_stage{};
    std::int8_t limit_direction{};
    bool limits_valid{};
    bool loaded_from_eeprom{};
};

class MechanicalCalibration {
public:
    MechanicalCalibration(EEPROM& eeprom, std::uint32_t encoder_cpr, std::int8_t encoder_sign)
        : eeprom_(eeprom), cpr_(encoder_cpr), sign_(encoder_sign) {}

    void load(std::uint16_t default_zero, float default_lower, float default_upper) {
        Record first{};
        Record second{};
        const bool first_valid = eeprom_.read(&first, MECHANICAL_CALIBRATION_SLOT_A) == HAL_OK && valid(first);
        const bool second_valid = eeprom_.read(&second, MECHANICAL_CALIBRATION_SLOT_B) == HAL_OK && valid(second);
        if (!first_valid && !second_valid) {
            snapshot_.encoder_zero = default_zero;
            snapshot_.lower_limit = default_lower;
            snapshot_.upper_limit = default_upper;
            snapshot_.limits_valid = default_lower < default_upper;
            return;
        }
        const Record& record = (!first_valid || (second_valid && newer(second.sequence, first.sequence))) ? second : first;
        sequence_ = record.sequence;
        active_slot_ = (&record == &second) ? MECHANICAL_CALIBRATION_SLOT_B : MECHANICAL_CALIBRATION_SLOT_A;
        snapshot_.encoder_zero = record.encoder_zero;
        snapshot_.limit_a_raw = record.limit_a_raw;
        snapshot_.limit_b_raw = record.limit_b_raw;
        has_raw_limits_ = (record.flags & kLimitsValid) != 0U && record.limit_a_raw != record.limit_b_raw;
        snapshot_.limits_valid = has_raw_limits_;
        snapshot_.limit_direction = has_raw_limits_
            ? ((record.flags & kPositiveDirection) != 0U ? 1 : -1) : 0;
        snapshot_.loaded_from_eeprom = true;
        if (has_raw_limits_) refresh_limits();
        else {
            snapshot_.lower_limit = default_lower;
            snapshot_.upper_limit = default_upper;
        }
    }

    bool set_zero(std::uint16_t raw_ticks) {
        snapshot_.encoder_zero = raw_ticks;
        if (has_raw_limits_) refresh_limits();
        return save();
    }

    bool set_limit(std::uint32_t stage, std::uint16_t raw_ticks) {
        if (stage == 1U) {
            first_limit_ = raw_ticks;
            snapshot_.limit_stage = 1U;
            return true;
        }
        if (stage == 2U && snapshot_.limit_stage == 1U && raw_ticks != first_limit_) {
            middle_point_ = raw_ticks;
            snapshot_.limit_stage = 2U;
            return true;
        }
        if (stage != 3U || snapshot_.limit_stage != 2U || raw_ticks == first_limit_) {
            snapshot_.limit_stage = 0U;
            return false;
        }

        const std::uint32_t forward_span = distance(first_limit_, raw_ticks, 1);
        const std::uint32_t forward_middle = distance(first_limit_, middle_point_, 1);
        if (forward_middle > 0U && forward_middle < forward_span) snapshot_.limit_direction = 1;
        else {
            const std::uint32_t reverse_span = distance(first_limit_, raw_ticks, -1);
            const std::uint32_t reverse_middle = distance(first_limit_, middle_point_, -1);
            if (reverse_middle == 0U || reverse_middle >= reverse_span) {
                snapshot_.limit_stage = 0U;
                return false;
            }
            snapshot_.limit_direction = -1;
        }
        snapshot_.limit_a_raw = first_limit_;
        snapshot_.limit_b_raw = raw_ticks;
        has_raw_limits_ = true;
        snapshot_.limits_valid = true;
        refresh_limits();
        snapshot_.limit_stage = 0U;
        return save();
    }

    const MechanicalCalibrationSnapshot& snapshot() const { return snapshot_; }

private:
    static constexpr std::uint32_t kMagic = 0x4D434A52U;  // RJCM
    static constexpr std::uint16_t kVersion = 2U;
    static constexpr std::uint16_t kLimitsValid = 1U;
    static constexpr std::uint16_t kPositiveDirection = 2U;
    struct __attribute__((packed)) Record {
        std::uint32_t magic{};
        std::uint16_t version{};
        std::uint16_t size{};
        std::uint32_t sequence{};
        std::uint16_t encoder_zero{};
        std::uint16_t flags{};
        std::uint16_t limit_a_raw{};
        std::uint16_t limit_b_raw{};
        std::uint8_t reserved[8]{};
        std::uint32_t crc{};
    };
    static_assert(sizeof(Record) == MECHANICAL_CALIBRATION_SLOT_SIZE);

    static std::uint32_t crc32(const std::uint8_t* bytes, std::size_t size) {
        std::uint32_t result = 0xFFFFFFFFU;
        for (std::size_t i = 0; i < size; ++i) {
            result ^= bytes[i];
            for (std::uint8_t bit = 0; bit < 8U; ++bit)
                result = (result >> 1U) ^ ((result & 1U) != 0U ? 0xEDB88320U : 0U);
        }
        return ~result;
    }
    static bool newer(std::uint32_t candidate, std::uint32_t current) {
        return static_cast<std::int32_t>(candidate - current) > 0;
    }
    static bool valid(const Record& record) {
        return record.magic == kMagic && (record.version == 1U || record.version == kVersion) && record.size == sizeof(Record) &&
               record.crc == crc32(reinterpret_cast<const std::uint8_t*>(&record), offsetof(Record, crc));
    }
    std::uint32_t distance(std::uint16_t from, std::uint16_t to, std::int8_t direction) const {
        return direction > 0 ? (static_cast<std::uint32_t>(to) + cpr_ - from) % cpr_
                             : (static_cast<std::uint32_t>(from) + cpr_ - to) % cpr_;
    }
    float ticks_to_angle(std::uint16_t raw) const {
        std::int32_t delta = static_cast<std::int32_t>(raw) - static_cast<std::int32_t>(snapshot_.encoder_zero);
        while (delta > static_cast<std::int32_t>(cpr_ / 2U)) delta -= static_cast<std::int32_t>(cpr_);
        while (delta < -static_cast<std::int32_t>(cpr_ / 2U)) delta += static_cast<std::int32_t>(cpr_);
        return static_cast<float>(sign_) * 6.28318530718F * static_cast<float>(delta) / static_cast<float>(cpr_);
    }
    void refresh_limits() {
        const float first = ticks_to_angle(snapshot_.limit_a_raw);
        const float second = ticks_to_angle(snapshot_.limit_b_raw);
        snapshot_.lower_limit = first < second ? first : second;
        snapshot_.upper_limit = first < second ? second : first;
    }
    bool save() {
        Record record{};
        record.magic = kMagic;
        record.version = kVersion;
        record.size = sizeof(record);
        record.sequence = ++sequence_;
        record.encoder_zero = snapshot_.encoder_zero;
        record.flags = has_raw_limits_ ? kLimitsValid : 0U;
        if (snapshot_.limit_direction > 0) record.flags |= kPositiveDirection;
        record.limit_a_raw = snapshot_.limit_a_raw;
        record.limit_b_raw = snapshot_.limit_b_raw;
        record.crc = crc32(reinterpret_cast<const std::uint8_t*>(&record), offsetof(Record, crc));
        const std::uint16_t destination = active_slot_ == MECHANICAL_CALIBRATION_SLOT_A
            ? MECHANICAL_CALIBRATION_SLOT_B : MECHANICAL_CALIBRATION_SLOT_A;
        if (eeprom_.write(&record, destination) != HAL_OK) return false;
        Record verify{};
        if (eeprom_.read(&verify, destination) != HAL_OK || !valid(verify)) return false;
        active_slot_ = destination;
        snapshot_.loaded_from_eeprom = true;
        return true;
    }
    EEPROM& eeprom_;
    std::uint32_t cpr_;
    std::int8_t sign_;
    MechanicalCalibrationSnapshot snapshot_{};
    std::uint32_t sequence_{};
    std::uint16_t active_slot_{MECHANICAL_CALIBRATION_SLOT_B};
    std::uint16_t first_limit_{};
    std::uint16_t middle_point_{};
    bool has_raw_limits_{};
};

}  // namespace ruka3
