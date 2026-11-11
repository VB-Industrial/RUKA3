#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "encoder_calibration.hpp"
#include "calibration_eeprom.h"

namespace ruka3 {

struct MechanicalCalibrationSnapshot {
    std::uint16_t encoder_zero{};
    float lower_limit{}, upper_limit{}, soft_lower_limit{}, soft_upper_limit{};
    std::uint16_t limit_a_raw{}, limit_b_raw{}, safe_margin_ticks{};
    std::uint8_t limit_stage{};
    std::int8_t limit_direction{};
    bool zero_valid{}, limits_valid{}, loaded_from_eeprom{};
};

// Shared RUKA2 calibration owns the sole runtime zero/limit/LUT record.
class JointCalibration {
public:
    JointCalibration(std::uint8_t joint, std::uint32_t cpr, std::int8_t sign,
                     std::uint32_t reference_units, std::uint16_t safe_margin_ticks)
        : joint_(joint), cpr_(cpr), sign_(sign), reference_units_(reference_units), safe_margin_ticks_(safe_margin_ticks) {}

    void load(std::uint16_t default_zero) {
        calibration_eeprom_initialize();
        core_.initialize(joint_, sign_ < 0, 0, cpr_, reference_units_, safe_margin_ticks_);
        snapshot_.encoder_zero = default_zero;
        if (!core_.has_stored_data() && joint_ >= 5 && at24_isConnected()) {
            // Read-only migration from the commissioned RUKA3 32-byte slots.
            // These addresses and the FOC record at byte 75 are never rewritten.
            LegacyRecord a{}, b{};
            const bool va = at24_read(0, reinterpret_cast<std::uint8_t*>(&a), sizeof(a), 100) && valid(a);
            const bool vb = at24_read(32, reinterpret_cast<std::uint8_t*>(&b), sizeof(b), 100) && valid(b);
            if (va || vb) {
                const auto& record = vb && (!va || static_cast<std::int32_t>(b.sequence - a.sequence) > 0) ? b : a;
                encoder_calibration_data imported{};
                imported.point_count = 2;
                imported.zero_raw = record.encoder_zero;
                imported.zero_valid = (record.flags & 4U) || (record.version < 3 && record.encoder_zero != default_zero);
                imported.limit_a_raw = record.limit_a_raw;
                imported.limit_b_raw = record.limit_b_raw;
                imported.safe_margin_ticks = record.safe_margin_ticks;
                if (record.flags & 1U) {
                    const bool positive = record.flags & 2U;
                    const auto distance = positive ? (record.limit_b_raw + cpr_ - record.limit_a_raw) % cpr_ :
                        (record.limit_a_raw + cpr_ - record.limit_b_raw) % cpr_;
                    imported.manual_span_ticks = positive ? static_cast<std::int32_t>(distance) : -static_cast<std::int32_t>(distance);
                    // Preserve the commissioned legacy margin calculation on import only.
                    if (imported.safe_margin_ticks == 0)
                        imported.safe_margin_ticks = std::clamp<std::uint32_t>(distance / 50U, 80U, 1000U);
                }
                core_.import_legacy(imported);
                legacy_imported_ = true;
            }
        }
        refresh();
    }

    void refresh() {
        const auto& data = core_.data();
        snapshot_.zero_valid = core_.zero_valid();
        if (snapshot_.zero_valid) snapshot_.encoder_zero = data.zero_raw;
        snapshot_.limit_a_raw = data.limit_a_raw;
        snapshot_.limit_b_raw = data.limit_b_raw;
        snapshot_.safe_margin_ticks = data.safe_margin_ticks;
        snapshot_.limit_direction = data.manual_span_ticks > 0 ? 1 : data.manual_span_ticks < 0 ? -1 : 0;
        snapshot_.limit_stage = static_cast<std::uint8_t>(core_.state());
        snapshot_.loaded_from_eeprom = core_.has_stored_data();
        std::int32_t a{}, sa{}, sb{}, b{};
        snapshot_.limits_valid = core_.calibrated_limit_ticks(a, sa, sb, b);
        if (snapshot_.limits_valid) {
            const float scale = sign_ * 6.28318530718F / cpr_;
            snapshot_.lower_limit = std::min(a * scale, b * scale);
            snapshot_.upper_limit = std::max(a * scale, b * scale);
            snapshot_.soft_lower_limit = std::min(sa * scale, sb * scale);
            snapshot_.soft_upper_limit = std::max(sa * scale, sb * scale);
        }
    }

    bool set_zero(std::uint16_t raw) { const bool result = core_.calibrate_zero(raw); refresh(); return result; }
    bool set_limit(std::uint32_t stage, std::uint16_t raw) {
        if (stage == 1 && !core_.begin_manual(true, raw)) return false;
        const bool ok = stage >= 1 && stage <= 3 && core_.advance_manual(0, true, raw, stage + 1);
        refresh();
        return ok;
    }
    EncoderCalibration& core() { return core_; }
    const EncoderCalibration& core() const { return core_; }
    const MechanicalCalibrationSnapshot& snapshot() const { return snapshot_; }
    bool legacy_imported() const { return legacy_imported_; }

private:
    struct __attribute__((packed)) LegacyRecord {
        std::uint32_t magic;
        std::uint16_t version, size;
        std::uint32_t sequence;
        std::uint16_t encoder_zero, flags, limit_a_raw, limit_b_raw, safe_margin_ticks;
        std::uint8_t reserved[6];
        std::uint32_t crc;
    };
    static_assert(sizeof(LegacyRecord) == 32);
    static bool valid(const LegacyRecord& record) {
        if (record.magic != 0x4D434A52U || record.version < 1 || record.version > 3 || record.size != 32) return false;
        std::uint32_t crc = 0xFFFFFFFFU;
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(&record);
        for (unsigned i = 0; i < offsetof(LegacyRecord, crc); ++i) {
            crc ^= bytes[i];
            for (unsigned j = 0; j < 8; ++j) crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
        return record.crc == ~crc;
    }
    std::uint8_t joint_;
    std::uint32_t cpr_;
    std::int8_t sign_;
    bool legacy_imported_{};
    std::uint32_t reference_units_;
    std::uint16_t safe_margin_ticks_;
    EncoderCalibration core_;
    MechanicalCalibrationSnapshot snapshot_;
};
} // namespace ruka3
