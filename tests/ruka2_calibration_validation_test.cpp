// Host regression test for validation telemetry (no MCU or EEPROM needed).
// Build with g++ -std=c++17 -ICore/Inc using this file and
// Core/Src/encoder_calibration.cpp; run the resulting executable.
#include <array>
#include <cstdint>
#include <cassert>
#include <iostream>
#define private public
#include "encoder_calibration.hpp"
#undef private

extern "C" bool encoder_calibration_storage_load(uint8_t, encoder_calibration_data*) { return false; }
static bool save_ok=true;
extern "C" bool encoder_calibration_storage_save(uint8_t, encoder_calibration_data* data) {
    ++data->sequence; return save_ok;
}

static EncoderCalibration linear(int sign = 1)
{
    EncoderCalibration cal;
    cal.data_.point_count = 3;
    cal.safe_end_ticks_ = sign * 1000;
    cal.tmc_samples_ = {0, sign * 10000, sign * 20000};
    cal.reverse_tmc_samples_ = cal.tmc_samples_;
    cal.tmc_sample_valid_.fill(true);
    cal.reverse_tmc_sample_valid_.fill(true);
    return cal;
}

int main()
{
    // Both deployed encoder resolutions and signs must share the same geometry.
    for (int cpr : {16384, 65536}) for (int sign : {1, -1}) {
        EncoderCalibration scaled;
        scaled.initialize(4, sign < 0, 0, cpr);
        assert(scaled.kEncoderTicksPerTurn == cpr);
        assert(scaled.kHalfTurnTicks == cpr / 2);
        const int scale = cpr / 16384;
        encoder_calibration_data stored{};
        stored.point_count = 3;
        stored.limit_a_raw = sign > 0 ? cpr - 500 * scale : 500 * scale;
        stored.manual_span_ticks = sign * 1000 * scale;
        stored.limit_b_raw = (stored.limit_a_raw + sign * 1000 * scale + cpr) % cpr;
        stored.zero_valid = 1;
        stored.zero_raw = 0;
        stored.correction_ticks[1] = sign * 100 * scale;
        scaled.import_legacy(stored);
        int32_t angle{};
        assert(scaled.calibrated_position_ticks(0, angle) && angle == 0);
        const auto raw = (stored.limit_a_raw + sign * 750 * scale + cpr) % cpr;
        assert(scaled.calibrated_position_ticks(raw, angle));
        assert(angle == sign * 200 * scale);
        assert(scaled.calibrate_zero(raw));
        assert(scaled.calibrated_position_ticks(raw, angle) && angle == 0);
        assert(scaled.data().correction_ticks[1] == sign * 100 * scale);
        const auto saved_sequence=scaled.data().sequence;
        save_ok=false;
        assert(!scaled.calibrate_zero((raw+10)%cpr));
        assert(scaled.data().zero_raw==raw && scaled.data().zero_valid);
        assert(scaled.data().sequence==saved_sequence);
        assert(scaled.data().correction_ticks[1]==sign*100*scale);
        assert(scaled.calibrated_position_ticks(raw, angle) && angle==0);
        save_ok=true;
        assert(scaled.begin_manual(true, raw));
        scaled.abort();
        auto stopped = scaled.update(1, true, raw, 0);
        assert(stopped.disable_driver);
    }
    // A partial raw span cannot define BLDC angle scale: gear_ratio already does.
    // A 1000-tick raw span here represents 800 physical encoder ticks.
    for (int cpr : {16384, 65536}) for (int sign : {1, -1}) {
        EncoderCalibration fixed;
        fixed.initialize(4,sign<0,0,cpr,65536);
        fixed.data_.point_count=3;
        fixed.data_.manual_span_ticks=sign*1000;
        fixed.data_.zero_valid=1;
        fixed.data_.zero_raw=0;
        fixed.safe_end_ticks_=sign*1000;
        const int reference_span=800*65536/cpr;
        fixed.tmc_samples_={0,reference_span/2,reference_span};
        fixed.reverse_tmc_samples_=fixed.tmc_samples_;
        fixed.tmc_sample_valid_.fill(true);
        fixed.reverse_tmc_sample_valid_.fill(true);
        assert(fixed.process_table());
        assert(fixed.data_.correction_ticks[2]==-sign*100);
        assert(fixed.corrected_position_ticks(sign*1000)-fixed.corrected_position_ticks(0)==sign*800);
        assert(fixed.corrected_position_ticks(sign*500)-fixed.corrected_position_ticks(0)==sign*400);
        for (int i=0; i<3; ++i) fixed.data_.correction_ticks[i] += 1000;
        assert(fixed.corrected_position_ticks(sign*1000)-fixed.corrected_position_ticks(0)==sign*800);
        assert(fixed.corrected_position_ticks(sign*500)-fixed.corrected_position_ticks(0)==sign*400);
    }
    // Reproduce entering the safe B endpoint from outside, overshooting it
    // inward, and continuing toward A without crossing B again. Include the
    // encoder's raw wrap and both signs of the physical calibration span.
    for (int sign : {1, -1}) {
        EncoderCalibration sweep;
        const int base = 16000;
        const auto raw = [=](int ticks) {
            return static_cast<uint16_t>((base + sign * ticks) & 16383);
        };
        sweep.data_.point_count = 3;
        sweep.data_.manual_span_ticks = sign * 1040;
        sweep.safe_end_ticks_ = sign * 1000;
        sweep.position_ticks_ = sign * 1010;
        sweep.previous_raw_ = raw(1010);
        sweep.state_ = EncoderCalibrationState::MoveToBStart;
        sweep.manual_reverse_first_ = true;
        auto action = sweep.update(10, true, raw(990), 12345);
        assert(action.zero_tmc_position);
        assert(sweep.state() == EncoderCalibrationState::ReverseSweepToA);
        assert(sweep.next_point_ == 1);
        sweep.update(20, true, raw(980), -sign * 200);
        sweep.update(30, true, raw(480), -sign * 10400);
        sweep.update(40, true, raw(-10), -sign * 20400);
        assert(sweep.state() == EncoderCalibrationState::ManualSettleAtA);
        for (int i = 0; i < 3; ++i) {
            assert(sweep.tmc_sample_valid_[i]);
            assert(sweep.reverse_tmc_sample_valid_[i]);
        }
        assert(sweep.process_table());
        assert(sweep.validation_diagnostics()[0] == 0);
        assert(std::abs(sweep.data_.tmc_span_steps - sign * 20200) < 100);
    }
    // Delayed sampling crosses multiple table nodes in one frame. Each crossing
    // gets its interpolated reference, rather than duplicate end-of-frame values.
    for (int sign : {1, -1}) {
        EncoderCalibration sparse;
        sparse.data_.point_count = 4;
        sparse.data_.manual_span_ticks = sign * 1000;
        sparse.safe_end_ticks_ = sign * 1000;
        sparse.previous_raw_ = static_cast<uint16_t>((8000 + sign * 900) & 16383);
        sparse.position_ticks_ = sign * 900;
        sparse.last_tmc_position_steps_ = sign * 9000;
        sparse.reference_initialized_ = true;
        sparse.state_ = EncoderCalibrationState::ReverseSweepToA;
        sparse.manual_reverse_first_ = true;
        sparse.next_point_ = 2;
        sparse.reverse_tmc_samples_[3] = sign * 10000;
        sparse.reverse_tmc_sample_valid_[3] = true;
        sparse.update(1, true, static_cast<uint16_t>((8000 + sign * 200) & 16383), sign * 2000);
        assert(sparse.reverse_tmc_samples_[2] == sign * 6660);
        assert(sparse.reverse_tmc_samples_[1] == sign * 3330);
        sparse.update(2, true, 8000, 0);
        assert(sparse.process_table());
    }
    for (int sign : {1, -1}) {
        auto cal = linear(sign);
        assert(cal.process_table());
        assert(cal.validation_diagnostics()[0] == 0);
        assert(cal.data_.tmc_span_steps == sign * 20000);
        for (int i = 0; i < 3; ++i) assert(cal.data_.correction_ticks[i] == 0);
    }
    auto cal = linear();
    cal.reverse_tmc_samples_[2] = 18000;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 4);
    assert(cal.validation_diagnostics()[2] == 2000);
    assert(cal.validation_diagnostics()[4] == 1000);
    cal.state_ = EncoderCalibrationState::Processing;
    cal.fail(EncoderCalibrationError::InvalidTable);
    assert(cal.failure_state() == 7 && cal.validation_diagnostics()[0] == 4);
    assert(cal.begin_manual(true, 100));
    assert(cal.validation_diagnostics()[0] == 0);

    cal = linear();
    cal.tmc_samples_[1] = 0;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 6 && cal.validation_diagnostics()[1] == 1);
    cal = linear();
    cal.reverse_tmc_samples_[1] = 0;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 7);
    cal = linear();
    cal.reverse_tmc_samples_ = {0, -10000, -20000};
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 3);
    cal = linear();
    cal.safe_end_ticks_ = 10000;
    cal.tmc_samples_[1] = cal.reverse_tmc_samples_[1] = 1000;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 5);
    cal = linear();
    cal.data_.point_count = 257;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 1);
    cal = linear();
    cal.tmc_sample_valid_[1] = false;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 15 && cal.validation_diagnostics()[1] == 1);
    cal = linear();
    cal.reverse_tmc_sample_valid_[1] = false;
    assert(!cal.process_table());
    assert(cal.validation_diagnostics()[0] == 16 && cal.validation_diagnostics()[1] == 1);
    cal = linear();
    assert(!cal.finalize_backlash());
    assert(cal.validation_diagnostics()[0] == 13);
    cal.backlash_sample_count_ = 4;
    assert(cal.finalize_backlash());
    assert(cal.data_.backlash_steps == 0);
    cal.backlash_samples_[2] = 20;
    cal.backlash_samples_[3] = 40;
    assert(cal.finalize_backlash());
    assert(cal.data_.backlash_steps == 30);
    std::cout << "Calibration validation diagnostics: PASS\n";
}
