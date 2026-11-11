#include <cstdint>
std::uint32_t joint_uptime_ms() { return 12345; }
#include "joint_register_codec.hpp"
#include <cassert>
#include <limits>
static ruka3::JointCalibration cal(4,16384,1,65536,208);
static int writes, invalid, syncs;
static bool save_ok=true;
static std::uint16_t raw=0;
static std::uint32_t backlash_started;
static ParameterValue last;
ruka3::JointCalibration& joint_calibration_instance() { return cal; }
void calibration_cancel() {}
void calibration_board_disable() {}
void calibration_board_sync() { ++syncs; }
std::uint16_t calibration_board_raw() { return raw; }
bool calibration_begin_backlash(std::uint32_t now) { backlash_started=now; return false; }
extern "C" bool encoder_calibration_storage_load(uint8_t,encoder_calibration_data*) { return false; }
extern "C" bool encoder_calibration_storage_save(uint8_t,encoder_calibration_data*) { return save_ok; }
void record_invalid_command() { ++invalid; }
ParameterWriteResult write_runtime_parameter(ParameterId,const ParameterValue& v) { ++writes; last=v; return ParameterWriteResult::OK; }
bool read_parameter(ParameterId id,ParameterValue& v) {
    if(id==ParameterId::ARM) { IntegerDiagnostics a{}; a[0]=std::get<bool>(last); v=a; }
    else v=0.5F;
    return true;
}
static auto invoke(ParameterId id,const uavcan_register_Value_1_0& in) {
    uavcan_register_Value_1_0 out{}; uavcan_register_Access_Response_1_0 response{};
    for(unsigned i=0;i<PARAMETER_CATALOG.size();++i) if(PARAMETER_CATALOG[i].id==id) {
        handle_parameter_register(i,in,out,response); return out;
    }
    assert(false); return out;
}
int main() {
    const auto name = [](ParameterId id) {
        for (const auto& p : PARAMETER_CATALOG) if (p.id == id) return p.name;
        assert(false); return std::string_view{};
    };
    assert(name(ParameterId::MAN_CAL)=="man_cal");
    assert(name(ParameterId::MOVE_CAL)=="move_cal");
    assert(name(ParameterId::ARM)=="arm");
    assert(name(ParameterId::MOVE)=="move");
    assert(name(ParameterId::ZERO_SET)=="zero_set");
    assert(name(ParameterId::GET_ANGLE)=="get_pos");
    assert(name(ParameterId::POS_SET)=="set_pos");
    assert(name(ParameterId::CAL_DIAG)=="cal_diag");
    assert(name(ParameterId::DRIVER_DIAG)=="tmc_diag");
    assert(name(ParameterId::POSITION_KP)=="kp");
    assert(name(ParameterId::POSITION_KD)=="kd");
    assert(name(ParameterId::ACTIVE_GAINS)=="active_gains");
    assert(name(ParameterId::POSITION_KI)=="ki");
    assert(name(ParameterId::INTEGRAL_LIMIT)=="i_limit");
    assert(name(ParameterId::INTEGRAL_CURRENT)=="i_current");
    uavcan_register_Value_1_0 in{}; in._tag_=5; in.integer32.value.count=1; in.integer32.value.elements[0]=1;
    auto out=invoke(ParameterId::ARM,in);
    assert(writes==1 && std::get<bool>(last)); assert(out._tag_==5 && out.integer32.value.count==5 && out.integer32.value.elements[0]==1);
    in.integer32.value.elements[0]=2; invoke(ParameterId::ARM,in); assert(writes==1 && invalid==1);
    in={}; in._tag_=13; in.real32.value.count=1; in.real32.value.elements[0]=0.5F;
    invoke(ParameterId::POS_SET,in); assert(writes==2 && std::get<float>(last)==0.5F);
    in.real32.value.elements[0]=std::numeric_limits<float>::quiet_NaN();
    invoke(ParameterId::POS_SET,in); assert(writes==2 && invalid==2);
    in={}; in._tag_=5; in.integer32.value.count=1; in.integer32.value.elements[0]=1;
    out=invoke(ParameterId::MAN_CAL,in); assert(out.integer32.value.elements[0]==2);
    in={}; invoke(ParameterId::LUFT_CAL,in); assert(backlash_started==12345);
    in._tag_=5; in.integer32.value.count=1; in.integer32.value.elements[0]=1;
    out=invoke(ParameterId::ZERO_SET,in); assert(out.integer32.value.elements[0]==0);
    // Exercise the real zero operation through both boards' generated codecs.
    cal.core().initialize(4,false,0,16384,65536,208);
    encoder_calibration_data stored{};
    stored.point_count=2; stored.zero_valid=1; stored.zero_raw=1000;
    stored.limit_a_raw=0; stored.limit_b_raw=8000; stored.manual_span_ticks=8000;
    stored.safe_margin_ticks=208;
    cal.core().import_legacy(stored); cal.refresh();
    raw=3000; in={};
    out=invoke(ParameterId::ZERO_SET,in);
    assert(out.integer32.value.elements[0]==1 && syncs==1);
    assert(cal.snapshot().zero_valid && cal.snapshot().encoder_zero==raw);
    assert(cal.snapshot().limits_valid);
    std::int32_t ticks{};
    assert(cal.core().calibrated_position_ticks(raw,ticks) && ticks==0);
    save_ok=false; raw=4000;
    out=invoke(ParameterId::ZERO_SET,in);
    assert(out.integer32.value.elements[0]==0 && syncs==1);
    assert(cal.snapshot().encoder_zero==3000);
}
