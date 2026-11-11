#include "calibration_geometry.hpp"
#include <cassert>
#include <cmath>
int main() {
    // Equivalent pole positions must not average into a half-period phase error.
    const int samples[] = {-1450,-1448,-280,-278};
    const auto mean=motor_calibration::electrical_offset_mean(samples,4,1170);
    assert(mean==-1449);
    assert(motor_calibration::circular_error(-279,mean,1170)==0);
    const int wrapped[] = {580,-580,579,-579};
    const auto boundary=motor_calibration::electrical_offset_mean(wrapped,4,1170);
    assert(boundary==585);
    assert(motor_calibration::circular_error(-585,boundary,1170)==0);
    std::array<int,2048> lut{};
    assert(motor_calibration::monotonic_lut(lut,16383));
    // A smooth periodic correction is physically monotonic across the wrap.
    for (unsigned i=0;i<lut.size();++i) lut[i]=std::lround(50*std::sin(i*2*3.141592653589793/lut.size()));
    assert(motor_calibration::monotonic_lut(lut,16383));
    // Nonzero offset must not shift the table's address frame.
    constexpr unsigned raw=6278, offset=298;
    const auto index=motor_calibration::lut_index(raw);
    assert(index!=motor_calibration::lut_index(raw-offset));
    const int correction=lut[index];
    const int measured=raw-offset-correction;
    assert(measured+offset+correction==raw);
    const auto old=lut[780];
    lut[780]=1134; // Real failure class: a spurious electrical-period jump.
    assert(!motor_calibration::monotonic_lut(lut,16383));
    lut[780]=old;assert(motor_calibration::monotonic_lut(lut,16383));
    lut[0]=lut.back()+8; // Broken periodic boundary must fail too.
    assert(!motor_calibration::monotonic_lut(lut,16383));
}
