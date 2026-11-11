#include "calibration_eeprom.h"
#include "encoder_calibration_storage.h"
#include <array>
#include <algorithm>
#include <cassert>
#include <cstring>
static std::array<uint8_t,16384> memory;
static int budget = 100000;
static bool probe_pending;
extern "C" uint8_t at24_isConnected() {
    // A consecutive address-only probe can encounter the preceding STOP/BUSY.
    const bool ready=!probe_pending; probe_pending=true; return ready;
}
extern "C" uint8_t at24_read(uint16_t address,uint8_t* data,uint8_t size,uint32_t) {
    probe_pending=false;
    assert(address+size<=memory.size()); std::memcpy(data,memory.data()+address,size); return 1;
}
extern "C" uint8_t at24_write(uint16_t address,uint8_t* data,uint8_t size,uint32_t) {
    probe_pending=false;
    assert(address>=0x3000 && address+size<=0x3600);
    const int n=std::min<int>(size,budget); std::memcpy(memory.data()+address,data,n); budget-=n; return n==size;
}
int main() {
    memory.fill(0x77);
    encoder_calibration_data a{}; a.point_count=3; a.zero_valid=1; a.zero_raw=400;
    assert(encoder_calibration_storage_save(4,&a)); assert(a.sequence==1);
    const auto baseline=memory;
    bool complete=false;
    for(int cut=0;cut<700;++cut) {
        memory=baseline; budget=cut;
        auto b=a; b.zero_raw=800;
        const bool saved=encoder_calibration_storage_save(4,&b);
        encoder_calibration_data loaded{};
        assert(encoder_calibration_storage_load(4,&loaded));
        assert(loaded.zero_raw==(saved?800:400));
        assert(std::equal(memory.begin(),memory.begin()+0x3000,baseline.begin()));
        assert(!encoder_calibration_storage_load(5,&loaded));
        if(saved) { assert(b.sequence==2); complete=true; break; }
    }
    assert(complete);
    memory[0x3300+100]^=1; // Corrupt newest record: older valid slot survives.
    encoder_calibration_data loaded{}; assert(encoder_calibration_storage_load(4,&loaded));
    assert(loaded.zero_raw==400);
}
