#include <voltbro/foc/command_1_0.h>
#include <cassert>
#include <cstdio>
int main() {
    voltbro_foc_command_1_0 cmd{};
    cmd._torque.newton_meter=-.2F;cmd.angle.radian=.125F;cmd.velocity.radian_per_second=-.3F;
    cmd.angle_kp.value=20;cmd.velocity_kp.value=.4F;
    uint8_t buffer[voltbro_foc_command_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{};
    size_t size=sizeof(buffer);
    assert(voltbro_foc_command_1_0_serialize_(&cmd,buffer,&size)==0);
    assert(size==28);
    voltbro_foc_command_1_0 restored{};
    size_t received=size;
    assert(voltbro_foc_command_1_0_deserialize_(&restored,buffer,&received)==0);
    assert(restored._torque.newton_meter==cmd._torque.newton_meter);
    assert(restored.angle.radian==cmd.angle.radian && restored.angle_kp.value==cmd.angle_kp.value);
    for(size_t i=0;i<size;++i)std::printf("%02x",buffer[i]);
}
