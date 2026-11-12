// Six Cyphal joints for integration tests on an isolated vcan interface.
// This executable never opens a physical interface: the test runner supplies its name.
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include "canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "voltbro/foc/command_1_0.h"
#include "voltbro/foc/state_simple_1_0.h"

using Clock = std::chrono::steady_clock;
static uint64_t now() { return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count(); }

struct Node {
  CanardInstance canard{canardInit([](CanardInstance *, size_t s) { return std::malloc(s); },
    [](CanardInstance *, void * p) { std::free(p); })};
  CanardTxQueue queue{canardTxInit(64, CANARD_MTU_CAN_FD)};
  CanardRxSubscription command_sub{}, register_sub{};
  CanardTransferID heartbeat_id{}, feedback_id{};
  double position{0.3}, target{0.3}, velocity{};
  bool armed{}, first{true};
  void send(CanardTransferKind kind, CanardPortID port, CanardNodeID dest, CanardTransferID tid,
    const void * data, size_t size) {
    CanardTransferMetadata meta{CanardPriorityNominal,kind,port,dest,tid};
    canardTxPush(&queue,&canard,now()+100000,&meta,size,data);
  }
};

int main(int argc, char ** argv) {
  if (argc != 3) return 2;
  const int fd = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
  const int enable=1; setsockopt(fd,SOL_CAN_RAW,CAN_RAW_FD_FRAMES,&enable,sizeof(enable));
  sockaddr_can addr{}; addr.can_family=AF_CAN; addr.can_ifindex=if_nametoindex(argv[1]);
  if (bind(fd,reinterpret_cast<sockaddr *>(&addr),sizeof(addr))) return 3;
  std::ofstream log(argv[2]);
  std::array<Node,6> nodes;
  for (size_t i=0;i<6;++i) {
    auto & n=nodes[i]; n.canard.node_id=101+i;
    canardRxSubscribe(&n.canard,CanardTransferKindMessage,1121+i,
      voltbro_foc_command_1_0_EXTENT_BYTES_,CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,&n.command_sub);
    canardRxSubscribe(&n.canard,CanardTransferKindRequest,384,
      uavcan_register_Access_Request_1_0_EXTENT_BYTES_,CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,&n.register_sub);
  }
  uint64_t last_feedback{},last_heartbeat{},last_update=now(),last_log{},armed_at{};
  bool injected=false;
  while (true) {
    const auto time=now(); const double dt=(time-last_update)*1e-6; last_update=time;
    canfd_frame raw{};
    while (::read(fd,&raw,sizeof(raw))>0) {
      CanardFrame frame{raw.can_id&CAN_EFF_MASK,raw.len,raw.data};
      for (size_t i=0;i<6;++i) {
        auto & n=nodes[i]; CanardRxTransfer transfer{};
        if (canardRxAccept(&n.canard,time,&frame,0,&transfer,nullptr)!=1) continue;
        auto size=transfer.payload_size; auto data=static_cast<const uint8_t *>(transfer.payload);
        if (transfer.metadata.transfer_kind==CanardTransferKindRequest) {
          uavcan_register_Access_Request_1_0 req{};
          uavcan_register_Access_Request_1_0_deserialize_(&req,data,&size);
          const std::string name(reinterpret_cast<const char *>(req.name.name.elements),req.name.name.count);
          uavcan_register_Access_Response_1_0 response{};
          if (name=="arm") {
            if (req.value._tag_) n.armed=req.value.integer32.value.elements[0];
            uavcan_register_Value_1_0_select_integer32_(&response.value);
            response.value.integer32.value.count=5; response.value.integer32.value.elements[0]=n.armed;
            log << "arm " << i << " " << n.armed << std::endl;
          } else if (name=="kp" || name=="kd") {
            uavcan_register_Value_1_0_select_real32_(&response.value);
            response.value.real32.value.count=1;
            response.value.real32.value.elements[0]=name=="kp" ? 30.f+i : .01f*i;
          } else if (name=="limits") {
            uavcan_register_Value_1_0_select_real32_(&response.value);
            response.value.real32.value.count=9;
            const std::array<float,9> limits{1,-1,-1,-.8F,.8F,1,1,0,0};
            std::copy(limits.begin(),limits.end(),response.value.real32.value.elements);
          }
          uint8_t payload[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{}; size=sizeof(payload);
          uavcan_register_Access_Response_1_0_serialize_(&response,payload,&size);
          n.send(CanardTransferKindResponse,384,transfer.metadata.remote_node_id,transfer.metadata.transfer_id,payload,size);
        } else {
          voltbro_foc_command_1_0 cmd{}; voltbro_foc_command_1_0_deserialize_(&cmd,data,&size);
          if (n.first) { log << "first " << i << " " << cmd.angle.radian << " " << cmd.angle_kp.value << " " << cmd.velocity_kp.value << std::endl; n.first=false; }
          n.target=cmd.angle.radian;
          if (!armed_at) armed_at=time;
        }
        n.canard.memory_free(&n.canard,transfer.payload);
      }
    }
    // Force an external soft-limit excursion after startup, then stop feedback later.
    if (armed_at && !injected && time-armed_at>4000000) {
      nodes[0].position=.9; injected=true; log << "injected" << std::endl;
    }
    for (size_t i=0;i<6;++i) {
      auto & n=nodes[i];
      const auto step=n.armed ? std::clamp(n.target-n.position,-.1*dt,.1*dt) : 0;
      n.position+=step; n.velocity=dt>0 ? step/dt : 0;
      if (time-last_feedback>=10000 && (!armed_at || time-armed_at<12000000)) {
        voltbro_foc_state_simple_1_0 msg{}; msg.angle.radian=n.position; msg.velocity.radian_per_second=n.velocity;
        uint8_t data[voltbro_foc_state_simple_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{}; size_t size=sizeof(data);
        voltbro_foc_state_simple_1_0_serialize_(&msg,data,&size); n.send(CanardTransferKindMessage,1001,CANARD_NODE_ID_UNSET,n.feedback_id++,data,size);
      }
      if (time-last_heartbeat>=100000) {
        uavcan_node_Heartbeat_1_0 msg{}; msg.mode.value=uavcan_node_Mode_1_0_OPERATIONAL;
        uint8_t data[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{}; size_t size=sizeof(data);
        uavcan_node_Heartbeat_1_0_serialize_(&msg,data,&size); n.send(CanardTransferKindMessage,7509,CANARD_NODE_ID_UNSET,n.heartbeat_id++,data,size);
      }
      while (const auto * item=canardTxPeek(&n.queue)) {
        canfd_frame out{}; out.can_id=item->frame.extended_can_id|CAN_EFF_FLAG; out.flags=CANFD_BRS;
        out.len=item->frame.payload_size; std::memcpy(out.data,item->frame.payload,out.len);
        if (::write(fd,&out,sizeof(out)) != sizeof(out)) return 4;
        n.canard.memory_free(&n.canard,canardTxPop(&n.queue,item));
      }
    }
    if (time-last_feedback>=10000) last_feedback=time;
    if (time-last_heartbeat>=100000) last_heartbeat=time;
    if (armed_at && time-last_log>=100000) {
      log << "sample " << (time-armed_at)*1e-6 << " " << nodes[0].position << " " << nodes[0].target << std::endl; last_log=time;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
