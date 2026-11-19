#include "ruka3_control/ruka3_system.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <optional>
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <thread>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "voltbro/foc/command_1_0.h"
#include "voltbro/foc/state_simple_1_0.h"

namespace ruka3_control
{
namespace
{
std::uint64_t micros()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double parameter(const hardware_interface::HardwareInfo & info, const char * name, double fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : std::stod(it->second);
}
}  // namespace

namespace detail
{
// One owner and one nonblocking socket, serviced by the controller-manager thread.
// Register RPCs wait only during activation; the regular read/write path never waits.
class CyphalRuntime
{
public:
  struct Joint
  {
    double position{}, velocity{};
    float lower{}, soft_lower{}, soft_upper{}, upper{};
    std::uint64_t feedback{}, heartbeat{};
    std::uint8_t health{};
    bool fault{};
  };
  std::array<Joint, kJointCount> joints{};
  std::array<CanardTransferID, kJointCount> command_ids{}, register_ids{};
  std::array<double, kJointCount> kp{}, kd{};
  bool tuning_enabled{};
  bool failed{};
  void fail(const std::string & reason)
  {
    if (!failed) RCLCPP_ERROR(diagnostics_node_->get_logger(), "CAN failure: %s", reason.c_str());
    failed = true;
  }

  void report_unhealthy(double feedback_timeout, double heartbeat_timeout) const
  {
    const auto now = micros();
    for (size_t i = 0; i < kJointCount; ++i) {
      const auto & j = joints[i];
      if (now-j.feedback > feedback_timeout*1e6 || now-j.heartbeat > heartbeat_timeout*1e6 ||
        j.fault || j.health >= uavcan_node_Health_1_0_CAUTION)
      {
        RCLCPP_ERROR(diagnostics_node_->get_logger(),
          "joint_%zu: feedback age=%.3fs (limit %.3fs), heartbeat age=%.3fs (limit %.3fs), fault=%d health=%u",
          i+1, (now-j.feedback)*1e-6, feedback_timeout, (now-j.heartbeat)*1e-6,
          heartbeat_timeout, j.fault, unsigned(j.health));
      }
    }
  }
  void recovery_status(bool value) {
    std_msgs::msg::Bool msg;
    msg.data = value;
    recovery_pub_->publish(msg);
  }


  explicit CyphalRuntime(const std::string & interface)
  : canard_(canardInit(
      [](CanardInstance *, size_t size) { return std::malloc(size); },
      [](CanardInstance *, void * ptr) { std::free(ptr); })),
    queue_(canardTxInit(256, CANARD_MTU_CAN_FD))
  {
    executor_.add_node(diagnostics_node_);
    gains_service_ = diagnostics_node_->create_service<SetGains>("/ruka3/set_gains",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<SetGains::Request> request) {
        auto response = std::make_shared<SetGains::Response>();
        response->results.resize(request->parameters.size());
        if (!tuning_enabled || gain_request_) {
          for (auto & result : response->results) result.reason =
            tuning_enabled ? "Another gains request is in progress" : "Hardware is not active";
          gains_service_->send_response(*header, *response);
          return;
        }
        gain_request_ = GainRequest{header, request, response};
      });
    canard_.node_id = kControllerNodeId;
    const unsigned index = if_nametoindex(interface.c_str());
    if (!index) throw std::runtime_error("SocketCAN interface not found: " + interface);
    fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error(std::strerror(errno));
    const int enable = 1;
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0 ||
      bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      const std::string error = std::strerror(errno);
      close(fd_); fd_ = -1;
      throw std::runtime_error(error);
    }
    canardRxSubscribe(&canard_, CanardTransferKindMessage, kFeedbackSubjectId,
      voltbro_foc_state_simple_1_0_EXTENT_BYTES_, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &feedback_sub_);
    canardRxSubscribe(&canard_, CanardTransferKindMessage, kHeartbeatSubjectId,
      uavcan_node_Heartbeat_1_0_EXTENT_BYTES_, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &heartbeat_sub_);
    canardRxSubscribe(&canard_, CanardTransferKindResponse, uavcan_register_Access_1_0_FIXED_PORT_ID_,
      uavcan_register_Access_Response_1_0_EXTENT_BYTES_, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &register_sub_);
  }

  ~CyphalRuntime()
  {
    canardRxUnsubscribe(&canard_, CanardTransferKindMessage, kFeedbackSubjectId);
    canardRxUnsubscribe(&canard_, CanardTransferKindMessage, kHeartbeatSubjectId);
    canardRxUnsubscribe(&canard_, CanardTransferKindResponse, uavcan_register_Access_1_0_FIXED_PORT_ID_);
    while (const auto * item = canardTxPeek(&queue_)) {
      canard_.memory_free(&canard_, canardTxPop(&queue_, item));
    }
    if (fd_ >= 0) close(fd_);
  }

  void send(CanardTransferKind kind, CanardPortID port, CanardNodeID node,
    CanardTransferID id, const void * payload, size_t size)
  {
    const CanardTransferMetadata metadata{CanardPriorityNominal, kind, port, node, id};
    const auto result = canardTxPush(&queue_, &canard_, micros() + 100000, &metadata, size, payload);
    if (result < 0) fail("TX enqueue failed: " + std::to_string(result));
  }

  void flush()
  {
    while (const auto * item = canardTxPeek(&queue_)) {
      if (item->tx_deadline_usec > micros()) {
        canfd_frame frame{};
        frame.can_id = item->frame.extended_can_id | CAN_EFF_FLAG;
        frame.flags = CANFD_BRS;
        frame.len = static_cast<__u8>(item->frame.payload_size);
        std::memcpy(frame.data, item->frame.payload, frame.len);
        if (::write(fd_, &frame, sizeof(frame)) != sizeof(frame)) {
          if (errno == EAGAIN || errno == ENOBUFS) return;
          fail(std::string("SocketCAN write: ") + std::strerror(errno));
        }
      } else {
        fail("TX deadline expired, CAN ID=" + std::to_string(item->frame.extended_can_id));
      }
      canard_.memory_free(&canard_, canardTxPop(&queue_, item));
    }
  }

  void poll()
  {
    const auto now = micros();
    if (now - last_heartbeat_ >= 1000000) {
      uavcan_node_Heartbeat_1_0 msg{};
      msg.uptime = (now - started_) / 1000000;
      msg.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
      std::uint8_t data[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{};
      size_t size = sizeof(data);
      uavcan_node_Heartbeat_1_0_serialize_(&msg, data, &size);
      send(CanardTransferKindMessage, kHeartbeatSubjectId, CANARD_NODE_ID_UNSET, heartbeat_id_++, data, size);
      last_heartbeat_ = now;
    }
    // Bound work per cycle even if another CAN participant floods the bus.
    for (unsigned count = 0; count < 256; ++count) {
      canfd_frame raw{};
      const auto size = ::read(fd_, &raw, sizeof(raw));
      if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) fail(std::string("SocketCAN read: ") + std::strerror(errno));
        break;
      }
      if ((size != CANFD_MTU && size != CAN_MTU) || !(raw.can_id & CAN_EFF_FLAG) ||
        (raw.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG))) continue;
      const CanardFrame frame{raw.can_id & CAN_EFF_MASK, raw.len, raw.data};
      CanardRxTransfer transfer{};
      const auto accepted = canardRxAccept(&canard_, now, &frame, 0, &transfer, nullptr);
      if (accepted < 0) fail("RX accept failed: " + std::to_string(accepted));
      if (accepted == 1) {
        receive(transfer, now);
        canard_.memory_free(&canard_, transfer.payload);
      }
    }
    flush();
  }

  void command(size_t index, double position, double velocity, double kp, double kd)
  {
    voltbro_foc_command_1_0 msg{};
    msg.angle.radian = position;
    msg.velocity.radian_per_second = velocity;
    msg.angle_kp.value = kp;
    msg.velocity_kp.value = kd;
    std::uint8_t data[voltbro_foc_command_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{};
    size_t size = sizeof(data);
    voltbro_foc_command_1_0_serialize_(&msg, data, &size);
    send(CanardTransferKindMessage, kServoCommandSubjectIds[index], CANARD_NODE_ID_UNSET,
      command_ids[index]++, data, size);
  }

  void request_register(size_t index, const char * name, int value = -1,
    std::optional<float> gain = std::nullopt)
  {
    uavcan_register_Access_Request_1_0 msg{};
    msg.name.name.count = std::strlen(name);
    std::memcpy(msg.name.name.elements, name, msg.name.name.count);
    if (value >= 0) {
      uavcan_register_Value_1_0_select_integer32_(&msg.value);
      msg.value.integer32.value.count = 1;
      msg.value.integer32.value.elements[0] = value;
    }
    if (gain) {
      uavcan_register_Value_1_0_select_real32_(&msg.value);
      msg.value.real32.value.count = 1;
      msg.value.real32.value.elements[0] = *gain;
    }
    std::uint8_t data[uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_]{};
    size_t size = sizeof(data);
    uavcan_register_Access_Request_1_0_serialize_(&msg, data, &size);
    pending_node_ = kFirstJointNodeId + index;
    pending_id_ = register_ids[index]++ & CANARD_TRANSFER_ID_MAX;
    response_ready_ = false;
    send(CanardTransferKindRequest, uavcan_register_Access_1_0_FIXED_PORT_ID_,
      pending_node_, pending_id_, data, size);
  }

  // Firmware responses are an internal ABI; only activation waits for register RPCs.
  bool access(size_t index, const char * name, uavcan_register_Access_Response_1_0 & response,
    int value = -1, std::optional<float> gain = std::nullopt)
  {
    request_register(index, name, value, gain);
    const auto deadline = micros() + 500000;
    do {
      poll();
      if (response_ready_) { response = response_; return true; }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (micros() < deadline && !failed);
    return false;
  }

  // Run on the CAN owner thread. Never wait for an RPC in the control loop.
  void tune()
  {
    executor_.spin_some();
    if (!gain_request_) return;
    auto & update = *gain_request_;
    if (!tuning_enabled || failed) {
      finish_tuning("Hardware is not active or CAN failed");
      return;
    }
    if (update.waiting) {
      if (!response_ready_ && micros() < update.deadline) return;
      auto & result = update.response->results[update.next];
      if (response_ready_) {
        const auto actual = response_.value.real32.value.elements[0];
        result.successful = actual == update.value;
        if (result.successful) {
          if (update.name == "kp") kp[update.joint] = actual;
          if (update.name == "kd") kd[update.joint] = actual;
          RCLCPP_INFO(diagnostics_node_->get_logger(), "joint_%zu: %s=%g applied",
            update.joint+1, update.name.c_str(), double(actual));
        } else result.reason = "Firmware did not accept the requested value";
      } else result.reason = "CAN register response timed out; application is unconfirmed";
      update.waiting = false;
      ++update.next;
    }
    while (update.next < update.request->parameters.size()) {
      const auto & parameter = update.request->parameters[update.next];
      const auto dot = parameter.name.find('.');
      const auto joint = parameter.name.substr(0, dot);
      const auto name = dot == std::string::npos ? "" : parameter.name.substr(dot+1);
      const auto found = std::find(kJointNames.begin(), kJointNames.end(), joint);
      // This is the public ROS boundary, not validation of our firmware's internal ABI.
      if (found == kJointNames.end() || (name != "kp" && name != "kd" && name != "ki") ||
        parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.value.double_value) || parameter.value.double_value < 0 ||
        !std::isfinite(static_cast<float>(parameter.value.double_value)))
      {
        update.response->results[update.next++].reason =
          "Expected joint_1..joint_6.kp/kd/ki and a finite nonnegative double";
        continue;
      }
      update.joint = std::distance(kJointNames.begin(), found);
      update.name = name;
      update.value = parameter.value.double_value;
      request_register(update.joint, name.c_str(), -1, update.value);
      update.deadline = micros() + 500000;
      update.waiting = true;
      return;
    }
    gains_service_->send_response(*update.header, *update.response);
    gain_request_.reset();
  }

  void finish_tuning(const std::string & reason)
  {
    if (!gain_request_) return;
    auto & update = *gain_request_;
    for (size_t i = update.next; i < update.response->results.size(); ++i)
      update.response->results[i].reason = reason;
    gains_service_->send_response(*update.header, *update.response);
    gain_request_.reset();
  }

  bool healthy(double feedback_timeout, double heartbeat_timeout) const
  {
    const auto now = micros();
    if (failed) return false;
    for (const auto & j : joints) {
      if (now-j.feedback > feedback_timeout*1e6 || now-j.heartbeat > heartbeat_timeout*1e6 ||
        j.fault || j.health >= uavcan_node_Health_1_0_CAUTION) return false;
    }
    return true;
  }

  void diagnostics(bool active, double feedback_timeout, double heartbeat_timeout)
  {
    const auto now = micros();
    if (now-last_diagnostics_ < 1000000) return;
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = diagnostics_node_->now();
    for (size_t i = 0; i < kJointCount; ++i) {
      const auto & j = joints[i];
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = std::string("RUKA3/") + kJointNames[i];
      status.hardware_id = std::to_string(kFirstJointNodeId+i);
      const bool stale = now-j.feedback > feedback_timeout*1e6 || now-j.heartbeat > heartbeat_timeout*1e6;
      status.level = failed || stale || j.fault || j.health >= uavcan_node_Health_1_0_CAUTION ?
        status.ERROR : status.OK;
      status.message = status.level == status.ERROR ? "fault or communication lost" : (active ? "active" : "inactive");
      for (const auto & entry : {std::make_pair("feedback_age_s", (now-j.feedback)*1e-6),
          std::make_pair("position_rad", j.position), std::make_pair("velocity_rad_s", j.velocity)}) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = entry.first; kv.value = std::to_string(entry.second);
        status.values.push_back(kv);
      }
      msg.status.push_back(status);
    }
    diagnostics_pub_->publish(msg);
    last_diagnostics_ = now;
  }

private:
  using SetGains = rcl_interfaces::srv::SetParameters;
  struct GainRequest
  {
    std::shared_ptr<rmw_request_id_t> header;
    std::shared_ptr<SetGains::Request> request;
    std::shared_ptr<SetGains::Response> response;
    size_t next{}, joint{};
    std::string name;
    float value{};
    std::uint64_t deadline{};
    bool waiting{};
  };
  std::optional<GainRequest> gain_request_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Service<SetGains>::SharedPtr gains_service_;

  void receive(const CanardRxTransfer & transfer, std::uint64_t now)
  {
    const auto source = transfer.metadata.remote_node_id;
    if (source < kFirstJointNodeId || source >= kFirstJointNodeId+kJointCount) return;
    auto & joint = joints[source-kFirstJointNodeId];
    auto size = transfer.payload_size;
    const auto * data = static_cast<const std::uint8_t *>(transfer.payload);
    if (transfer.metadata.transfer_kind == CanardTransferKindResponse) {
      if (source == pending_node_ && transfer.metadata.transfer_id == pending_id_) {
        uavcan_register_Access_Response_1_0_deserialize_(&response_, data, &size);
        response_ready_ = true;
      }
    } else if (transfer.metadata.port_id == kFeedbackSubjectId) {
      voltbro_foc_state_simple_1_0 msg{};
      voltbro_foc_state_simple_1_0_deserialize_(&msg, data, &size);
      joint.position = msg.angle.radian;
      joint.velocity = msg.velocity.radian_per_second;
      joint.fault = msg.has_fault.value;
      joint.feedback = now;
    } else {
      uavcan_node_Heartbeat_1_0 msg{};
      uavcan_node_Heartbeat_1_0_deserialize_(&msg, data, &size);
      joint.health = msg.health.value;
      joint.heartbeat = now;
    }
  }

  CanardInstance canard_;
  CanardTxQueue queue_;
  CanardRxSubscription feedback_sub_{}, heartbeat_sub_{}, register_sub_{};
  int fd_{-1};
  const std::uint64_t started_{micros()};
  std::uint64_t last_heartbeat_{}, last_diagnostics_{};
  CanardTransferID heartbeat_id_{}, pending_id_{};
  CanardNodeID pending_node_{CANARD_NODE_ID_UNSET};
  bool response_ready_{};
  uavcan_register_Access_Response_1_0 response_{};
  rclcpp::Node::SharedPtr diagnostics_node_{std::make_shared<rclcpp::Node>("ruka3_hardware_diagnostics")};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{
    diagnostics_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10)};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_pub_{
    diagnostics_node_->create_publisher<std_msgs::msg::Bool>("/ruka3/limit_recovery", rclcpp::QoS(1).transient_local())};

};
}  // namespace detail

Ruka3System::~Ruka3System() = default;

CallbackReturn Ruka3System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  can_interface_ = info.hardware_parameters.at("can_interface");
  activation_timeout_ = parameter(info, "activation_timeout", 5.0);
  feedback_timeout_ = parameter(info, "feedback_timeout", 0.25);
  heartbeat_timeout_ = parameter(info, "heartbeat_timeout", 2.5);
  return CallbackReturn::SUCCESS;
}

CallbackReturn Ruka3System::on_configure(const rclcpp_lifecycle::State &)
{
  try { bus_ = std::make_unique<detail::CyphalRuntime>(can_interface_); }
  catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "CAN configure: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Ruka3System::on_activate(const rclcpp_lifecycle::State &)
{
  const auto deadline = micros() + activation_timeout_*1e6;
  do {
    bus_->poll();
    if (bus_->healthy(feedback_timeout_, heartbeat_timeout_)) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (micros() < deadline && !bus_->failed);
  if (!bus_->healthy(feedback_timeout_, heartbeat_timeout_)) {
    RCLCPP_ERROR(get_logger(), "Activation: waiting for six healthy joint nodes timed out");
    return CallbackReturn::ERROR;
  }
  // Snapshot the firmware settings before enabling any motor.
  for (size_t i = 0; i < kJointCount; ++i) {
    uavcan_register_Access_Response_1_0 response{};
    if (!bus_->access(i, "limits", response) || response.value.real32.value.elements[0] == 0) {
      RCLCPP_ERROR(get_logger(), "joint_%zu needs mechanical calibration", i+1);
      return CallbackReturn::ERROR;
    }
    auto & j = bus_->joints[i];
    const auto * limits = response.value.real32.value.elements;
    j.lower = limits[1]; j.soft_lower = limits[3]; j.soft_upper = limits[4]; j.upper = limits[5];
    if (j.soft_lower > 0 || j.soft_upper < 0) {
      RCLCPP_ERROR(get_logger(), "joint_%zu: zero is outside calibrated soft limits", i+1);
      return CallbackReturn::ERROR;
    }
    // Each hardware activation starts from the requested startup gains.
    for (const auto & setting : {std::make_pair("kp", 10.f), std::make_pair("kd", 0.f),
        std::make_pair("ki", .1f)})
    {
      if (!bus_->access(i, setting.first, response, -1, setting.second) ||
        response.value.real32.value.elements[0] != setting.second)
      {
        RCLCPP_ERROR(get_logger(), "joint_%zu: startup %s could not be applied", i+1, setting.first);
        return CallbackReturn::ERROR;
      }
    }
    if (!bus_->access(i, "kp", response)) {
      RCLCPP_ERROR(get_logger(), "joint_%zu: kp read timed out", i+1);
      return CallbackReturn::ERROR;
    }
    bus_->kp[i] = response.value.real32.value.elements[0];
    if (!bus_->access(i, "kd", response)) {
      RCLCPP_ERROR(get_logger(), "joint_%zu: kd read timed out", i+1);
      return CallbackReturn::ERROR;
    }
    bus_->kd[i] = response.value.real32.value.elements[0];
    RCLCPP_INFO(get_logger(), "joint_%zu: firmware Kp=%g Kd=%g", i+1, bus_->kp[i], bus_->kd[i]);

  }
  for (size_t i = 0; i < kJointCount; ++i) {
    uavcan_register_Access_Response_1_0 response{};
    if (!bus_->access(i, "arm", response, 1) || response.value.integer32.value.elements[0] != 1) {
      // Roll back this activation, including an arm request whose response was lost.
      for (size_t n = 0; n <= i; ++n) bus_->access(n, "arm", response, 0);
      RCLCPP_ERROR(get_logger(), "joint_%zu did not arm", i+1);
      return CallbackReturn::ERROR;
    }
  }
  command_.fill(0); command_velocity_.fill(0);
  recovery_ = {};
  bus_->recovery_status(false);
  active_ = true;
  bus_->tuning_enabled = true;
  // Real states remain measured. Only the command is zero; firmware owns the approach profile.
  for (size_t i = 0; i < kJointCount; ++i) bus_->command(i, 0, 0, bus_->kp[i], bus_->kd[i]);
  bus_->flush();
  RCLCPP_INFO(get_logger(), "RUKA3 armed: all six targets are zero");
  return CallbackReturn::SUCCESS;
}

void Ruka3System::hold()
{
  if (!bus_ || !active_) return;
  bus_->tuning_enabled = false;
  bus_->finish_tuning("Hardware deactivated");
  // If feedback is stale, let each drive's local watchdog capture its actual position.
  if (!bus_->healthy(feedback_timeout_, heartbeat_timeout_)) { active_ = false; return; }
  for (size_t i = 0; i < kJointCount; ++i) bus_->command(i, bus_->joints[i].position, 0, bus_->kp[i], bus_->kd[i]);
  bus_->flush();
  active_ = false;
}

CallbackReturn Ruka3System::on_deactivate(const rclcpp_lifecycle::State &) { hold(); return CallbackReturn::SUCCESS; }
CallbackReturn Ruka3System::on_cleanup(const rclcpp_lifecycle::State &) { hold(); bus_.reset(); return CallbackReturn::SUCCESS; }
CallbackReturn Ruka3System::on_shutdown(const rclcpp_lifecycle::State &) { hold(); return CallbackReturn::SUCCESS; }
CallbackReturn Ruka3System::on_error(const rclcpp_lifecycle::State &) { hold(); return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::StateInterface> Ruka3System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < kJointCount; ++i) {
    result.emplace_back(kJointNames[i], "position", &position_[i]);
    result.emplace_back(kJointNames[i], "velocity", &velocity_[i]);
  }
  return result;
}
std::vector<hardware_interface::CommandInterface> Ruka3System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < kJointCount; ++i) {
    result.emplace_back(kJointNames[i], "position", &command_[i]);
    result.emplace_back(kJointNames[i], "velocity", &command_velocity_[i]);
  }
  return result;
}

hardware_interface::return_type Ruka3System::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> &)
{
  if (!start.empty()) {
    if (recovery_.latched && !recovery_.settled(bus_->joints)) {
      RCLCPP_ERROR(get_logger(), "Wait for limit recovery before restarting arm_controller");
      return hardware_interface::return_type::ERROR;
    }
    recovery_ = {};
    command_.fill(0); command_velocity_.fill(0);
    bus_->recovery_status(false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ruka3System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) return hardware_interface::return_type::ERROR;
  bus_->poll();
  for (size_t i = 0; i < kJointCount; ++i) {
    position_[i] = bus_->joints[i].position;
    velocity_[i] = bus_->joints[i].velocity;
  }
  bus_->tune();
  bus_->diagnostics(active_, feedback_timeout_, heartbeat_timeout_);
  if (active_ && !bus_->healthy(feedback_timeout_, heartbeat_timeout_)) {
    bus_->report_unhealthy(feedback_timeout_, heartbeat_timeout_);
    RCLCPP_ERROR(get_logger(), "Hardware read failed; leaving active state (CAN failed=%d)", bus_->failed);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ruka3System::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_) return hardware_interface::return_type::OK;
  if (recovery_.update(bus_->joints, period.seconds())) {
    bus_->recovery_status(true);
    RCLCPP_WARN(get_logger(), "Soft limit exceeded: returning inside limits; trajectory blocked until controller restart");
  }
  for (size_t i = 0; i < kJointCount; ++i) {
    const auto & j = bus_->joints[i];
    const auto target = recovery_.latched ? recovery_.target[i] :
      std::clamp(command_[i], double(j.soft_lower), double(j.soft_upper));
    double velocity = recovery_.latched ? recovery_.velocity[i] : command_velocity_[i];
    if (!recovery_.latched && ((target >= j.soft_upper && velocity > 0) ||
      (target <= j.soft_lower && velocity < 0))) velocity = 0;
    bus_->command(i, target, velocity, bus_->kp[i], bus_->kd[i]);
  }
  bus_->flush();
  return bus_->failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}
}  // namespace ruka3_control

PLUGINLIB_EXPORT_CLASS(ruka3_control::Ruka3System, hardware_interface::SystemInterface)
