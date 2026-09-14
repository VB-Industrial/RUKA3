#include "main.h"

#include <array>
#include <cstdint>
#include <memory>

#include <cyphal/allocators/o1/o1_allocator.h>
#include <cyphal/cyphal.h>
#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/node/registers_utils.hpp>
#include <cyphal/providers/G4CAN.h>
#include <cyphal/subscriptions/subscription.h>

#include <uavcan/node/Health_1_0.hpp>
#include <uavcan/node/Heartbeat_1_0.hpp>
#include <uavcan/node/Mode_1_0.hpp>
#include <voltbro/foc/state_simple_1_0.hpp>

#include "robot_config.h"
#include "protocol.h"
#include <voltbro/utils.hpp>

extern FDCAN_HandleTypeDef hfdcan1;

extern "C" {
bool global_allocation_lock = false;
}

namespace {

constexpr std::uint32_t kBootRequestMagic = ruka3::protocol::kBootRequestMagic;
constexpr std::uint32_t kNetworkTimeoutMs = ruka3::protocol::kNetworkTimeoutMs;
constexpr auto& kProfile = ruka3::joint_profile(RUKA3_JOINT_INDEX);

using Heartbeat = uavcan_node_Heartbeat_1_0;
using FocState = voltbro_foc_state_simple_1_0;

std::uint64_t monotonic_micros() {
    return static_cast<std::uint64_t>(HAL_GetTick()) * 1000U;
}

UtilityConfig utilities(monotonic_micros, Error_Handler);
std::shared_ptr<CyphalInterface> interface;
ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<4>> registers_handler;

bool bootloader_reboot_pending = false;
std::uint32_t bootloader_reboot_at = 0U;
bool network_seen = false;
std::uint32_t network_last_seen_at = 0U;

void force_power_stage_safe() {
    HAL_GPIO_WritePin(DRV_WAKE_GPIO_Port, DRV_WAKE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INDUCT_EN_GPIO_Port, INDUCT_EN_Pin, GPIO_PIN_RESET);
}

void note_network_activity() {
    network_seen = true;
    network_last_seen_at = HAL_GetTick();
}

bool network_is_online() {
    return network_seen &&
           (static_cast<std::uint32_t>(HAL_GetTick() - network_last_seen_at) <= kNetworkTimeoutMs);
}

bool parse_bool_robust(const uavcan_register_Value_1_0& value, bool& parsed) {
    if (parse_register_bit(value, parsed)) {
        return true;
    }
    if ((value._tag_ == REGISTER_INTEGER32_TAG) && (value.integer32.value.count > 0U)) {
        parsed = value.integer32.value.elements[0] != 0;
        return true;
    }
    if ((value._tag_ == REGISTER_NATURAL32_TAG) && (value.natural32.value.count > 0U)) {
        parsed = value.natural32.value.elements[0] != 0U;
        return true;
    }
    if ((value._tag_ == REGISTER_REAL32_TAG) && (value.real32.value.count > 0U)) {
        parsed = value.real32.value.elements[0] != 0.0F;
        return true;
    }
    return false;
}

class NetworkHeartbeatSubscription final : public AbstractSubscription<Heartbeat> {
public:
    explicit NetworkHeartbeatSubscription(InterfacePtr interface_in)
        : AbstractSubscription<Heartbeat>(interface_in,
                                          uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_) {}

private:
    void handler(const Heartbeat&, CanardRxTransfer*) override {
        note_network_activity();
    }
};

ReservedObject<NetworkHeartbeatSubscription> network_heartbeat_subscription;

void request_bootloader(const uavcan_register_Value_1_0& input,
                        uavcan_register_Value_1_0& output,
                        RegisterAccessResponse& response) {
    note_network_activity();
    bool requested = false;
    if (parse_bool_robust(input, requested) && requested) {
        bootloader_reboot_pending = true;
        bootloader_reboot_at = HAL_GetTick() + 150U;
    }
    response.persistent = false;
    response._mutable = true;
    fill_register_bit(output, bootloader_reboot_pending);
}

void read_node_id(const uavcan_register_Value_1_0& input,
                  uavcan_register_Value_1_0& output,
                  RegisterAccessResponse& response) {
    note_network_activity();
    (void)input;
    response.persistent = true;
    response._mutable = false;
    fill_register_natural32(output, kProfile.node_id);
}

void read_network_online(const uavcan_register_Value_1_0& input,
                         uavcan_register_Value_1_0& output,
                         RegisterAccessResponse& response) {
    note_network_activity();
    (void)input;
    response.persistent = false;
    response._mutable = false;
    fill_register_bit(output, network_is_online());
}

void read_network_timeout_ms(const uavcan_register_Value_1_0& input,
                             uavcan_register_Value_1_0& output,
                             RegisterAccessResponse& response) {
    note_network_activity();
    (void)input;
    response.persistent = true;
    response._mutable = false;
    fill_register_natural32(output, kNetworkTimeoutMs);
}

[[noreturn]] void enter_bootloader() {
    force_power_stage_safe();
    (void)HAL_FDCAN_Stop(&hfdcan1);
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = kBootRequestMagic;
    TAMP->BKP1R = 0U;
    TAMP->BKP2R = 0U;
    HAL_PWR_DisableBkUpAccess();
    NVIC_SystemReset();
    while (true) {}
}

void publish_heartbeat() {
    static CanardTransferID transfer_id = 0;
    static std::uint32_t uptime = 0;
    const bool online = network_is_online();
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    Heartbeat message{
        .uptime = uptime++,
        .health = {static_cast<std::uint8_t>(online ? uavcan_node_Health_1_0_NOMINAL
                                                    : uavcan_node_Health_1_0_ADVISORY)},
        .mode = {uavcan_node_Mode_1_0_MAINTENANCE},
        .vendor_specific_status_code = static_cast<std::uint8_t>(
            RUKA3_JOINT_INDEX | (online ? 0U : 0x80U)),
    };
    interface->send_msg(&message,
                        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                        &transfer_id,
                        MICROS_S * 2U);
}

void publish_safe_state() {
    static CanardTransferID transfer_id = 0;
    FocState message{};
    message.timestamp.microsecond = monotonic_micros();
    message.has_fault.value = false;
    interface->send_msg(&message,
                        kProfile.feedback_subject_id,
                        &transfer_id,
                        MICROS_S);
}

}  // namespace

extern "C" void app() {
    force_power_stage_safe();

    interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        kProfile.node_id,
        &hfdcan1,
        50,
        utilities
    ));

    registers_handler.create(
        std::array<RegisterDefinition, 4>{{
            {"node.id", read_node_id},
            {"command.bootloader", request_bootloader},
            {"network.online", read_network_online},
            {"network.timeout_ms", read_network_timeout_ms},
        }},
        interface
    );
    node_info_reader.create(interface,
                            "ruka3.g431.bootstrap",
                            uavcan_node_Version_1_0{1, 0},
                            uavcan_node_Version_1_0{1, 0},
                            uavcan_node_Version_1_0{1, 0},
                            0);
    network_heartbeat_subscription.create(interface);

    HAL_IMPORTANT(apply_filter(0U, &hfdcan1,
                               network_heartbeat_subscription->make_filter(kProfile.node_id)))
    HAL_IMPORTANT(apply_filter(1U, &hfdcan1,
                               registers_handler->make_filter(kProfile.node_id)))
    HAL_IMPORTANT(apply_filter(2U, &hfdcan1,
                               node_info_reader->make_filter(kProfile.node_id)))
    HAL_IMPORTANT(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                               FDCAN_REJECT,
                                               FDCAN_REJECT,
                                               FDCAN_REJECT_REMOTE,
                                               FDCAN_REJECT_REMOTE))
    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler,
        0U))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))
    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))

    std::uint32_t next_heartbeat = HAL_GetTick();
    std::uint32_t next_feedback = HAL_GetTick();
    while (true) {
        interface->loop();
        force_power_stage_safe();
        const std::uint32_t now = HAL_GetTick();
        if (static_cast<std::int32_t>(now - next_heartbeat) >= 0) {
            next_heartbeat = now + 1000U;
            publish_heartbeat();
        }
        if (static_cast<std::int32_t>(now - next_feedback) >= 0) {
            next_feedback = now + 100U;
            publish_safe_state();
        }
        if (bootloader_reboot_pending &&
            (static_cast<std::int32_t>(now - bootloader_reboot_at) >= 0)) {
            enter_bootloader();
        }
    }
}
