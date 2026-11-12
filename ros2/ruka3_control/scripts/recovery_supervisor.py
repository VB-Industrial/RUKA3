#!/usr/bin/env python3
"""Cancel the active trajectory when hardware takes ownership for limit recovery."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal


class RecoverySupervisor(Node):
    def __init__(self):
        super().__init__('ruka3_recovery_supervisor')
        self.cancel = self.create_client(CancelGoal, '/arm_controller/follow_joint_trajectory/_action/cancel_goal')
        self.pending = False
        self.future = None
        self.create_subscription(Bool, '/ruka3/limit_recovery', self.recovery, rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.1, self.tick)

    def recovery(self, msg):
        self.pending = msg.data

    def tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            self.future.result()
            self.future = None
            self.pending = False
            self.get_logger().warn('Limit recovery: trajectory cancelled; restart arm_controller after recovery')
        if self.pending and self.cancel.service_is_ready():
            self.future = self.cancel.call_async(CancelGoal.Request())


def main():
    rclpy.init()
    node = RecoverySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
