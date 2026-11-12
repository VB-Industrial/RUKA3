#!/usr/bin/env python3
"""Run against demo.launch.py use_rviz:=false; plan and execute on mock hardware."""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

rclpy.init()
node = Node('ruka3_mock_moveit_smoke')
states = []
node.create_subscription(JointState, '/joint_states', states.append, 10)
client = ActionClient(node, MoveGroup, '/move_action')
assert client.wait_for_server(timeout_sec=15)
controller = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
assert controller.wait_for_server(timeout_sec=15)
# Give MoveIt's separate controller client time to finish DDS discovery too.
deadline = time.monotonic()+2
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
assert states
initial = dict(zip(states[-1].name, states[-1].position))
print('Initial:', initial)
assert all(abs(initial[f'joint_{i}']) < .001 for i in range(1, 7))
goal = MoveGroup.Goal()
goal.request.group_name = 'ruka3_arm'
goal.request.num_planning_attempts = 3
goal.request.allowed_planning_time = 5.
goal.request.max_velocity_scaling_factor = .5
goal.request.max_acceleration_scaling_factor = .5
constraint = Constraints()
for i in range(1, 7):
    joint = JointConstraint()
    joint.joint_name = f'joint_{i}'
    joint.position = .03 if i == 1 else 0.
    joint.tolerance_above = joint.tolerance_below = .001
    joint.weight = 1.
    constraint.joint_constraints.append(joint)
goal.request.goal_constraints = [constraint]
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=10)
handle = future.result()
assert handle and handle.accepted
future = handle.get_result_async()
rclpy.spin_until_future_complete(node, future, timeout_sec=30)
assert future.done() and future.result().result.error_code.val == 1, (
    None if not future.done() else future.result().result.error_code.val)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.1)
final = dict(zip(states[-1].name, states[-1].position))
assert abs(final['joint_1']-.03) < .002, final
print('PASS: MoveIt planned and executed; final:', final)
node.destroy_node()
rclpy.shutdown()
