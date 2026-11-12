#!/usr/bin/env python3
"""Run under unshare -Urn, with ROS sourced. No access to physical CAN interfaces."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListControllers

root = Path(__file__).resolve().parents[2]
log_dir = root / 'build/ros2-smoke'
log_dir.mkdir(exist_ok=True)
subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
subprocess.run(['ip', 'link', 'add', 'ruka3test', 'type', 'vcan'], check=True)
subprocess.run(['ip', 'link', 'set', 'ruka3test', 'up'], check=True)
os.environ['ROS_DOMAIN_ID'] = '93'
processes = []
streams = []
try:
    for command, filename in [
        ([str(root / 'build/ros2/ruka3_control/ruka3_virtual_joints'), 'ruka3test', str(log_dir / 'joints.log')], 'emulator.log'),
        (['ros2', 'launch', 'ruka3_control', 'real.launch.py', 'can_interface:=ruka3test'], 'real.log'),
    ]:
        stream = open(log_dir / filename, 'w')
        streams.append(stream)
        processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
    rclpy.init()
    node = Node('ruka3_virtual_can_smoke')
    states, controller, recovery = [], [], []
    node.create_subscription(JointState, '/joint_states', states.append, 10)
    node.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', controller.append, 10)
    node.create_subscription(Bool, '/ruka3/limit_recovery', recovery.append,
        rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    def wait_for(predicate, timeout):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert predicate(), (log_dir / 'real.log').read_text()[-5000:]

    wait_for(lambda: len(states) > 10 and len(controller) > 3, 12)
    positions = dict(zip(states[-1].name, states[-1].position))
    print('Initial measured positions:', positions, flush=True)
    assert positions['joint_1'] > .05, positions
    # JTC must keep hardware's zero goal even when the actual state starts at 0.3.
    assert all(abs(x) < .001 for x in controller[-1].reference.positions), controller[-1]
    wait_for(lambda: abs(dict(zip(states[-1].name, states[-1].position))['joint_1']) < .03, 5)
    trajectory = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    assert trajectory.wait_for_server(timeout_sec=3)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f'joint_{i}' for i in range(1, 7)]
    point = JointTrajectoryPoint()
    point.positions = point.velocities = [0.]*6
    point.time_from_start.sec = 20
    goal.trajectory.points = [point]
    for name in goal.trajectory.joint_names:
        tolerance = JointTolerance()
        tolerance.name = name
        tolerance.position = 2.0  # Let recovery_supervisor own cancellation in this test.
        goal.path_tolerance.append(tolerance)
    sent = trajectory.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=3)
    assert sent.result().accepted
    result = sent.result().get_result_async()
    wait_for(lambda: any(x.data for x in recovery), 10)
    rclpy.spin_until_future_complete(node, result, timeout_sec=3)
    assert result.done() and result.result().status == GoalStatus.STATUS_CANCELED, result.result()
    print('Active trajectory cancelled by recovery supervisor', flush=True)
    wait_for(lambda: states and .785 < dict(zip(states[-1].name, states[-1].position))['joint_1'] < .80, 6)
    print('Soft-limit recovery reached inner boundary', flush=True)
    wait_for(lambda: any(line.startswith('sample ') and float(line.split()[1]) > 13
        for line in (log_dir/'joints.log').read_text().splitlines()), 12)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    assert client.wait_for_service(timeout_sec=3)
    request = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, request, timeout_sec=3)
    assert request.done()
    names = {x.name: x.state for x in request.result().controller}
    assert names['arm_controller'] != 'active', names
    print('Feedback loss deactivated trajectory controller:', names, flush=True)
    entries = (log_dir/'joints.log').read_text().splitlines()
    first = [x.split() for x in entries if x.startswith('first ')]
    assert len(first)==6 and all(float(x[2])==0 and float(x[3])==30+int(x[1]) and abs(float(x[4])-.01*int(x[1]))<1e-5 for x in first), first
    samples = [list(map(float,x.split()[1:])) for x in entries if x.startswith('sample ')]
    assert max(abs(x[2]) for x in samples if x[0]<3.5)<.001, samples[:20]
    # The stale startup trajectory is never resumed after returning to 0.79.
    assert all(.785 < x[2] < .80 for x in samples if 8 < x[0] < 11), samples[-30:]
    print('PASS: node mapping, RPC arm/limits, per-joint firmware gains, zero startup, JTC ownership, recovery latch, feedback timeout', flush=True)
    node.destroy_node()
    rclpy.shutdown()
finally:
    for proc in reversed(processes):
        if proc.poll() is None:
            os.kill(proc.pid, signal.SIGINT)
    for proc in processes:
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    for stream in streams:
        stream.close()
