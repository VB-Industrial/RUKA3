#!/usr/bin/env python3
"""Check the VR-style launch pair on mock hardware, including MoveIt execution."""
from pathlib import Path
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers


def main():
    logs = Path.cwd() / 'log' / 'ruka3-smoke'
    logs.mkdir(parents=True, exist_ok=True)
    processes, streams = [], []

    def launch(filename, *arguments):
        stream = open(logs / filename.replace('.launch.py', '.log'), 'w')
        streams.append(stream)
        processes.append(subprocess.Popen([
            'ros2', 'launch', 'ruka3_moveit_config', filename,
            'use_mock_hardware:=true', *arguments,
        ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

    try:
        launch('with_cont.launch.py')
        rclpy.init()
        node = Node('ruka3_launch_smoke')
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert client.wait_for_service(timeout_sec=30), str(logs)
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if future.done() and {c.name: c.state for c in future.result().controller}.get('arm_controller') == 'active':
                break
            time.sleep(.1)
        else:
            raise AssertionError('arm_controller did not activate; see ' + str(logs))
        print('PASS: with_cont starts mock controllers', flush=True)
        launch('without_cont.launch.py', 'use_rviz:=false')
        subprocess.run(['python3', str(Path(__file__).with_name('mock_moveit_smoke.py'))], check=True, timeout=90)
        print('PASS: without_cont plans and executes through existing controllers', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.kill(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
