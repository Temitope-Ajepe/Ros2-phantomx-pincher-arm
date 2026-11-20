#!/usr/bin/env python3
"""Move ONE arm joint by a small angle through MoveIt, very slowly.
Usage: moveit_nudge.py <pan|lift|elbow|wrist> <delta_rad>
"""
import sys
import time
from threading import Thread

import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

NAMES = ["pan", "lift", "elbow", "wrist"]


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in NAMES:
        print("usage: moveit_nudge.py <pan|lift|elbow|wrist> <delta_rad>")
        return
    index = NAMES.index(sys.argv[1])
    delta = float(sys.argv[2])
    if abs(delta) > 0.3:
        print("refused: keep |delta| <= 0.3 rad")
        return

    rclpy.init()
    node = Node("moveit_nudge")
    moveit2 = MoveIt2(
        node=node,
        joint_names=phantomx_pincher.joint_names(),
        base_link_name=phantomx_pincher.base_link_name(),
        end_effector_name=phantomx_pincher.end_effector_name(),
        group_name=phantomx_pincher.MOVE_GROUP_ARM,
        callback_group=ReentrantCallbackGroup(),
        execute_via_moveit=True,
    )
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.1

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    # wait for the first joint state
    for _ in range(50):
        if moveit2.joint_state is not None:
            break
        time.sleep(0.1)
    state = moveit2.joint_state
    if state is None:
        print("no joint state received")
        return
    positions = dict(zip(state.name, state.position))
    target = [positions[name] for name in moveit2.joint_names]
    print("current:", [round(v, 3) for v in target])
    target[index] += delta
    print("target: ", [round(v, 3) for v in target])

    moveit2.move_to_configuration(target)
    moveit2.wait_until_executed()
    print("done")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
