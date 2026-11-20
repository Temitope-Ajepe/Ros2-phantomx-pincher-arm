#!/usr/bin/env python3
"""Nudge one arm joint by a small angle. Usage: nudge_joint.py <pan|lift|elbow|wrist> <delta_rad> <seconds>"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = {
    "pan": "phantomx_pincher_arm_shoulder_pan_joint",
    "lift": "phantomx_pincher_arm_shoulder_lift_joint",
    "elbow": "phantomx_pincher_arm_elbow_flex_joint",
    "wrist": "phantomx_pincher_arm_wrist_flex_joint",
}
ORDER = ["pan", "lift", "elbow", "wrist"]


def main():
    which, delta, seconds = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    if which not in JOINTS or abs(delta) > 0.3 or seconds < 2.0:
        print("Refused: joint must be pan/lift/elbow/wrist, |delta| <= 0.3 rad, seconds >= 2")
        return

    rclpy.init()
    node = Node("nudge_joint")
    state = {}

    def on_state(msg):
        for name, pos in zip(msg.name, msg.position):
            state[name] = pos

    node.create_subscription(JointState, "/joint_states", on_state, 10)
    pub = node.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    # wait for the current joint positions and for the controller to listen
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(JOINTS[k] in state for k in ORDER) and pub.get_subscription_count() > 0:
            break
    else:
        print("No joint states or no controller found")
        return

    target = [state[JOINTS[k]] for k in ORDER]
    target[ORDER.index(which)] += delta
    print("current:", [round(state[JOINTS[k]], 3) for k in ORDER])
    print("target: ", [round(v, 3) for v in target])

    msg = JointTrajectory()
    msg.joint_names = [JOINTS[k] for k in ORDER]
    point = JointTrajectoryPoint()
    point.positions = target
    point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
    msg.points = [point]
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
