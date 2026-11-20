#!/usr/bin/env python3
"""Record the min/max of every joint while the arm is moved by hand."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Limits(Node):
    def __init__(self):
        super().__init__("measure_joint_limits")
        self.lo, self.hi = {}, {}
        self.create_subscription(JointState, "/joint_states", self.callback, 10)

    def callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.lo[name] = min(pos, self.lo.get(name, pos))
            self.hi[name] = max(pos, self.hi.get(name, pos))


def main():
    rclpy.init()
    node = Limits()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("\njoint: min_deg  max_deg   (min_rad  max_rad)")
    for name in sorted(node.lo):
        lo, hi = node.lo[name], node.hi[name]
        print(f"{name}: {math.degrees(lo):7.1f} {math.degrees(hi):7.1f}   ({lo:.3f} {hi:.3f})")


if __name__ == "__main__":
    main()
