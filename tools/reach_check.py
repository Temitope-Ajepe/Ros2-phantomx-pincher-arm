#!/usr/bin/env python3
# Ask MoveIt if points are reachable. It does NOT move the arm.
# Usage: reach_check.py x,y,z [x,y,z ...]  (metres, arm base frame)
import sys, time
from math import degrees
from threading import Thread
import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher as r
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

rclpy.init()
node = Node("reach_check")
m = MoveIt2(node=node, joint_names=r.joint_names(), base_link_name=r.base_link_name(),
            end_effector_name=r.end_effector_name(), group_name=r.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup())
ex = rclpy.executors.MultiThreadedExecutor(2)
ex.add_node(node)
Thread(target=ex.spin, daemon=True).start()
time.sleep(2.0)
for arg in sys.argv[1:]:
    p = tuple(float(v) for v in arg.split(","))
    res = m.compute_ik(position=p, quat_xyzw=[0.0, 1.0, 0.0, 0.0])
    if res is None:
        print(p, "NOT reachable")
    else:
        a = dict(zip(res.name, res.position))
        print(p, "reachable, joints (deg):", [round(degrees(a[n]), 1) for n in m.joint_names])
rclpy.shutdown()
