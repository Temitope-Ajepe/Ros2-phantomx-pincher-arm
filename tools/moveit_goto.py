#!/usr/bin/env python3
# Move the arm to joint angles (degrees) through MoveIt, very slowly.
# Usage: moveit_goto.py pan lift elbow wrist
import sys, time
from math import radians
from threading import Thread
import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher as r
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

LIMITS_DEG = [(-77, 77), (-83, 81), (-128, 128), (-97, 79)]  # measured safe ranges
deg = [float(v) for v in sys.argv[1:5]]
if len(deg) != 4 or any(not (lo <= d <= hi) for d, (lo, hi) in zip(deg, LIMITS_DEG)):
    sys.exit("usage: moveit_goto.py pan lift elbow wrist (degrees, inside the safe range)")

rclpy.init()
node = Node("moveit_goto")
m = MoveIt2(node=node, joint_names=r.joint_names(), base_link_name=r.base_link_name(),
            end_effector_name=r.end_effector_name(), group_name=r.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(), execute_via_moveit=True)
m.max_velocity = 0.1
m.max_acceleration = 0.1
ex = rclpy.executors.MultiThreadedExecutor(2)
ex.add_node(node)
Thread(target=ex.spin, daemon=True).start()
time.sleep(2.0)
print("going to", deg)
m.move_to_configuration([radians(d) for d in deg])
m.wait_until_executed()
print("done")
rclpy.shutdown()
