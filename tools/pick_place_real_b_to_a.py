#!/usr/bin/env python3
# DRY RUN of pick and place A (left) -> B (right) with nothing in the gripper. Very slow.
# Poses are joint angles in degrees: pan, lift, elbow, wrist.
import os, time
from math import radians
from threading import Thread
import rclpy
from builtin_interfaces.msg import Duration
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher as r
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = [0, 0, 0, 0]
A_UP, A_DOWN = [36.9, 26.0, 82.0, 63.9], [36.9, 45.7, 81.4, 53.3]
B_UP, B_DOWN = [-36.9, 26.0, 82.0, 63.9], [-36.9, 45.7, 81.4, 53.3]
OPEN, CLOSED = 0.0155, 0.0095   # head about 1.5 cm: close slightly past it
FINGERS = ["phantomx_pincher_gripper_finger1_joint", "phantomx_pincher_gripper_finger2_joint"]

rclpy.init()
node = Node("pick_place_dry_run")
m = MoveIt2(node=node, joint_names=r.joint_names(), base_link_name=r.base_link_name(),
            end_effector_name=r.end_effector_name(), group_name=r.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(), execute_via_moveit=True)
m.max_velocity = 0.1
m.max_acceleration = 0.1
pub = node.create_publisher(JointTrajectory, "/gripper_trajectory_controller/joint_trajectory", 10)
ex = rclpy.executors.MultiThreadedExecutor(2)
ex.add_node(node)
Thread(target=ex.spin, daemon=True).start()
time.sleep(2.0)


def arm(name, deg):
    print(name, deg, flush=True)
    m.move_to_configuration([radians(d) for d in deg])
    m.wait_until_executed()
    time.sleep(1.0)


def gripper(name, opening):
    print("gripper", name, flush=True)
    point = JointTrajectoryPoint(positions=[opening, opening], time_from_start=Duration(sec=3))
    pub.publish(JointTrajectory(joint_names=FINGERS, points=[point]))
    time.sleep(4.5)


gripper("open", OPEN)
arm("hover B", B_UP)
arm("down at B", B_DOWN)
input("Place the figure between the open pads at B (hands away from the joints), then press Enter: ")
gripper("close", CLOSED)
arm("up at B", B_UP)
arm("hover A", A_UP)
arm("down at A", A_DOWN)
gripper("open", OPEN)
arm("up at A", A_UP)
arm("hover B again", B_UP)
arm("home", HOME)
print("finished", flush=True)
os._exit(0)
