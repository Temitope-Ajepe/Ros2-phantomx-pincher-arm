#!/usr/bin/env python3
"""
Note: !! This file does not contain the full solution. Your task is to implement the missing parts. !!

Demo script for pick&place task sequence. Compatible with fake, simulated and real robot.
- ros2 run phantomx_pincher_demos demo_pick_and_place.py

This demo is used as a part of Gazebo demo `demo_orbit_pick_and_place.launch.py`.
- ros2 launch phantomx_pincher_demos demo_orbit_pick_and_place.launch.py
"""

import random
from copy import deepcopy
from math import pi
from threading import Thread
from typing import Tuple

import geometry_msgs.msg
import rclpy
import std_msgs.msg
import tf2_geometry_msgs
import tf2_ros
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Size and pose of a blank module cover (box geometry)
MODULE_COVER_SIZE: Tuple[float, float, float] = (0.148, 0.148, 0.003)
MODULE_COVER_POSE = geometry_msgs.msg.PoseStamped(
    header=std_msgs.msg.Header(
        frame_id="phantomx_pincher_task_board_module_link",
    ),
    pose=geometry_msgs.msg.Pose(
        position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=MODULE_COVER_SIZE[2] / 2.0),
        orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    ),
)

# Size and pose of an object for pick&place (box geometry)
OBJECT_SIZE: Tuple[float, float, float] = (0.015,) * 3
INITIAL_OBJECT_POSE = geometry_msgs.msg.PoseStamped(
    header=std_msgs.msg.Header(
        frame_id="phantomx_pincher_task_board_module_link",
    ),
    pose=geometry_msgs.msg.Pose(
        # On top of the module (in the centre), with a tolerance of 5 mm
        position=geometry_msgs.msg.Point(
            x=0.0,
            y=0.0,
            z=MODULE_COVER_SIZE[2] + OBJECT_SIZE[2] / 2.0 + 0.005,
        ),
        orientation=geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
    ),
)

# Fraction of the arm's maximum speed (0.1 = very slow, for the real arm)
SPEED_SCALE = 0.3

def main():
    rclpy.init()

    # Create node for this demo
    node = Node("demo_pick_and_place")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interfaces
    moveit2 = MoveIt2(
        node=node,
        joint_names=phantomx_pincher.joint_names(),
        base_link_name=phantomx_pincher.base_link_name(),
        end_effector_name=phantomx_pincher.end_effector_name(),
        group_name=phantomx_pincher.MOVE_GROUP_ARM,
        callback_group=callback_group,
        execute_via_moveit=True,
    )
    gripper = GripperInterface(
        node=node,
        gripper_joint_names=phantomx_pincher.gripper_joint_names(),
        open_gripper_joint_positions=phantomx_pincher.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=phantomx_pincher.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=phantomx_pincher.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
        execute_via_moveit=True,
    )

    # Instantiate tf2 buffer and listener for looking up transforms
    tf2_buffer = tf2_ros.Buffer(node=node)
    tf2_listener = tf2_ros.TransformListener(buffer=tf2_buffer, node=node)

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = SPEED_SCALE
    moveit2.max_acceleration = SPEED_SCALE
    gripper.max_velocity = SPEED_SCALE
    gripper.max_acceleration = SPEED_SCALE

    # Wait until the simulated robot is ready to be controlled
    rate = node.create_rate(1.0)
    while rclpy.ok() and moveit2.joint_state is None:
        rate.sleep()
    rate.destroy()

    ### Setup the Planning Scene ###
    # Clean the previous scene
    reset_planning_scene(moveit2=moveit2)
    # Move to initial joint configuration
    moveit2.move_to_configuration(
        joint_positions=[
            0.0,
            0.0,
            pi / 2,
            pi / 2,
        ],
    )
    moveit2.wait_until_executed()
    # Setup the new scene
    setup_planning_scene(moveit2=moveit2)

    # Initialize the grasp pose
    grasp_pose = INITIAL_OBJECT_POSE
    grasp_pose.pose.position.z += OBJECT_SIZE[2]

    ##
    # TODO: Make your changes here (pick & place the object)
    ##
    # A is where the cube starts; B is 5 cm to the side of A
    pose_a = grasp_pose
    pose_b = deepcopy(grasp_pose)
    pose_b.pose.position.y += 0.05

    pick_and_place(moveit2, gripper, tf2_buffer, pose_a, pose_b)  # A -> B
    pick_and_place(moveit2, gripper, tf2_buffer, pose_b, pose_a)  # B -> A

    rclpy.shutdown()
    executor_thread.join()
    exit(0)




def hover(pose, height=0.10):
    """Return a copy of a pose raised by `height` metres."""
    raised = deepcopy(pose)
    raised.pose.position.z += height
    return raised


def move_to(moveit2, tf2_buffer, pose):
    """Convert a pose to the arm's base frame, move there and wait."""
    pose_base = tf2_buffer.transform(pose, phantomx_pincher.base_link_name())
    moveit2.move_to_pose(
        position=pose_base.pose.position,
        quat_xyzw=pose_base.pose.orientation,
        tolerance_orientation=0.5,
    )
    moveit2.wait_until_executed()


def pick_and_place(moveit2, gripper, tf2_buffer, pick_pose, place_pose):
    """Pick the cube at `pick_pose` and put it down at `place_pose`."""
    prefix = "phantomx_pincher_"

    gripper.open()
    gripper.wait_until_executed()

    move_to(moveit2, tf2_buffer, hover(pick_pose))
    move_to(moveit2, tf2_buffer, pick_pose)

    moveit2.attach_collision_object(
        id="object",
        link_name=prefix + "gripper_finger_base_link",
        touch_links=[
            prefix + "gripper_finger1_link",
            prefix + "gripper_finger2_link",
            prefix + "gripper_finger_base_link",
            prefix + "gripper_servo_link",
        ],
    )
    gripper.move_to_configuration([0.008, 0.008])
    gripper.wait_until_executed()

    move_to(moveit2, tf2_buffer, hover(pick_pose))
    move_to(moveit2, tf2_buffer, hover(place_pose))
    move_to(moveit2, tf2_buffer, place_pose)

    gripper.open()
    gripper.wait_until_executed()
    moveit2.detach_collision_object("object")

    move_to(moveit2, tf2_buffer, hover(place_pose))
    
def setup_planning_scene(    
    moveit2: MoveIt2,
):
    """
    Setup the planning scene of MoveIt for collision checking.
    """

    ## 1: Add `module_cover` to the planning scene
    moveit2.add_collision_box(
        "module_cover",
        pose=MODULE_COVER_POSE,
        size=MODULE_COVER_SIZE,
    )

    ## 2 (Optional): Add `object` to the planning scene
    moveit2.add_collision_box(
        "object",
        pose=INITIAL_OBJECT_POSE,
        size=OBJECT_SIZE,
    )


def reset_planning_scene(
    moveit2: MoveIt2,
):
    """
    Remove all scene objects from previous run.
    """

    moveit2.detach_all_collision_objects()
    moveit2.remove_collision_object("module_cover")
    moveit2.remove_collision_object("object")


if __name__ == "__main__":
    main()
