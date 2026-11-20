# Ros2-phantomx-pincher-arm

PhantomX Pincher arm (5 AX-12A servos, ArbotiX-M board) running on **ROS 2 Humble** with **ros2_control** and **MoveIt2**, including a slow pick-and-place demo (A to B and back) on the real arm.

This is Part II (bis) of the Robotics Project (University of Burgundy Europe, Master in Computer Vision and Robotics, professor Joaquin Jorge Rodriguez). No camera is used.

## What it does

- A ros2_control hardware plugin drives the real servos through the professor's C++ `ArbotixDriver`.
- Arm and gripper are controlled by `joint_trajectory_controller`; motions are planned and executed by MoveIt2.
- A pick-and-place demo carries a small cylinder (2 to 2.5 cm wide, 4 cm tall) from point A (left) to point B (right) and back, slowly and without shaking.

## Architecture

```
demo scripts (pymoveit2)
        |
     MoveIt2  ->  joint_trajectory_controller / gripper_trajectory_controller
                          |
              ros2_control hardware plugin  (phantomx_pincher_hardware)
                          |
                professor's ArbotixDriver (serial 57600, Dynamixel protocol 1.0)
                          |
           ArbotiX-M board -> servos 1-4 (arm) and 5 (gripper)
```

## ROS1 to ROS2 conversion

| ROS1 (arbotix_ros / PhantomX_robotic_arm) | ROS2 in this repo |
|---|---|
| `arbotix_python` driver node | ros2_control hardware plugin `phantomx_pincher_hardware/PhantomxPincherHardware` using the professor's `ArbotixDriver` |
| arbotix joint controllers | `joint_trajectory_controller` (arm) and `gripper_trajectory_controller` (gripper) |
| joint state publishing by the driver | `joint_state_broadcaster` |
| MoveIt (ROS1) config | `phantomx_pincher_moveit_config` (MoveIt2) |
| URDF / meshes | `phantomx_pincher_description` (xacro, `ros2_control` tag with our plugin) |
| catkin / rosrun / roslaunch | colcon / ros2 run / ros2 launch (Python launch files) |

## Packages

- `phantomx_pincher_hardware`: ros2_control hardware interface and the `ArbotixDriver` (edited, see below).
- `phantomx_pincher_description`: robot description (from snt-spacer, adapted for the real arm).
- `phantomx_pincher_moveit_config`: MoveIt2 configuration (from snt-spacer, adapted).
- `tools/`: small scripts to test joints, check reachability and run the demo.
- `demos/`: demo scripts.

## Run it

```
cd ~/arm_ws
colcon build
source install/setup.bash
ros2 launch phantomx_pincher_hardware real.launch.py
```

Then, in a second terminal:

```
python3 src/Ros2-phantomx-pincher-arm/tools/pick_place_real_both.py
```

Scripts in `tools/`: `pick_place_real.py` (A to B), `pick_place_real_b_to_a.py` (B to A), `pick_place_real_both.py` (both in one run), `moveit_goto.py` (go to a pose in degrees), `reach_check.py` (check that a point is reachable), `nudge_joint.py` and `moveit_nudge.py` (small single-joint moves), `measure_joint_limits.py`.

Safety: before launching, check the bus with the professor's `arbotix_test` (twice, with the launch closed). Unplugging the 12V supply is the only instant stop. Hold the arm when the power goes off, because the servos go limp.

## Calibration and settings (measured on the real arm)

- AX-12A: 1024 ticks = 300 degrees; neutral ticks {506, 510, 505, 512, 512}.
- Gripper: closed = 0 ticks, open = 416 ticks; each finger opens up to 0.0158 m (gap between pads about 3 cm).
- Joint limits (rad): pan +-1.35, lift -1.45 to 1.43, elbow +-2.25, wrist -1.70 to 1.38.
- Motion: MoveIt velocity and acceleration scaling 0.1; servo moving speed 60.
- Gripper torque limit: 350 of 1023 (register 34).
- Points A and B are defined in joint space: A = (0.12 m, +0.09 m), B = (0.12 m, -0.09 m) in the arm base frame, found with inverse kinematics (`reach_check.py`). The working pose lowers the pads to about 2 cm above the table.

## Changes to the professor's ArbotixDriver

- `sync_write_positions`: all five goals are sent in a single broadcast SYNC_WRITE packet per control cycle.
- `read_status_packet` skips bare write acknowledgements (`FF FF id 02 00 chk`). The ArbotiX board relays these even for broadcast writes, and the driver took them as the answer to a read. Found by capturing the raw bytes.
- Status Return Level is set to 1 at start-up and restored to 2 on deactivate.
- `read_position` reports a clear message when a servo answers with the wrong number of data bytes.
- The hardware plugin tolerates a few failed reads in a row and raises an error after more than 50.

## Problems found and how they were solved

- Elbow neutral position was wrong (195 instead of 505 ticks): measured and corrected.
- Reads failed after the first write: cause was the relayed acknowledgements, fixed in the driver (above).
- Start-up safety: each goal is set to the current position before the torque is switched on, so the arm does not jump.
- Gripper overload: squeezing too hard made servo 5 shut down (error byte 0x20), and it ignored commands until power was cycled. Fixed with the torque limit and a softer closing target.
- Loose 12V plug or 3-pin cable caused time-outs: always check the bus before a run, tape or replace flaky cables.
- Table-height calibration with a ruler, base plate taped to the table to reduce shaking.
- Rare MoveIt abort on a single move (status 6): rerunning the same move works.

## Credits and licences

- `phantomx_pincher_description` and `phantomx_pincher_moveit_config` come from [snt-spacer/phantomx_pincher](https://github.com/snt-spacer/phantomx_pincher) (BSD 3-Clause, University of Luxembourg). Their licence is in `LICENSES/`. We changed the `ros2_control` entry, URDF, SRDF and kinematics for the real arm.
- The C++ `ArbotixDriver` is from the professor's repository [vibot-lab/PhantomX_robotic_arm](https://github.com/vibot-lab/PhantomX_robotic_arm), used with his permission and edited as listed above.
- ROS1 reference: arbotix_ros (Vanadium Labs).
- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) 3.0.0 (Andrej Orsula) for the MoveIt2 Python interface.
- Written with AI assistance (Claude by Anthropic) for debugging, code and documentation; all tests were run on the real arm by the author.

Licence of this repository: BSD 3-Clause (see `LICENSE`).
