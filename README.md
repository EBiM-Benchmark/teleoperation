# Introduction  
## About
This repository implements the teleoperation framework for the [**Franka Mobile FR3 Duo**](https://franka.de/mobile-fr3-duo) used in the [**EBiM competition**](https://ebim-benchmark.github.io/competition.html).
For **manipulation tasks**, it provides the configuration and implementation for teleoperation using the [**Franka GELLO Duo**](https://franka.de/de/gello).
For **mobile tasks**, it supports teleoperation using either a **keyboard** or a **USB foot pedal**.
Unless there are specific application requirements, **keyboard-based teleoperation is recommended** for mobile navigation, as it provides a more standardized and reliable control interface.

## Package Overview
* **franka_gello_state_publisher** - Reads the states of the Franka GELLO devices and publishes the joint states of the left arm, right arm, and grippers.
* **franka_gello_state_subscriber** - Subscribes to the published GELLO states for testing or for subsequent integration with robot control.
* **keyboard_state_publisher** - Reads keyboard inputs (`W`, `A`, `S`, `D`, `Q`, `E`) and publishes the corresponding keyboard states.
* **keyboard_state_subscriber** - Subscribes to the keyboard state topic and prints the corresponding key events
* **pedal_state_publisher** - Reads the state of the USB foot pedal and publishes the pedal states.
* **pedal_state_subscriber** - Subscribes to the pedal state topic and prints the corresponding pedal actions.
* **franka_gripper_manager** - Provides functionality for controlling and managing the Franka grippers.
* **franka_fr3_arm_controllers** - Contains the control packages for the Franka FR3 robotic arms.

# Deployment Guide

## Prepare the Pixi ROS 2 Environment

Enter the workspace:

```bash
cd ~/my_ros_ws
pixi shell
```

Verify that the environment is correctly configured:

```bash
echo $CONDA_PREFIX
which ros2
python --version
echo $ROS_DISTRO
```

The output should be similar to:

```bash
/home/demo/my_ros_ws/.pixi/envs/default
/home/demo/my_ros_ws/.pixi/envs/default/bin/ros2
Python 3.9.x
humble
```

If the `pixi shell` environment is not available, you will need to install and configure **Pixi** and **ROS 2 Humble** on the new machine before proceeding.

## Install Python Dependencies

```bash
cd ~/my_ros_ws
pixi shell

pip install dynamixel-sdk tyro evdev
```

If `colcon` is not available, install it using:

```bash
pixi add colcon-core colcon-common-extensions
```

Alternatively, you can install it with `pip`:

```bash
pip install colcon-core colcon-common-extensions
```

## Configure Device Permissions

The GELLO devices communicate through serial ports and require the **dialout** group permission:

```bash
sudo usermod -aG dialout $USER
```

The USB foot pedal is accessed via `/dev/input/eventX` and requires the **input** group permission:

```bash
sudo usermod -aG input $USER
```

After executing the above commands, **log out and log back in (or reboot)** for the changes to take effect.

Verify that the permissions have been applied:

```bash
groups
```

The output should include:

```bash
dialout input
```

## Build the ROS 2 Workspace

```bash
cd ~/my_ros_ws/gello_software/ros2
colcon build --symlink-install
source install/setup.bash
```

If the build completes successfully, the ROS 2 workspace is ready to use.

# GELLO

## Verify the GELLO USB Devices

After connecting the GELLO devices, check the available serial devices:

```bash
ls /dev/serial/by-id/
```

On the current test machine, the two GELLO devices are identified as:

```text
usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00
usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00
```

> **Note:** The USB device IDs may be different on a new computer. Be sure to verify the device IDs before updating the configuration.

## GELLO Duo Configuration File

The configuration file is located at:

```text
~/my_ros_ws/gello_software/ros2/src/franka_gello_state_publisher/config/franka_gello_duo.yaml
```

Example configuration:

```yaml
LEFT:
namespace: "left"
  com_port: "usb-ROBOTIS_OpenRB-150_38F23AFA5157375037202020FF11170D-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [4.712, 3.142, 4.712, 3.142, 4.712, 3.142, 4.712]
  gripper_range_rad: [2.521, 3.253]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]

RIGHT:
  namespace: "right"
  com_port: "usb-ROBOTIS_OpenRB-150_BDEDB3875157375037202020FF102618-if00"
  num_arm_joints: 7
  joint_signs: [1, 1, -1, 1, 1, 1, 1]
  gripper: true
  assembly_offsets: [1.571, 3.142, 1.571, 3.142, 1.571, 3.142, 0.000]
  gripper_range_rad: [2.570, 3.299]
  dynamixel_torque_enable: [0,0,0,0,0,0,0,0]
  dynamixel_goal_position: [0.0,0.0,0.0,-1.571,0.0,1.571,0.0,3.509]
  dynamixel_kp_p: [30,60,0,30,0,0,0,50]
  dynamixel_kp_i: [0,0,0,0,0,0,0,0]
  dynamixel_kp_d: [250,100,80,60,30,10,5,0]
```

> **Note:**
> For `com_port`, only specify the USB device ID. Do **not** include `/dev/serial/by-id/`, as the launch file will automatically prepend this path.

## Start the GELLO Publisher

Terminal 1:

```bash id="6dj50s"
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=franka_gello_duo.yaml
```

Expected output:

```bash id="d3jsi0"
[left.gello_publisher]: Publishing GELLO joint states.
[right.gello_publisher]: Publishing GELLO joint states.
```

## Start the GELLO Subscriber

Terminal 2:

```bash id="1wy5b7"
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run franka_gello_state_subscriber franka_gello_state_subscriber
```

After moving the GELLO devices, you should see the states of the left arm, right arm, and grippers changing in the terminal output.

# Keyboard

## Start the Keyboard Publisher

Terminal 1:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

## Start the Keyboard Subscriber

Terminal 2:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run keyboard_state_subscriber keyboard_state_subscriber
```

After pressing **W**, **A**, **S**, **D**, **Q**, or **E**, the subscriber will print the corresponding key press or action.

# USB Foot Pedal

## Detect the Foot Pedal Device

After connecting the USB foot pedal, list the available input devices:

```bash
ls -l /dev/input/by-id/
```

On the current test machine, the following devices are available:

```text
usb-PCsensor_FootSwitch-event-kbd
usb-PCsensor_FootSwitch-event-mouse
usb-PCsensor_FootSwitch-event-if01
usb-PCsensor_FootSwitch-mouse
```

This project uses the following device:

```text
/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd
```

## Start the Pedal Publisher

Terminal 1:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run pedal_state_publisher pedal_state_publisher
```

If the device is detected successfully, you should see output similar to:

```text
Using pedal device: /dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd PCsensor FootSwitch Keyboard
```

## Start the Pedal Subscriber

Terminal 2:

```bash
cd ~/my_ros_ws
pixi shell
cd ~/my_ros_ws/gello_software/ros2
source install/setup.bash
ros2 run pedal_state_subscriber pedal_state_subscriber
```

Pressing the left, middle, or right pedal should cause the subscriber to print the corresponding pedal event or action.


# Troubleshooting

## Different USB IDs on a New Computer

The USB IDs assigned to the GELLO devices may differ on a new computer. To check the current device IDs, run:

```bash
ls /dev/serial/by-id/
```

Then update the corresponding entries in:

```text
franka_gello_state_publisher/config/franka_gello_duo.yaml
```

To identify the USB foot pedal device, run:

```bash
ls -l /dev/input/by-id/
```

If the device name differs from the default, update the following constant accordingly:

```python
PEDAL_DEVICE = "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd"
```









