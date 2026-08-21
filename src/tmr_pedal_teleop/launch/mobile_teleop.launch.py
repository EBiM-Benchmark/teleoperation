"""Launch the laptop-side mobile teleop: foot pedals -> base + spine + LABS.

Reads both PCsensor foot switches and bridges them to the TMR swerve base
(swerve_drive_controller/cmd_vel) and the Franka spine. Assumes the robot's own
stack (controllers, spine server) is already running and reachable over DDS
(matching ROS_DOMAIN_ID / RMW).

Pressing ``m`` in the keyboard_state_publisher terminal toggles the pedals between
DRIVE (base and spine motion) and RECORD (LABS data collection). Set ``task_id`` to a
task UUID from the LABS station's config_tasks.yml, or RECORD mode cannot start an
episode.

Set ``record:=false`` to bring up motion only, without any LABS or state-publisher nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("tmr_pedal_teleop"), "config", "pedal_map.yaml"
    )
    config = LaunchConfiguration("config")
    labs_url = LaunchConfiguration("labs_url")
    task_id = LaunchConfiguration("task_id")
    record = LaunchConfiguration("record")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_cfg,
                description="Path to the pedal_map.yaml parameter file.",
            ),
            DeclareLaunchArgument(
                "labs_url",
                default_value="http://localhost:3001",
                description="Base URL of the LABS data-collection API.",
            ),
            DeclareLaunchArgument(
                "task_id",
                default_value="",
                description="LABS task UUID new episodes are filed under "
                "(from the station's config_tasks.yml).",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description="Start the LABS bridge and state publishers as well as motion.",
            ),
            Node(
                package="pedal_state_publisher",
                executable="pedal_state_publisher",
                name="pedal_state_publisher",
                output="screen",
            ),
            # Owns the DRIVE/RECORD mode and latches it on /teleop/pedal_mode.
            Node(
                package="tmr_pedal_teleop",
                executable="mode_manager",
                name="mode_manager",
                parameters=[config],
                output="screen",
            ),
            # Reads 'm' (and w/a/s/d/q/e) from its own terminal, which must have focus.
            Node(
                package="keyboard_state_publisher",
                executable="keyboard_state_publisher",
                name="keyboard_state_publisher",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="base_bridge",
                name="base_bridge",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="spine_bridge",
                name="spine_bridge",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="labs_pedal_bridge",
                name="labs_pedal_bridge",
                parameters=[config, {"labs_url": labs_url, "task_id": task_id}],
                output="screen",
                condition=IfCondition(record),
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="mobile_base_state_bridge",
                name="mobile_base_state_bridge",
                parameters=[config],
                output="screen",
                condition=IfCondition(record),
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="spine_state_publisher",
                name="spine_state_publisher",
                parameters=[config],
                output="screen",
                condition=IfCondition(record),
            ),
        ]
    )
