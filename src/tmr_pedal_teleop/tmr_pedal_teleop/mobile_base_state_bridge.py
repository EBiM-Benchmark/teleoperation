"""Republish the swerve base's odometry in the two message types LABS can record.

LABS' dataset builder understands ``PoseStamped`` and ``TwistStamped`` observations but
not ``nav_msgs/Odometry``, and it hard-drops ``/tf`` at dataset-build time - so base pose
would be missing from every exported episode. Splitting odometry into an explicit pose
and twist topic here means LABS needs no new message support, which keeps the vendored
``frankarobotics/labs`` checkout mergeable.

The input topic name is a parameter because it has to be confirmed against the live
robot::

    ros2 topic list | grep -i swerve
    ros2 topic info /swerve_drive_controller/odometry
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class MobileBaseStateBridge(Node):
    def __init__(self):
        super().__init__("mobile_base_state_bridge")

        self.declare_parameter("odom_topic", "/swerve_drive_controller/odometry")
        self.declare_parameter("pose_topic", "/mobile_base/pose")
        self.declare_parameter("twist_topic", "/mobile_base/twist")

        gp = self.get_parameter
        odom_topic = gp("odom_topic").value

        self.pose_pub = self.create_publisher(PoseStamped, gp("pose_topic").value, 10)
        self.twist_pub = self.create_publisher(TwistStamped, gp("twist_topic").value, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f"mobile_base_state_bridge: {odom_topic} -> "
            f"{gp('pose_topic').value} + {gp('twist_topic').value}"
        )

    def _on_odom(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        # Velocities are expressed in child_frame_id (the base), not header.frame_id
        # (the odom frame), so label the twist with the frame it is actually in.
        twist.header.stamp = msg.header.stamp
        twist.header.frame_id = msg.child_frame_id or msg.header.frame_id
        twist.twist = msg.twist.twist
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
