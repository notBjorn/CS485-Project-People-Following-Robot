"""Give TIAGo the same drive topics as the TurtleBot 4.

TIAGo's base is ros2_control's mobile_base_controller, which listens on
/mobile_base_controller/cmd_vel and publishes /mobile_base_controller/odom.
This node relays

    /cmd_vel (geometry_msgs/TwistStamped)  ->  /mobile_base_controller/cmd_vel
    /mobile_base_controller/odom           ->  /odom (nav_msgs/Odometry)

so teleop, Nav2 and our nodes use /cmd_vel and /odom on either robot.
Started by tiago.launch.py.
"""

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TiagoBaseRelay(Node):

    def __init__(self):
        super().__init__('tiago_base_relay')
        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('cmd_vel_out', '/mobile_base_controller/cmd_vel')
        self.declare_parameter('odom_in', '/mobile_base_controller/odom')
        self.declare_parameter('odom_out', '/odom')

        def param(name):
            return self.get_parameter(name).value

        self.cmd_pub = self.create_publisher(TwistStamped, param('cmd_vel_out'), 10)
        self.odom_pub = self.create_publisher(Odometry, param('odom_out'), 10)
        self.create_subscription(TwistStamped, param('cmd_vel_in'), self.cmd_pub.publish, 10)
        self.create_subscription(Odometry, param('odom_in'), self.odom_pub.publish,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"Relaying {param('cmd_vel_in')} -> {param('cmd_vel_out')} and "
            f"{param('odom_in')} -> {param('odom_out')}")


def main():
    rclpy.init()
    node = TiagoBaseRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
