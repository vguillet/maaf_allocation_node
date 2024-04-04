
##################################################################################################################


# Built-in/Generic Imports


# Libs

# ROS2 Imports
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Local Imports
from .allocation_logics.CBAA.ICBAA_allocation_node import ICBAANode


##################################################################################################################


class maaf_allocation_node:
    def __init__(self):
        self.allocation_node = ICBAANode()


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    node = maaf_allocation_node()

    # -> Spin fast
    rclpy.spin(node.allocation_node)

    node.allocation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
