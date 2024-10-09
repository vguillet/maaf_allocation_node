
##################################################################################################################


# Built-in/Generic Imports


# Libs
import cProfile
import pstats
import io

# ROS2 Imports
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Local Imports
from .allocation_logics.CBAA.ICBAA_allocation_node import ICBAANode
from .allocation_logics.CBBA.ICBBA_allocation_node import ICBBANode


##################################################################################################################


class maaf_allocation_node:
    def __init__(self):
        # self.allocation_node = ICBAANode()
        self.allocation_node = ICBBANode()


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    node = maaf_allocation_node()

    # -> Spin fast
    rclpy.spin(node.allocation_node)

    # -> Terminate node
    node.allocation_node.destroy_node()
    rclpy.shutdown()


# def main(args=None):
#     # `rclpy` library is initialized
#     rclpy.init(args=args)
#
#     node = maaf_allocation_node()
#
#     # Profile the main logic
#     pr = cProfile.Profile()
#     pr.enable()
#
#     # -> Spin fast
#     rclpy.spin(node.allocation_node)
#
#     pr.disable()
#
#     # -> Print the profiling results
#     s = io.StringIO()
#     sortby = 'cumulative'
#     ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
#     ps.print_stats()
#
#     # -> Dump to file
#     with open(f"/home/vguillet/ros2_ws/{node.allocation_node.id}.txt", "w") as f:
#         f.write(s.getvalue())
#
#     # -> Terminate node
#     node.allocation_node.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    main()
