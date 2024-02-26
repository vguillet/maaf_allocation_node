
from math import pi
from numpy import arctan2, arcsin

from rclpy.time import Time


def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw (rad).
    quat = [x, y, z, w]

    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = arctan2(sinr_cosp, cosr_cosp) * 180 / pi

    sinp = 2 * (w * y - z * x)
    pitch = arcsin(sinp) * 180 / pi

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = arctan2(siny_cosp, cosy_cosp) * 180 / pi

    return roll, pitch, yaw


def timestamp_from_ros_time(ros_time: Time) -> float:
    """
    Convert a ROS Time object to a Python datetime timestamp.

    :param ros_time: A ROS Time object.
    :return: A Python datetime timestamp.
    """
    return ros_time.nanosec / 1e9 + ros_time.sec


def timestamp_to_ros_time(timestamp: float) -> Time:
    """
    Convert a Python datetime timestamp to a ROS Time object.

    :param timestamp: A Python datetime timestamp.
    :return: A ROS Time object.
    """
    # Extract seconds and nanoseconds from the timestamp
    seconds = int(timestamp)
    nanoseconds = int((timestamp - seconds) * 1e9)

    # Create a Time object from seconds and nanoseconds
    return Time(seconds=seconds, nanoseconds=nanoseconds)