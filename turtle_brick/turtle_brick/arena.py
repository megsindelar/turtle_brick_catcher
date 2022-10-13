from matplotlib.pyplot import cla
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion


class Arena(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('arena')

        #create a broadcaster that will repeatedly publish to /tf
        #self.broadcaster = TransformBroadcaster(self)

        #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.timer)

        self.dx = 4

    def timer(self):

        # world__brick_link = TransformStamped()
        x = self.dx
        #time = self.get_clock().now().to_msg()
        # world__brick_link.header.frame_id = "world"
        # world__brick_link.child_frame_id = "brick"
        # world__brick_link.header.stamp = time
        # world__brick_link.transform.translation.z = float(self.dx)

        # self.broadcaster.sendTransform(world__brick_link)


def arena_entry(args=None):
    rclpy.init(args=args)
    arena = Arena()
    rclpy.spin(arena)
    rclpy.shutdown()