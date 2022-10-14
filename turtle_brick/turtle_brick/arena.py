from audioop import add
from distutils.log import info
from matplotlib.pyplot import cla
from numpy import block
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion


class Arena(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('arena')

        #create a broadcaster that will publish to /tf_static
        self.static_broadcaster = TransformBroadcaster(self)

        #create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)

        #create marker publisher
        self.pub_marker = self.create_publisher(MarkerArray,"visualization_marker_array",10)

        #marker
        self.wall1 = Marker()
        self.wall1.header.frame_id = "/world"
        self.wall1.header.stamp = self.get_clock().now().to_msg()
        self.wall1.id = 1
        self.wall1.type = self.wall1.CUBE
        self.wall1.action = self.wall1.ADD
        self.wall1.scale.x = 10.0
        self.wall1.scale.y = 0.1
        self.wall1.scale.z = 0.5
        self.wall1.color.r = 1.0
        self.wall1.color.g = 0.0
        self.wall1.color.b = 1.0
        self.wall1.color.a = 1.0
        self.wall1.pose.position.x = 0.0
        self.wall1.pose.position.y = -5.0
        self.wall1.pose.position.z = 0.0
     
        self.wall2 = Marker()
        self.wall2.header.frame_id = "/world"
        self.wall2.header.stamp = self.get_clock().now().to_msg()
        self.wall2.id = 2
        self.wall2.type = self.wall2.CUBE
        self.wall2.action = self.wall2.ADD
        self.wall2.scale.x = 10.0
        self.wall2.scale.y = 0.1
        self.wall2.scale.z = 0.5
        self.wall2.color.r = 1.0
        self.wall2.color.g = 0.0
        self.wall2.color.b = 1.0
        self.wall2.color.a = 1.0
        self.wall2.pose.position.x = 0.0
        self.wall2.pose.position.y = 5.0
        self.wall2.pose.position.z = 0.0

        self.wall3 = Marker()
        self.wall3.header.frame_id = "/world"
        self.wall3.header.stamp = self.get_clock().now().to_msg()
        self.wall3.id = 3
        self.wall3.type = self.wall3.CUBE
        self.wall3.action = self.wall3.ADD
        self.wall3.scale.x = 0.1
        self.wall3.scale.y = 10.0
        self.wall3.scale.z = 0.5
        self.wall3.color.r = 1.0
        self.wall3.color.g = 0.0
        self.wall3.color.b = 1.0
        self.wall3.color.a = 1.0
        self.wall3.pose.position.x = 5.0
        self.wall3.pose.position.y = 0.0
        self.wall3.pose.position.z = 0.0

        self.wall4 = Marker()
        self.wall4.header.frame_id = "/world"
        self.wall4.header.stamp = self.get_clock().now().to_msg()
        self.wall4.id = 4
        self.wall4.type = self.wall4.CUBE
        self.wall4.action = self.wall4.ADD
        self.wall4.scale.x = 0.1
        self.wall4.scale.y = 10.0
        self.wall4.scale.z = 0.5
        self.wall4.color.r = 1.0
        self.wall4.color.g = 0.0
        self.wall4.color.b = 1.0
        self.wall4.color.a = 1.0
        self.wall4.pose.position.x = -5.0
        self.wall4.pose.position.y = 0.0
        self.wall4.pose.position.z = 0.0


        self.wall_array = MarkerArray()
        self.wall_array.markers.append(self.wall1)
        self.wall_array.markers.append(self.wall2)
        self.wall_array.markers.append(self.wall3)
        self.wall_array.markers.append(self.wall4)
  

        #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.timer)

        self.dx = 4

    def timer(self):

        odom__brick_link = TransformStamped()
        time = self.get_clock().now().to_msg()
        odom__brick_link.header.frame_id = "odom"
        odom__brick_link.child_frame_id = "brick"
        odom__brick_link.header.stamp = time
        odom__brick_link.transform.translation.z = float(self.dx)

        self.broadcaster.sendTransform(odom__brick_link)

        self.pub_marker.publish(self.wall_array)
        
def arena_entry(args=None):
    rclpy.init(args=args)
    arena = Arena()
    rclpy.spin(arena)
    rclpy.shutdown()