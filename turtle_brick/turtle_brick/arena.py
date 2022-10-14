from audioop import add
from distutils.log import info
from matplotlib.pyplot import cla
from numpy import block
from pyrsistent import b
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
import std_srvs
from std_srvs.srv import Empty
from .quaternion import angle_axis_to_quaternion
#from turtle_brick_interfaces.srv import Place


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

        #create marker publisher for wall array
        self.pub_wall = self.create_publisher(MarkerArray,"visualization_marker_array",10)

        #create marker publisher for brick
        self.pub_brick = self.create_publisher(Marker,"visualization_marker",10)

        #create a service for brick to fall
        #self.place = self.create_service(Place,"place",self.brick_callback)

        #create a service for brick to fall
        #self.drop = self.create_service(Empty,"drop",self.drop_callback)

        """
        NOTEE FOR SELF
        
        need to re-adjust wall 
        """

        #marker
        self.wall1 = Marker()
        self.wall1.header.frame_id = "/world"
        self.wall1.header.stamp = self.get_clock().now().to_msg()
        self.wall1.id = 1
        self.wall1.type = self.wall1.CUBE
        self.wall1.action = self.wall1.ADD
        self.wall1.scale.x = 11.0
        self.wall1.scale.y = 0.1
        self.wall1.scale.z = 0.5
        self.wall1.color.r = 1.0
        self.wall1.color.g = 0.0
        self.wall1.color.b = 1.0
        self.wall1.color.a = 1.0
        self.wall1.pose.position.x = -5.5
        self.wall1.pose.position.y = -5.5
        self.wall1.pose.position.z = 0.0
     
        self.wall2 = Marker()
        self.wall2.header.frame_id = "/world"
        self.wall2.header.stamp = self.get_clock().now().to_msg()
        self.wall2.id = 2
        self.wall2.type = self.wall2.CUBE
        self.wall2.action = self.wall2.ADD
        self.wall2.scale.x = 11.0
        self.wall2.scale.y = 0.1
        self.wall2.scale.z = 0.5
        self.wall2.color.r = 1.0
        self.wall2.color.g = 0.0
        self.wall2.color.b = 1.0
        self.wall2.color.a = 1.0
        self.wall2.pose.position.x = -5.5
        self.wall2.pose.position.y = 5.5
        self.wall2.pose.position.z = 0.0

        self.wall3 = Marker()
        self.wall3.header.frame_id = "/world"
        self.wall3.header.stamp = self.get_clock().now().to_msg()
        self.wall3.id = 3
        self.wall3.type = self.wall3.CUBE
        self.wall3.action = self.wall3.ADD
        self.wall3.scale.x = 0.1
        self.wall3.scale.y = 11.0
        self.wall3.scale.z = 0.5
        self.wall3.color.r = 1.0
        self.wall3.color.g = 0.0
        self.wall3.color.b = 1.0
        self.wall3.color.a = 1.0
        self.wall3.pose.position.x = 0.0
        self.wall3.pose.position.y = 0.0
        self.wall3.pose.position.z = 0.0

        self.wall4 = Marker()
        self.wall4.header.frame_id = "/world"
        self.wall4.header.stamp = self.get_clock().now().to_msg()
        self.wall4.id = 4
        self.wall4.type = self.wall4.CUBE
        self.wall4.action = self.wall4.ADD
        self.wall4.scale.x = 0.1
        self.wall4.scale.y = 11.0
        self.wall4.scale.z = 0.5
        self.wall4.color.r = 1.0
        self.wall4.color.g = 0.0
        self.wall4.color.b = 1.0
        self.wall4.color.a = 1.0
        self.wall4.pose.position.x = -11.0
        self.wall4.pose.position.y = 0.0
        self.wall4.pose.position.z = 0.0

        self.wall_array = MarkerArray()
        self.wall_array.markers.append(self.wall1)
        self.wall_array.markers.append(self.wall2)
        self.wall_array.markers.append(self.wall3)
        self.wall_array.markers.append(self.wall4)
  

        #create a timer callback to broadcast transforms at 250 Hz
        self.timer = self.create_timer(0.004, self.timer)

        self.count1 = 0

        self.dx = 4
        self.dy = 2
        self.dz = 10.0

        self.brick_move()

    #def brick_callback(self, request, response):
    def brick_move(self):
        #brick_init = request.move_brick

        #brick_init_x = brick_init[0]
        #brick_init_y = brick_init[1]
        brick_init_x = -4.0
        brick_init_y = -4.0

        odom__brick_link = TransformStamped()
        time = self.get_clock().now().to_msg()
        odom__brick_link.header.frame_id = "odom"
        odom__brick_link.child_frame_id = "brick"
        odom__brick_link.header.stamp = time
        odom__brick_link.transform.translation.x = float(brick_init_x)
        odom__brick_link.transform.translation.y = float(brick_init_y)

        self.broadcaster.sendTransform(odom__brick_link)

        # response.x = brick_init_x
        # response.y = brick_init_y
        # return response

    #def drop_callback(self, request, response):
    def drop(self):
        #check brick z pose and turtle bot 
        if self.dz > 0.0:
            self.dz -= 1.0
        return 

    def timer(self):

        odom__brick_link = TransformStamped()
        time = self.get_clock().now().to_msg()
        odom__brick_link.header.frame_id = "odom"
        odom__brick_link.child_frame_id = "brick"
        odom__brick_link.header.stamp = time
        odom__brick_link.transform.translation.x = float(self.dx)
        odom__brick_link.transform.translation.y = float(self.dy)

        self.brick = Marker()
        self.brick.header.frame_id = "/brick"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.type = self.brick.CUBE
        self.brick.action = self.brick.ADD
        self.brick.scale.x = 0.2
        self.brick.scale.y = 0.3
        self.brick.scale.z = 0.1
        self.brick.color.r = 1.0
        self.brick.color.g = 0.0
        self.brick.color.b = 0.0
        self.brick.color.a = 1.0

        if self.count1 > 20:
            self.drop()
            self.count1=0
        else:
            self.count1+=1

        odom__brick_link.transform.translation.z = float(self.dz)

        self.broadcaster.sendTransform(odom__brick_link)

        self.pub_brick.publish(self.brick)

        self.pub_wall.publish(self.wall_array)

        
def arena_entry(args=None):
    rclpy.init(args=args)
    arena = Arena()
    rclpy.spin(arena)
    rclpy.shutdown()