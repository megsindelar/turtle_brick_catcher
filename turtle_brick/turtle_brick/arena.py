from audioop import add
from distutils.log import info
from matplotlib.pyplot import cla
from numpy import block
from pyrsistent import b
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Point
from turtlesim.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
import std_srvs
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from .quaternion import angle_axis_to_quaternion
from turtle_brick_interfaces.srv import Place
from turtle_brick_interfaces.msg import RobotMove, Tilt
from enum import Enum, auto

class State(Enum):
    DROP = auto()
    START = auto()
    MOVE_BRICK = auto()
    TARG = auto()
    DROPPING = auto()
    DONE = auto()


class Arena(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('arena')

        self.state = State.START

        #load parameters from yaml
        self.declare_parameter('acceleration', 9.8)
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value


        #create a broadcaster that will publish to /tf_static
        self.static_broadcaster = TransformBroadcaster(self)

        #create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)

        #create marker publisher for wall array
        self.pub_wall = self.create_publisher(MarkerArray,"visualization_marker_array",10)

        #create marker publisher for brick
        self.pub_brick = self.create_publisher(Marker,"visualization_marker",10)

        #create a subscriber to see if robot moves
        self.sub = self.create_subscription(RobotMove, "move_robot", self.robot_move_callback, 10)

        #create a subscriber for turtle pose
        self.turtle_sub = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)

        #create a publisher for if brick hit target (either platform or ground)
        self.pub_brick_hit = self.create_publisher(Bool, "brick_hit", 10)

        #create a publisher for if brick landed on the ground after platform tilt
        self.pub_brick_landed = self.create_publisher(Bool, "brick_landed", 10)

        #create a service for brick to fall
        self.place = self.create_service(Place,"place",self.brick_callback)

        #create a service for brick to fall
        self.drop = self.create_service(Empty,"drop",self.drop_callback)

        #create a subscriber for if brick hit target
        self.hit_targ_sub = self.create_subscription(Bool,'hit_targ',self.hit_targ_callback, 10)

        #create a subscriber for tilt_plat to tilt brick
        self.brick_tilt_sub = self.create_subscription(Tilt,'tilt_plat',self.brick_tilt_callback,10)

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
        self.wall1.scale.x = 12.0
        self.wall1.scale.y = 0.1
        self.wall1.scale.z = 0.5
        self.wall1.color.r = 1.0
        self.wall1.color.g = 0.0
        self.wall1.color.b = 1.0
        self.wall1.color.a = 1.0
        self.wall1.pose.position.x = -1.5
        self.wall1.pose.position.y = -0.5
        self.wall1.pose.position.z = 0.0
     
        self.wall2 = Marker()
        self.wall2.header.frame_id = "/world"
        self.wall2.header.stamp = self.get_clock().now().to_msg()
        self.wall2.id = 2
        self.wall2.type = self.wall2.CUBE
        self.wall2.action = self.wall2.ADD
        self.wall2.scale.x = 12.0
        self.wall2.scale.y = 0.1
        self.wall2.scale.z = 0.5
        self.wall2.color.r = 1.0
        self.wall2.color.g = 0.0
        self.wall2.color.b = 1.0
        self.wall2.color.a = 1.0
        self.wall2.pose.position.x = -1.5
        self.wall2.pose.position.y = 11.5
        self.wall2.pose.position.z = 0.0

        self.wall3 = Marker()
        self.wall3.header.frame_id = "/world"
        self.wall3.header.stamp = self.get_clock().now().to_msg()
        self.wall3.id = 3
        self.wall3.type = self.wall3.CUBE
        self.wall3.action = self.wall3.ADD
        self.wall3.scale.x = 0.1
        self.wall3.scale.y = 12.0
        self.wall3.scale.z = 0.5
        self.wall3.color.r = 1.0
        self.wall3.color.g = 0.0
        self.wall3.color.b = 1.0
        self.wall3.color.a = 1.0
        self.wall3.pose.position.x = 4.5
        self.wall3.pose.position.y = 5.5
        self.wall3.pose.position.z = 0.0

        self.wall4 = Marker()
        self.wall4.header.frame_id = "/world"
        self.wall4.header.stamp = self.get_clock().now().to_msg()
        self.wall4.id = 4
        self.wall4.type = self.wall4.CUBE
        self.wall4.action = self.wall4.ADD
        self.wall4.scale.x = 0.1
        self.wall4.scale.y = 12.0
        self.wall4.scale.z = 0.5
        self.wall4.color.r = 1.0
        self.wall4.color.g = 0.0
        self.wall4.color.b = 1.0
        self.wall4.color.a = 1.0
        self.wall4.pose.position.x = -7.5
        self.wall4.pose.position.y = 5.5
        self.wall4.pose.position.z = 0.0

        self.wall_array = MarkerArray()
        self.wall_array.markers.append(self.wall1)
        self.wall_array.markers.append(self.wall2)
        self.wall_array.markers.append(self.wall3)
        self.wall_array.markers.append(self.wall4)

        self.brick = Marker()
        self.brick.header.frame_id = "/brick"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.type = self.brick.CUBE
        self.brick.action = self.brick.ADD
        self.brick.scale.x = 0.2
        self.brick.scale.y = 0.3
        self.brick.scale.z = 0.1
        self.brick.color.r = 0.7
        self.brick.color.g = 0.3
        self.brick.color.b = 0.3
        self.brick.color.a = 1.0
  

        #create a timer callback to broadcast transforms at 250 Hz
        self.timer = self.create_timer(0.004, self.timer)

        #create a timer callback to broadcast transforms at 10 Hz
        self.timer_wall = self.create_timer(0.1, self.timer_wall)

        self.count1 = 0
        self.dx = 4
        self.dy = 2

        self.g = 9.81
        self.freq = 250

        self.F_move = 0
        self.goal = 0
        self.brick_hit = Bool()
        self.brick_hit.data = False

        self.turtle_pose = Pose()

        self.turtle_pose_past_x = 0
        self.turtle_pose_past_y = 0

        self.F_tilt = 0

        self.tilt_brick = 0

        self.done = 0

        self.brick_land = Bool()
        self.brick_land.data = False

        self.y_brick_fall = 5.5
        self.z_brick_fall = 0.0
        # self.odom = "odom"
        # self.base = "base_link"

    def brick_callback(self, request, response):
        self.brick_init_x = request.x
        self.brick_init_y = request.y
        self.brick_init_z = request.z
        #brick_init_x = -4.0
        #brick_init_y = -4.0 250Hz, get dist, falls a specific dist every time it gets callback
        self.state = State.MOVE_BRICK

        response.x = self.brick_init_x
        response.y = self.brick_init_y
        response.z = self.brick_init_z
        return response

    def hit_targ_callback(self,msg):
        self.targ = msg
        self.get_logger().info(f'targ: {self.targ.data}')
        self.get_logger().info(f'done: {self.done}')
        if self.targ.data == True and self.done == 0:
            self.state = State.TARG
            self.get_logger().info("hi buddy")
            self.done = 1

    def brick_tilt_callback(self,msg):
        self.brick_tilt_rad = msg.tilt
        self.get_logger().info(f'brick tilt: {self.brick_tilt_rad}')

    def turtle_pose_callback(self,msg):
        self.turtle_pose = msg

    def robot_move_callback(self, msg):
        self.robot_move_data = msg
        self.robot_move = self.robot_move_data.robotmove
        self.plat_z = self.robot_move_data.height

    def drop_callback(self, request, response):
        if self.state == State.MOVE_BRICK:
            self.state = State.DROP
        self.n = 1
        return response

    def timer_wall(self):
        self.pub_wall.publish(self.wall_array)

    def timer(self):
        if self.state == State.START:
            self.odom__brick_link = TransformStamped()
            time = self.get_clock().now().to_msg()
            self.odom__brick_link.header.frame_id = "odom"
            self.odom__brick_link.child_frame_id = "brick"
            self.odom__brick_link.header.stamp = time
            self.done = 0

        elif self.state == State.MOVE_BRICK:
            self.odom__brick_link.transform.translation.x = self.brick_init_x
            self.odom__brick_link.transform.translation.y = self.brick_init_y
            self.odom__brick_link.transform.translation.z = self.brick_init_z

            self.dz = self.brick_init_z

            time = self.get_clock().now().to_msg()
            self.odom__brick_link.header.stamp = time
            self.broadcaster.sendTransform(self.odom__brick_link)
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.pub_brick.publish(self.brick)

        elif self.state == State.DROP:
            #brick falls to platform
            self.z_goal = self.plat_z + 0.1 
            self.F_move = 1

            if self.dz > self.z_goal:
                self.dz = self.brick_init_z - 0.5*self.acceleration*((self.n/self.freq)**2)
                self.n+=1

            self.odom__brick_link.transform.translation.z = float(self.dz)

            time = self.get_clock().now().to_msg()
            self.odom__brick_link.header.stamp = time
            self.broadcaster.sendTransform(self.odom__brick_link)
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.pub_brick.publish(self.brick)


            self.get_logger().info(f'brick actually hit: {self.brick_hit}')
            self.abs_z = abs(self.dz - self.z_goal)
            self.get_logger().info(f'abs: {self.abs_z}')
            if self.abs_z < 0.08:
                self.brick_hit.data = True
                self.get_logger().info(f'brick actually hit: {self.brick_hit}')
                

        elif self.state == State.TARG:
            self.get_logger().info("TARG STATE!")

            self.odom__brick_link.transform.translation.x = self.turtle_pose.x
            self.odom__brick_link.transform.translation.y = self.turtle_pose.y

            x_center = 5.5
            y_center = 5.5

            diff_x = abs(x_center - self.odom__brick_link.transform.translation.x)
            diff_y = abs(y_center - self.odom__brick_link.transform.translation.y)

            self.get_logger().info(f'diff x: {diff_x}')
            self.get_logger().info(f'diff y: {diff_y}')

            if self.F_tilt == 1:
                self.odom__brick_link.transform.rotation.x = 0.0
                self.odom__brick_link.transform.translation.y = 5.05
                self.odom__brick_link.transform.translation.z = -0.9
                self.tilt_brick = 0
                self.state = State.DONE

            if (diff_x < 0.05 and diff_y < 0.05) or self.tilt_brick == 1:
                #tilt brick and fall off platform
                self.get_logger().info("TILT BRICK")
                self.tilt_brick = 1

                if self.F_tilt == 0:
                    self.odom__brick_link.transform.rotation.x = self.brick_tilt_rad
                    if self.y_brick_fall > 5.05:
                        self.y_brick_fall -= 0.01
                    else:
                        self.y_brick_fall = 5.05

                    if self.z_brick_fall > -0.9:
                        self.z_brick_fall -= 0.01
                    else:
                        self.z_brick_fall = -0.9

                    if self.z_brick_fall == -0.9 and self.y_brick_fall == 5.05:
                        self.F_tilt = 1

                    self.odom__brick_link.transform.translation.y = self.y_brick_fall
                    self.odom__brick_link.transform.translation.z = self.z_brick_fall
                    

            self.get_logger().info(f'f tilt: {self.F_tilt}')
            self.get_logger().info(f'z brick fall: {self.z_brick_fall}')
            self.get_logger().info(f'y brick fall: {self.y_brick_fall}')
            self.get_logger().info(f'trans x: {self.odom__brick_link.transform.translation.x}')
            self.get_logger().info(f'trans y: {self.odom__brick_link.transform.translation.y}')
            self.get_logger().info(f'trans z: {self.odom__brick_link.transform.translation.z}')
            self.get_logger().info(f'rot x: {self.odom__brick_link.transform.rotation.x}')


            time = self.get_clock().now().to_msg()
            self.odom__brick_link.header.stamp = time
            self.broadcaster.sendTransform(self.odom__brick_link)
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.pub_brick.publish(self.brick)

        elif self.state == State.DONE:
            self.brick_land.data = True
            self.get_logger().info("DONE")

            self.odom__brick_link.transform.translation.x = self.brick_init_x
            self.odom__brick_link.transform.translation.y = self.brick_init_y
            self.odom__brick_link.transform.translation.z = self.brick_init_z

            self.F_tilt = 0
            self.brick_hit.data = False
            self.done = 0
            self.tilt_brick = 0

            self.y_brick_fall = 5.5
            self.z_brick_fall = 0.0

            self.state = State.START

            time = self.get_clock().now().to_msg()
            self.odom__brick_link.header.stamp = time
            self.broadcaster.sendTransform(self.odom__brick_link)
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.brick.lifetime.sec = 1
            self.pub_brick.publish(self.brick)


        #odom__brick_link.transform.translation.z = float(self.dz)
        #self.get_logger().info(f"State: {self.state}")
        self.pub_brick_landed.publish(self.brick_land)
        self.pub_brick_hit.publish(self.brick_hit)
        
def arena_entry(args=None):
    rclpy.init(args=args)
    arena = Arena()
    rclpy.spin(arena)
    rclpy.shutdown()