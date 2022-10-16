from email.mime import base
from re import M
from sklearn.metrics import euclidean_distances
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from math import sqrt
from geometry_msgs.msg import Point
from enum import Enum, auto
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from turtle_brick_interfaces.msg import RobotMove

class State(Enum):
    DETECTED = auto()
    UNDETECTED = auto()

class Catcher(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('catcher')     

        self.state = State.UNDETECTED 

        self.get_logger().info("HI!")

        #create a listener for brick position  
        self.tf_buffer = Buffer()  
        self.tf_brick_listener = TransformListener(self.tf_buffer, self)

        #create a publisher to send velocity commands to turtlesim
        self.goal_pose_pub = self.create_publisher(Point,"goal_pose",10)

        #create a publisher to say whether the robot will move
        self.robot_move_pub = self.create_publisher(RobotMove, "move_robot", 10)

        #create marker publisher for brick
        self.pub_text = self.create_publisher(Marker,"visualization_marker",10)


        #Declare a frequency parameter for timer callback
        #self.declare_parameter("frequency", 100.0)
        #self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.frequency = 100
        # #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer((1/self.frequency), self.timer)

        self.text = Marker()
        self.text.header.frame_id = "/world"
        self.text.id = 5
        self.text.header.stamp = self.get_clock().now().to_msg()
        self.text.type = self.text.TEXT_VIEW_FACING
        self.text.action = self.text.ADD
        self.text.text = "Unreachable"
        self.text.lifetime.sec = 3
        self.text.scale.x = 10.0
        self.text.scale.y = 10.0
        self.text.scale.z = 3.0
        self.text.color.r = 0.9
        self.text.color.g = 0.9
        self.text.color.b = 0.9
        self.text.color.a = 1.0
        self.text.pose.position.x = 0.0
        self.text.pose.position.y = 0.0
        self.text.pose.position.z = 0.0


        self.max_vel = 0.22

        self.odom = "odom"
        self.brick = "brick"
        self.base = "base_link"

        self.x_brick_prev = 0
        self.y_brick_prev = 0
        self.z_brick_prev = 0
        self.F = 0

        self.marker = 0

        self.life = 3

        self.counts = self.frequency*self.life

        self.counter = 0

        self.robot = RobotMove()

        self.robot.robotmove = False

        self.move = Point()

        self.count = 0


    def brick_to_base(self,x_brick,y_brick,z_brick,x_plat,y_plat,z_plat):
        z_diff = z_brick - z_plat

        if z_diff > 0:
            g = 9.81
            euclid_dist = sqrt((x_brick - x_plat)**2 + (y_brick - y_plat)**2)
            t_b = sqrt(((2*(z_brick - z_plat))/g))      #brick falling
            t_r = euclid_dist/self.max_vel
        
            if t_r <= t_b:
                #robot can catch the brick
                self.move.x = x_brick
                self.move.y = y_brick
                self.move.z = z_brick
                self.goal_pose_pub.publish(self.move)

                self.robot.robotmove = True
                self.robot.height = z_plat
                self.robot_move_pub.publish(self.robot)
                self.get_logger().info(f"MOVE TO GOAL: {self.move}")
                self.get_logger().info(f"Rob: {self.robot}")
                self.marker = 0
                # if self.count < 20:
                #     self.count += 1
                # else:
                #     self.robot.robotmove = False
                #     self.count = 0
                self.state = State.DETECTED

            else:
                #robot can't catch the brick     
                self.get_logger().info(f"Can't move to goal")
                self.z_brick_prev = -10.0
                #self.state = State.TRIED    
                self.marker = 1
                self.state = State.UNDETECTED

        else: 
            self.get_logger().info("Brick starts below robot platform, can't move to goal!")
            self.z_brick_prev = -10.0
            #self.state = State.TRIED
            self.marker = 1
            

    def timer(self):
        if self.state == State.UNDETECTED:
            try:
                base_t = self.tf_buffer.lookup_transform(
                self.odom,
                self.base,
        rclpy.time.Time())
                #self.get_logger().info(f'transform: {base_t}')
                #base_t = brick_t.transform.translation.x
                #self.get_logger().info(f'base_t: {base_t}')
            except TransformException as ex:
                        #self.get_logger().info(
                        #    f'Could not transform {self.odom} to {self.base}: {ex}')
                        return

            x_base = base_t.transform.translation.x
            y_base = base_t.transform.translation.y
            z_base = base_t.transform.translation.z

            #transform base to platform
            x_plat = x_base
            y_plat = y_base
            z_plat = z_base + 0.85
            self.robot.height = z_plat

            try:
                brick_t = self.tf_buffer.lookup_transform(
                self.odom,
                self.brick,
        rclpy.time.Time())
                #self.get_logger().info(f'transform: {brick_t}')
            except TransformException as ex:
                        #self.get_logger().info(
                        #    f'Could not transform {self.odom} to {self.brick}: {ex}')
                        return

            x_brick = brick_t.transform.translation.x
            y_brick = brick_t.transform.translation.y
            z_brick = brick_t.transform.translation.z

            z_difference = abs(z_brick - self.z_brick_prev)

            #if self.F == 1:
            if z_brick!=self.z_brick_prev and z_difference < 0.0005:
                self.brick_to_base(x_brick,y_brick,z_brick,x_plat,y_plat,z_plat)
                #else:
                    #self.get_logger().info('NOT READY!')

            #else:
            #    self.F = 1
                #self.get_logger().info('HELLOOOOOOOOOOOOOOOO!')

            self.z_brick_prev = z_brick

            if self.marker == 1:
                self.text.header.stamp = self.get_clock().now().to_msg()
                self.pub_text.publish(self.text)
                if self.counter < self.counts:
                    self.counter+=1
                else:
                    self.marker=0
                    self.counter=0


        elif self.state == State.DETECTED:
            #self.get_logger().info('Detected!')
            meg = self.max_vel
            self.robot.robotmove = False

        # x_t_brick = brick_t.transform.translation.x
        # y_t_brick = brick_t.transform.translation.y
        # z_t_brick = brick_t.transform.translation.z

        # base_t = self.tf_buffer2.lookup_transform("base_link","odom",Time)
        # x_t_base = base_t.transform.translation.x
        # y_t_base = base_t.transform.translation.y
        # z_t_base = base_t.transform.translation.z + 0.85       #adding the height to the top of the platform

        #self.brick_to_base(x_t_brick,y_t_brick,z_t_brick,x_t_base,y_t_base,z_t_base)
        self.robot_move_pub.publish(self.robot)

def catcher_entry(args=None):
    rclpy.init(args=args)
    catcher = Catcher()
    rclpy.spin(catcher)
    rclpy.shutdown()