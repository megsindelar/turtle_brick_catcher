from tokenize import Double
from types import NoneType
from unicodedata import name
from matplotlib.pyplot import angle_spectrum
from sklearn.metrics import euclidean_distances
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Point, PoseWithCovariance, TwistWithCovariance
from turtlesim.msg import Pose
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion
from std_msgs.msg import Bool, Float64
from turtle_brick_interfaces.msg import RobotMove, Tilt
from nav_msgs.msg import Odometry
import numpy as np

class Robot(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('turtle_robot')
        #load parameters from yaml file
        self.declare_parameter('platform_height')
        self.platform_height = self.get_parameter('platform_height').get_parameter_value().double_value
        self.declare_parameter('wheel_radius')
        self.wheel_rad = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.declare_parameter('max_velocity')
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.declare_parameter('acceleration')
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value

        # self.plat_height = self.get_parameter('platform_height').get_parameter_value().double_value
        # self.vel_max = self.get_parameter('max_velocity').get_parameter_value().double_value
        # self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value

        

        #create a static broadcaster which will publish once to /tf_static
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        #create a joint_state_publisher
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        #create a publisher for cmd_vel   
        self.cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        #create a subscriber to goal_pose
        self.goal_pose_sub = self.create_subscription(Point,"goal_pose",self.goal_pose_callback,10)

        #create a subscriber to see if robot moves
        self.move_robot_sub = self.create_subscription(RobotMove, "move_robot", self.move_robot_callback, 10)

        #create a publisher for if brick hit target (either ground or platform)
        self.hit_targ_pub = self.create_publisher(Bool, "hit_targ", 10)

        #create a subscriber for if brick hit target (either platform or ground)
        self.brick_hit_sub = self.create_subscription(Bool, "brick_hit", self.brick_hit_callback, 10)

        #create a subscriber to turtle_pose
        self.turtle_pose = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)

        #create a subsciber to tilt_plat to tilt the platform
        self.tilt_plat_sub = self.create_subscription(Tilt, 'tilt_plat', self.tilt_plat_callback, 10)

        #create a subscriber to see if brick landed after platform tilt
        self.brick_landed_sub = self.create_subscription(Bool, 'brick_landed', self.brick_landed_callback, 10)

        #create a subsriber to see if brick is on ground
        self.brick_ground_sub = self.create_subscription(Bool, 'brick_ground', self.brick_ground_callback, 10)

        #create a publisher for wheel odometry
        self.wheel_odometry_pub = self.create_publisher(Odometry, "odom", 10)

        #create a subscriber for wheel odometry to test
        #self.wheel_odometry_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)


        world__odom_link = TransformStamped()
        world__odom_link.header.stamp = self.get_clock().now().to_msg()
        world__odom_link.header.frame_id = "world"
        world__odom_link.child_frame_id = "odom"
        world__odom_link.transform.translation.x = -7.0          #needs to be turtles initial conditions
        world__odom_link.transform.translation.z = 1.0          #needs to be turtles initial conditions
        self.static_broadcaster.sendTransform(world__odom_link)

        #create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)
        #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.timer)

        #NEED TO TILT PLAT BACK

        self.dx = 1
        self.lx = 3.0
        self.ly = 0.0
        self.lz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.i = 0
        self.count = 0
        self.robot_move = False

        self.max_vel = 1.0

        self.move_robot_now = 0
        self.wait = 0

        self.targ = Bool()
        self.targ.data = False

        self.brick_hit = 0

        self.offset_plat_joint = 0.0
        self.base_stem_joint = 0.0
        self.stem_wheel_joint = 0.0
        self.plat_joint_vel = 0.0
        self.stem_joint_vel = 0.0
        self.wheel_joint_vel = 0.0

        self.F_tilt = 0

        self.tilt_platform = 0

        self.wheel_turn = Odometry()
        self.wheel_turn.pose
        self.wheel_pose = PoseWithCovariance()
        self.wheel_twist = TwistWithCovariance()


    def brick_hit_callback(self,msg):
        self.brick_platform = msg
        if self.brick_platform.data == True:
            self.brick_hit = 1

    def turtle_pose_callback(self,msg):
        self.pose = msg

    def goal_pose_callback(self,msg):
        self.goal = msg
        self.get_logger().info(f'goal pose: {self.goal}')

    def move_robot_callback(self,msg):
        self.brick = msg
        self.robot_move = self.brick.robotmove
        if self.robot_move == True:
            self.move_robot_now = 1

    def brick_ground_callback(self,msg):
        self.brick_ground = msg.data

    def tilt_plat_callback(self,msg):
        self.plat_tilt_rad = msg.tilt

    def brick_landed_callback(self,msg):
        self.brick_land = msg.data

    # def odom_callback(self,msg):
    #     self.wheel_odom = msg
    #     self.get_logger().info(f'wheel odom sub: {self.wheel_odom}')

    def timer(self):
        odom__base_link = TransformStamped()
        time = self.get_clock().now().to_msg()
        odom__base_link.header.frame_id = "odom"
        odom__base_link.child_frame_id = "base_link"
        odom__base_link.header.stamp = time

        if self.i<8:
            self.lx += 1.0
            self.i+=1


        self.move = Twist()

        if self.move_robot_now == 1:
            self.get_logger().info('Hi')
            self.diff_x = self.goal.x - self.pose.x 
            self.diff_y = self.goal.y - self.pose.y

            self.get_logger().info(f'log diff x: {self.diff_x}')
            self.get_logger().info(f'log diff y: {self.diff_y}')


            self.get_logger().info(f'log pose x: {self.pose.x}')
            self.get_logger().info(f'log pose y: {self.pose.y}')

            self.theta = np.arctan2(self.diff_y, self.diff_x)
            self.vel_x = self.max_vel*np.cos(self.theta)
            self.vel_y = self.max_vel*np.sin(self.theta)

            self.stem_wheel_joint = self.theta

            self.get_logger().info(f'log vel x: {self.vel_x}')
            self.get_logger().info(f'log vel y: {self.vel_y}')


            # self.wheel_turn.header.frame_id = "stem"
            # self.wheel_turn.child_frame_id = "wheel"
            # self.wheel_turn.header.stamp = self.get_clock().now().to_msg()
            # self.axis_wheel = [0, 0, 1.0]
            # self.theta_wheel = 0.6
            # self.quaternion = angle_axis_to_quaternion(self.theta_wheel, self.axis_wheel)
            # self.wheel_turn.pose.pose.orientation = self.quaternion
            # self.wheel_turn.twist.angular.z = 0.5

            # self.wheel_odometry_pub.publish(self.wheel_turn)


            self.F_tilt = 0

            if abs(self.diff_x) > 0.05:
                self.move.linear.x = float(self.vel_x)
            else:
                self.move.linear.x = 0.0
                
            if abs(self.diff_y) > 0.05:
                self.move.linear.y = float(self.vel_y)
            else:
                self.move.linear.y = 0.0

            self.lin_x = self.move.linear.x
            self.lin_y = self.move.linear.y

            self.cmd_vel_pub.publish(self.move)

            odom__base_link.transform.translation.x = float(self.pose.x)
            odom__base_link.transform.translation.y = float(self.pose.y)

            self.abs_diff_x = abs(self.goal.x - self.pose.x)
            self.abs_diff_y = abs(self.goal.y - self.pose.y)
            self.get_logger().info(f'abs diff x: {self.abs_diff_x}')
            self.get_logger().info(f'abs diff y: {self.abs_diff_y}')
            self.get_logger().info(f'move x: {self.move.linear.x}')
            self.get_logger().info(f'move y: {self.move.linear.y}')
           

            if self.abs_diff_x < 0.05 and self.abs_diff_y < 0.05:
                self.wait = 1
                self.move_robot_now = 0

        
        elif self.move_robot_now == 0 and self.wait == 1:
            #if self.hit_targ == True:
            self.get_logger().info('WAITING')
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0
            odom__base_link.transform.translation.x = float(self.pose.x)
            odom__base_link.transform.translation.y = float(self.pose.y)
            
            if self.brick_hit == 1:
                self.get_logger().info(f'brick_hit: {self.brick_hit}')

                self.get_logger().info('AHHHHHHHHHHHHHHHH')

                self.targ.data = True

                x_center = 5.5
                y_center = 5.5
                self.difference_x = x_center - self.pose.x
                self.difference_y = y_center - self.pose.y

                self.theta = np.arctan2(self.difference_y, self.difference_x)
                self.vel_x = self.max_vel*np.cos(self.theta)
                self.vel_y = self.max_vel*np.sin(self.theta)

                self.get_logger().info(f'log diff x: {self.difference_x}')
                self.get_logger().info(f'log diff y: {self.difference_y}')


                self.get_logger().info(f'log pose x: {self.pose.x}')
                self.get_logger().info(f'log pose y: {self.pose.y}')

                self.get_logger().info(f'log vel x: {self.vel_x}')
                self.get_logger().info(f'log vel y: {self.vel_y}')


                if abs(self.difference_x) > 0.05:
                    self.move.linear.x = float(self.vel_x)
                else:
                    self.move.linear.x = 0.0
                    
                if abs(self.difference_y) > 0.05:
                    self.move.linear.y = float(self.vel_y)
                else:
                    self.move.linear.y = 0.0

                self.cmd_vel_pub.publish(self.move)

                odom__base_link.transform.translation.x = float(self.pose.x)
                odom__base_link.transform.translation.y = float(self.pose.y)

                if self.move.linear.x == 0.0 and self.move.linear.y == 0.0:
                    self.wait = 0
                    self.tilt_platform = 1
                      

              

        # elif self.move_robot_now == 0 and self.wait == 1:
        #     self.move.linear.x = 0.0
        #     self.move.linear.y = 0.0
        #     odom__base_link.transform.translation.x = float(self.pose.x)
        #     odom__base_link.transform.translation.y = float(self.pose.y)

        else:
            print("NOOOOOOOOOOOOOOOOOO")
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0

            odom__base_link.transform.translation.x = 5.5
            odom__base_link.transform.translation.y = 5.5

            self.stem_wheel_joint = 0.0

            #joint states
            if self.tilt_platform == 1:

                self.targ.data = False

                if self.F_tilt == 0:
                    if self.offset_plat_joint < self.plat_tilt_rad:
                        self.offset_plat_joint += 0.05
                    else:
                        self.offset_plat_joint = self.plat_tilt_rad
                        self.F_tilt = 1

                elif self.F_tilt == 2:
                    if self.offset_plat_joint > 0.0:
                        self.offset_plat_joint -= 0.05
                        self.get_logger().info(f'plat_joint_in: {self.offset_plat_joint}')
                    else:
                        self.offset_plat_joint = 0.0
                        self.tilt_platform = 0
                        self.brick_hit = 0

                if self.F_tilt == 1 and self.brick_ground == True:
                    self.F_tilt = 2

            #self.get_logger().info(f'F_tilt: {self.F_tilt}')
            #self.get_logger().info(f'plat_joint: {self.offset_plat_joint}')


        self.get_logger().info(f'move_turtle: {self.move}')
        self.cmd_vel_pub.publish(self.move)
        self.hit_targ_pub.publish(self.targ)


        #wheel odometry publisher
        # self.wheel_turn.header.stamp = self.get_clock().now().to_msg()
        # self.wheel_turn.header.frame_id = 
        # self.wheel_turn.child_frame_id = 
        # self.wheel_turn.pose = 
        # self.wheel_turn.twist = 
        # self.wheel_odometry_pub.publish(self.wheel_turn)

        #if self.count == 30:

        self.broadcaster.sendTransform(odom__base_link)

        self.joints = JointState()
        self.joints.header.stamp = self.get_clock().now().to_msg()
        self.joints.name = ['offset_to_platform', 'base_to_stem', 'stem_to_wheel']
        self.joints.position = [float(self.offset_plat_joint), float(self.base_stem_joint), float(self.stem_wheel_joint)]
        self.joints.velocity = [float(self.plat_joint_vel), float(self.stem_joint_vel), float(self.wheel_joint_vel)]
        self.joint_state_publisher.publish(self.joints)

        self.get_logger().info(f'wait: {self.wait}')
        self.get_logger().info(f'rob mov: {self.move_robot_now}')
        #else:
        #    self.count+=1

        self.wheel_turn.header.frame_id = "stem"
        self.wheel_turn.child_frame_id = "wheel"
        self.wheel_turn.header.stamp = self.get_clock().now().to_msg()
        self.axis_wheel = [0, 0, 1.0]
        self.theta_wheel = 0.6
        self.quaternion = angle_axis_to_quaternion(self.theta_wheel, self.axis_wheel)
        self.wheel_turn.pose.pose.orientation = self.quaternion
        self.wheel_turn.twist.twist.angular.z = 0.6
        self.get_logger().info(f'wheel odom: {self.wheel_turn}')
        self.wheel_turn.header.stamp = self.get_clock().now().to_msg()
        self.wheel_odometry_pub.publish(self.wheel_turn)

        self.get_logger().info(f'wheel rad: {self.wheel_rad}')



def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    rclpy.shutdown()