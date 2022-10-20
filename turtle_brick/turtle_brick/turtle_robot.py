import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Point
from turtlesim.msg import Pose
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion
from std_msgs.msg import Bool
from turtle_brick_interfaces.msg import RobotMove, Tilt
from nav_msgs.msg import Odometry
import numpy as np


def wheel_vel_turn(goal_x, goal_y, pose_x, pose_y, max_vel):
    """Compute the turn of the wheel and velocity of robot.

    Args:
        goal_x: position in x-axis (float)
        goal_y: position in y-axis (float)
        pose_x: position in x-axis (float)
        pose_y: position in y-axis (float)
        max_vel: velocity (float)

    Returns
    -------
        theta: rotation angle in radians
        theta_turn: rotation angle in radians
        vel_x: velocity in x-direction
        vel_y: velocity in y-direction

    """
    diff_x = goal_x - pose_x
    diff_y = goal_y - pose_y
    theta = np.arctan2(diff_y, diff_x)
    theta_turn = np.arctan2(diff_x, diff_y)
    vel_x = max_vel*np.cos(theta)
    vel_y = max_vel*np.sin(theta)

    return theta, theta_turn, vel_x, vel_y


class Robot(Node):
    """Move robot link/joint frames in space.

    Static Broadcasts:
        world to odom
    Broadcasts:
        odom to base_link
    Publishers:
        joint_states (sensor_msgs/msg/JointState): publishes joint commands to the robot
        turtle1/cmd_vel (geometry_msgs/msg/Twist): publishes velocities to move the robot
        hit_targ (std_msgs/msg/Bool): publishes that the platform is tilting
        odom (nav_msgs/msg/Odometry): publishes wheel odometry
    Subscribers:
        goal_pose (geometry_msgs/msg/Point): goal position for robot
        move_robot (turtle_brick_interfaces/msg/RobotMove): see if robot moves and plat height
        brick_hit (std_msgs/msg/Bool): checks if brick hit platform or ground
        turtle1/pose (turtlesim/msg/Pose): position of turtle in turtlesim
        tilt_plat (turtle_brick_interfaces/msg/Tilt): radians to tilt the platform
        brick_ground (std_msgs/msg/Bool): brick is off the platform
    Parameters:
        max_velocity (double): default is 1.0
        wheel_radius (double): default is 0.3
    Timers:
        timer: runs at 100 Hz
    """

    def __init__(self):
        super().__init__('turtle_robot')
        # load parameters from yaml file
        self.declare_parameter('max_velocity', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.declare_parameter('wheel_radius', 0.3)
        self.wheel_rad = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # create a static broadcaster which will publish once to /tf_static
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # create a joint_state_publisher
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # create a publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        # create a subscriber to goal_pose
        self.goal_pose_sub = self.create_subscription(Point, "goal_pose",
                            self.goal_pose_callback, 10)

        # create a subscriber to see if robot moves
        self.move_robot_sub = self.create_subscription(RobotMove,
              "move_robot", self.move_robot_callback, 10)

        # create a publisher for platform tilting (brick should fall)
        self.hit_targ_pub = self.create_publisher(Bool, "hit_targ", 10)

        # create a subscriber for if brick hit target (either platform or ground)
        self.brick_hit_sub = self.create_subscription(Bool, "brick_hit",
                            self.brick_hit_callback, 10)

        # create a subscriber to turtle_pose
        self.turtle_pose = self.create_subscription(Pose, "turtle1/pose",
                        self.turtle_pose_callback, 10)

        # create a subsciber to tilt_plat to tilt the platform
        self.tilt_plat_sub = self.create_subscription(Tilt, 'tilt_plat',
                            self.tilt_plat_callback, 10)

        # create a subsriber to see if brick is off the platform
        self.brick_ground_sub = self.create_subscription(Bool, 'brick_ground',
                            self.brick_ground_callback, 10)

        # create a publisher for wheel odometry
        self.wheel_odometry_pub = self.create_publisher(Odometry, "odom", 10)

        # static broadcast for world frame to odom frame
        world__odom_link = TransformStamped()
        world__odom_link.header.stamp = self.get_clock().now().to_msg()
        world__odom_link.header.frame_id = "world"
        world__odom_link.child_frame_id = "odom"
        world__odom_link.transform.translation.x = -7.0
        world__odom_link.transform.translation.z = 1.0
        self.static_broadcaster.sendTransform(world__odom_link)

        # create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)
        # create a timer callback to broadcast transforms at 100 Hz
        self.freq = 100
        self.timer = self.create_timer((1/self.freq), self.timer)

        # initializing variables
        self.robot_move = False
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
        self.wheel_circ = 2*np.pi*self.wheel_rad
        self.F_tilt = 0
        self.wheel_roll = 0.0
        self.theta_wheel = 0.0
        self.tilt_platform = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.F_dist = 0
        self.odometry = Odometry()

    def brick_hit_callback(self, msg):
        """Get if brick hit platform.

        A callback function for /brick_hit (std_msgs/msg/Bool) topic

        Args:
            msg: the data from the topic /brick_hit

        Returns
        -------
            no returns

        """
        self.brick_platform = msg
        if self.brick_platform.data is True:
            self.brick_hit = 1

    def turtle_pose_callback(self, msg):
        """Turtle position data.

        A callback function for /turtle1/pose (turtlesim/msg/Pose) topic

        Args:
            msg: the data from the topic turtle1/pose

        Returns
        -------
            no returns

        """
        self.pose = msg

    def goal_pose_callback(self, msg):
        """Goal position for the robot.

        A callback function for /goal_pose (geometry_msgs/msg/Point) topic

        Args:
            msg: the data from the topic /goal_pose

        Returns
        -------
            no returns

        """
        self.goal = msg

    def move_robot_callback(self, msg):
        """To see if robot moves and platform height.

        A callback function for /move_robot (turtle_brick_interfaces/msg/RobotMove) topic

        Args:
            msg: the data from the topic /move_robot

        Returns
        -------
            no returns

        """
        self.brick = msg
        self.robot_move = self.brick.robotmove
        if self.robot_move is True:
            self.move_robot_now = 1

    def brick_ground_callback(self, msg):
        """Get if brick is off the platform.

        A callback function for /brick_ground (std_msgs/msg/Bool) topic

        Args:
            msg: the data from the topic /brick_ground

        Returns
        -------
            no returns

        """
        self.brick_ground = msg.data

    def tilt_plat_callback(self, msg):
        """ How much to tilt the platform in radians.
        
        A callback function for /tilt_plat (turtle_brick_interfaces/msg/Tilt) topic

        Args:
            msg: the data from the topic /tilt_plat

        Returns
        -------
            no returns

        """
        self.plat_tilt_rad = msg.tilt

    def timer(self):
        """Timer callback at 100 Hz.

        Publishes:
            joint_states (sensor_msgs/msg/JointState): joint commands to the robot
            turtle1/cmd_vel (geometry_msgs/msg/Twist): velocities to move the robot
            hit_targ (std_msgs/msg/Bool): that the platform is tilting
            odom (nav_msgs/msg/Odometry): wheel odometry

        Broadcasts:
            odom to base_link

        Args:
            no arguments

        Returns
        -------
            no returns

        """
        odom__base_link = TransformStamped()
        time = self.get_clock().now().to_msg()
        odom__base_link.header.frame_id = "odom"
        odom__base_link.child_frame_id = "base_link"
        odom__base_link.header.stamp = time

        self.move = Twist()

        if self.move_robot_now == 1:
            self.diff_x = self.goal.x - self.pose.x
            self.diff_y = self.goal.y - self.pose.y
            self.theta, self.theta_turn, self.vel_x, self.vel_y = wheel_vel_turn(self.goal.x,
                         self.goal.y, self.pose.x, self.pose.y, self.max_velocity)

            if self.F_dist == 0:
                self.euclid_dist = np.sqrt(self.diff_x**2 + self.diff_y**2)
                self.F_dist = 1

            self.stem_turn_step = (1 / self.freq)*(self.wheel_circ / self.euclid_dist
                                                   )*self.max_velocity
            if self.stem_wheel_joint > -30.0:
                self.stem_wheel_joint -= self.stem_turn_step

            self.theta_wheel = self.stem_wheel_joint

            self.base_stem_joint = self.theta_turn
            self.theta_stem = self.theta_turn

            self.F_tilt = 0

            if abs(self.diff_x) > 0.05:
                self.move_x =float(self.vel_x)
            else:
                self.move_x = 0.0

            if abs(self.diff_y) > 0.05:
                self.move_y =float(self.vel_y)
            else:
                self.move_y = 0.0

            self.lin_x = self.move_x
            self.lin_y = self.move_y
            self.move = Twist(linear=Vector3(x=self.max_velocity*np.cos(self.theta),
                    y=self.max_velocity*np.sin(self.theta), z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0))
            self.cmd_vel_pub.publish(self.move)

            odom__base_link.transform.translation.x =float(self.pose.x)
            odom__base_link.transform.translation.y =float(self.pose.y)

            self.abs_diff_x = abs(self.goal.x - self.pose.x)
            self.abs_diff_y = abs(self.goal.y - self.pose.y)

            if self.abs_diff_x < 0.05 and self.abs_diff_y < 0.05:
                self.wait = 1
                self.move_robot_now = 0
                self.stem_wheel_joint = 0.0

        elif self.move_robot_now == 0 and self.wait == 1:
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0
            odom__base_link.transform.translation.x =float(self.pose.x)
            odom__base_link.transform.translation.y =float(self.pose.y)

            self.theta, self.theta_turn, self.vel_x, self.vel_y = wheel_vel_turn(self.goal.x,
                                                                                 self.goal.y,
                                                                                 self.pose.x,
                                                                                 self.pose.y,
                                                                                 self.max_velocity)

            self.base_stem_joint = self.theta_turn
            self.theta_stem = self.theta_turn

            if self.brick_hit == 1:

                self.targ.data = True

                if self.stem_wheel_joint < 30.0:
                    self.stem_wheel_joint += self.stem_turn_step

                self.theta_wheel = self.stem_wheel_joint

                x_center = 5.5
                y_center = 5.5
                self.difference_x = x_center - self.pose.x
                self.difference_y = y_center - self.pose.y

                self.theta = np.arctan2(self.difference_y, self.difference_x)
                self.vel_x = self.max_velocity*np.cos(self.theta)
                self.vel_y = self.max_velocity*np.sin(self.theta)

                if abs(self.difference_x) > 0.05:
                    self.move_x = float(self.vel_x)
                else:
                    self.move_x = 0.0

                if abs(self.difference_y) > 0.05:
                    self.move_y = float(self.vel_y)
                else:
                    self.move_y = 0.0

                self.move = Twist(linear=Vector3(x=self.max_velocity*np.cos(self.theta),
                    y = self.max_velocity*np.sin(self.theta), z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0))
                self.cmd_vel_pub.publish(self.move)

                odom__base_link.transform.translation.x = float(self.pose.x)
                odom__base_link.transform.translation.y = float(self.pose.y)

                if self.move_x == 0.0 and self.move_y == 0.0:
                    self.wait = 0
                    self.tilt_platform = 1

        else:
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0

            odom__base_link.transform.translation.x = 5.5
            odom__base_link.transform.translation.y = 5.5

            self.base_stem_joint = 0.0
            self.theta_stem = 0.0
            self.stem_wheel_joint = 0.0
            self.theta_wheel = 0.0

            # joint states
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
                    else:
                        self.offset_plat_joint = 0.0
                        self.tilt_platform = 0
                        self.brick_hit = 0

                if self.F_tilt == 1 and self.brick_ground is True:
                    self.F_tilt = 2

        self.cmd_vel_pub.publish(self.move)
        self.hit_targ_pub.publish(self.targ)

        self.broadcaster.sendTransform(odom__base_link)

        self.joints = JointState()
        self.joints.header.stamp = self.get_clock().now().to_msg()
        self.joints.name = ['offset_to_platform', 'base_to_stem', 'stem_to_wheel']
        self.joints.position = [float(self.offset_plat_joint), float(self.base_stem_joint),
            float(self.stem_wheel_joint)]
        self.joints.velocity = [float(self.plat_joint_vel), float(self.stem_joint_vel),
            float(self.wheel_joint_vel)]
        self.joint_state_publisher.publish(self.joints)

        # Wheel odometry publisher
        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id = "base_link"
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.axis_wheel = [1, 1, 1]
        self.odom_theta = 0.0
        self.quaternion = angle_axis_to_quaternion(self.odom_theta, self.axis_wheel)
        self.odometry.pose.pose.orientation = self.quaternion
        self.odometry.pose.pose.position.x = odom__base_link.transform.translation.x
        self.odometry.pose.pose.position.y = odom__base_link.transform.translation.y
        self.odometry.pose.pose.position.z = odom__base_link.transform.translation.z
        self.wheel_odometry_pub.publish(self.odometry)


def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    rclpy.shutdown()
