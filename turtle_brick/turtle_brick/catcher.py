import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import sqrt
from geometry_msgs.msg import Point
from enum import Enum, auto
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from turtle_brick_interfaces.msg import RobotMove, Tilt


class State(Enum):
    """
    Different possible states of the program.

    Purpose: determines what functions get called in the main timer
    """

    DETECTED = auto()
    UNDETECTED = auto()


class Catcher(Node):
    """
    Coordinates robot to catch falling brick.

    Listeners:
        odom to base_link
        odom to brick
    Publishers:
        goal_pose (geometry_msgs/msg/Point): goal position for robot
        move_robot (turtle_brick_interfaces/msg/RobotMove): see if robot moves and plat height
        tilt_plat (turtle_brick_interfaces/msg/Tilt): radians to tilt the platform
        brick_ground (std_msgs/msg/Bool): brick is off the platform
    Markers:
        visualization_marker (visualization_msgs/msg/Marker): text marker in /world frame
    Parameters:
        platform_height (double): default is 1.55
        wheel_radius (double): default is 0.3
        acceleration (double): default is 9.8
    Timers:
        timer: runs at 100 Hz
    """

    def __init__(self):
        super().__init__('catcher')

        self.state = State.UNDETECTED

        # load parameters from yaml
        self.declare_parameter('platform_height', 1.55)
        self.platform_height = self.get_parameter('platform_height'
                                                  ).get_parameter_value().double_value
        self.declare_parameter('wheel_radius', 0.3)
        self.wheel_rad = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.declare_parameter('acceleration', 9.8)
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value

        # create a listener for brick position
        self.tf_buffer = Buffer()
        self.tf_brick_listener = TransformListener(self.tf_buffer, self)

        # create a publisher to send velocity commands to turtlesim
        self.goal_pose_pub = self.create_publisher(Point, "goal_pose", 10)

        # create a publisher to say whether the robot will move
        self.robot_move_pub = self.create_publisher(RobotMove, "move_robot", 10)

        # create marker publisher for brick
        self.pub_text = self.create_publisher(Marker, "visualization_marker", 10)

        # create a publisher for tilt
        self.pub_tilt = self.create_publisher(Tilt, "tilt_plat", 10)

        # create a publisher for when brick is on ground
        self.brick_tilt_pub = self.create_publisher(Bool, 'brick_ground', 10)

        # create a timer callback to broadcast transforms at 100 Hz
        self.frequency = 100
        self.timer = self.create_timer((1/self.frequency), self.timer)

        # text marker
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

        # initialize variables
        self.max_vel = 1.0
        self.life = 0
        self.odom = "odom"
        self.brick = "brick"
        self.base = "base_link"
        self.x_brick_prev = 0
        self.y_brick_prev = 0
        self.z_brick_prev = 0
        self.F = 0
        self.marker = 0
        self.robot = RobotMove()
        self.robot.robotmove = False
        self.move = Point()
        self.base_height = 0.4
        self.stem_height = 0.2
        self.tilt_platform = Tilt()
        self.tilt_platform.tilt = 0.6
        self.brick_landed = Bool()
        self.brick_landed.data = False
        self.z_plat_bottom = 0.226
        self.z_plat = self.platform_height - (self.base_height/2
                                              ) - self.stem_height - self.wheel_rad
        self.F = 0

    def brick_to_base(self, x_brick, y_brick, z_brick, x_plat, y_plat, z_plat):
        """
        Determine if robot can catch the brick.

        Args:
            x_brick: position in x-axis (float)
            y_brick: position in y-axis (float)
            z_brick: position in z-axis (float)
            x_plat: position in x-axis (float)
            y_plat: position in y-axis (float)
            z_plat: position in z-axis (float)

        Returns
        -------
            no returns

        """
        z_diff = z_brick - z_plat
        self.get_logger().info(f'z_diff: {z_diff}')
        if z_diff > 0:
            euclid_dist = sqrt((x_brick - x_plat)**2 + (y_brick - y_plat)**2)
            t_b = sqrt(((2*(z_brick - z_plat))/self.acceleration))
            t_r = euclid_dist/self.max_vel

            if t_r <= t_b:
                # robot can catch the brick
                self.move.x = x_brick
                self.move.y = y_brick
                self.move.z = z_brick
                self.goal_pose_pub.publish(self.move)

                self.robot.robotmove = True
                self.robot.height = z_plat
                self.robot_move_pub.publish(self.robot)
                self.marker = 0

                self.pub_tilt.publish(self.tilt_platform)
                self.state = State.DETECTED

            else:
                # robot can't catch the brick
                self.z_brick_prev = -10.0
                self.marker = 1
                self.get_logger().info(f'marker txt2: {self.marker}')
                self.state = State.UNDETECTED

        else:
            self.get_logger().info(f'marker txt3: {self.marker}')
            self.z_brick_prev = -10.0
            self.marker = 1

    def timer(self):
        """
        Timer callback at 100 Hz.

        Publishes:
            goal_pose (geometry_msgs/msg/Point): goal position for robot
            move_robot (turtle_brick_interfaces/msg/RobotMove):
            see if robot moves and plat height
            tilt_plat (turtle_brick_interfaces/msg/Tilt): radians to tilt the platform
            brick_ground (std_msgs/msg/Bool): brick is off the platform
        Listeners:
            odom to base_link
            odom to brick
        Markers:
            visualization_marker (visualization_msgs/msg/Marker): text marker in /world frame

        Args:
            no arguments

        Returns
        -------
            no returns

        """
        if self.state == State.UNDETECTED:
            try:
                base_t = self.tf_buffer.lookup_transform(self.odom, self.base, rclpy.time.Time())
            except TransformException:
                return

            x_base = base_t.transform.translation.x
            y_base = base_t.transform.translation.y
            # z_base = base_t.transform.translation.z

            x_plat = x_base
            y_plat = y_base
            self.robot.height = self.z_plat

            try:
                brick_t = self.tf_buffer.lookup_transform(self.odom, self.brick, rclpy.time.Time())
            except TransformException:
                return

            x_brick = brick_t.transform.translation.x
            y_brick = brick_t.transform.translation.y
            z_brick = brick_t.transform.translation.z

            z_difference = abs(z_brick - self.z_brick_prev)

            if z_brick != self.z_brick_prev and z_difference < 0.002 and self.F == 0:
                self.brick_to_base(x_brick, y_brick, z_brick, x_plat, y_plat, self.z_plat)

            else:
                self.F = 0

            self.z_brick_prev = z_brick

            self.get_logger().info(f'marker txt: {self.marker}')
            if self.marker == 1:
                self.text.header.stamp = self.get_clock().now().to_msg()
                self.text.lifetime.sec = 3
                self.pub_text.publish(self.text)
                self.get_logger().info('HI')
            if self.life < 300:
                self.life += 1
            else:
                self.marker = 0
                self.life = 0

        elif self.state == State.DETECTED:
            self.robot.robotmove = False

            try:
                brick_t = self.tf_buffer.lookup_transform(self.odom, self.brick, rclpy.time.Time())
            except TransformException:
                return

            x_brick = brick_t.transform.translation.x
            y_brick = brick_t.transform.translation.y
            z_brick = brick_t.transform.translation.z

            x_center = 5.5
            y_center = 5.5

            x_diff = abs(x_brick - x_center)
            y_diff = abs(y_brick - y_center - 0.45)

            if z_brick == (self.z_plat - self.z_plat_bottom) and x_diff < 0.08 and y_diff < 0.91:
                self.brick_landed.data = True
                self.z_brick_prev = 0
                self.F = 1
                self.state = State.UNDETECTED

        self.robot_move_pub.publish(self.robot)
        self.brick_tilt_pub.publish(self.brick_landed)


def catcher_entry(args=None):
    rclpy.init(args=args)
    catcher = Catcher()
    rclpy.spin(catcher)
    rclpy.shutdown()
