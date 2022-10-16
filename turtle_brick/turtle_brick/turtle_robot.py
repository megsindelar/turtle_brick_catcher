from matplotlib.pyplot import angle_spectrum
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Point
from turtlesim.msg import Pose
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion
from std_msgs.msg import Bool
from turtle_brick_interfaces.msg import RobotMove

class Robot(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('turtle_robot')
        #create a static broadcaster which will publish once to /tf_static
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        #create a joint_state_publisher
        #self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)   #LEFT OFF HERE

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

        #create a listener for brick position  
        # self.tf_buffer = Buffer()  
        # self.tf_brick_listener = TransformListener(self.tf_buffer, self)

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

        #create a subscriber to turtle_pose
        self.turtle_pose = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)

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

            if self.diff_x > 0:
                self.move.linear.x = self.max_vel
            elif self.diff_x < 0:
                self.move.linear.x = -self.max_vel
            else:
                self.move.linear.x = 0.0
                
            if self.diff_y > 0:
                self.move.linear.y = self.max_vel
            elif self.diff_y < 0:
                self.move.linear.y = -self.max_vel
            else:
                self.move.linear.y = 0.0

            self.lin_x = self.move.linear.x
            self.lin_y = self.move.linear.y

            odom__base_link.transform.translation.x = float(self.pose.x)
            odom__base_link.transform.translation.y = float(self.pose.y)

            self.abs_diff_x = abs(self.goal.x - self.pose.x)
            self.abs_diff_y = abs(self.goal.y - self.pose.y)

            if self.abs_diff_x < 0.01 and self.abs_diff_y < 0.01:
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

                self.targ.data = True

                x_center = 5.5
                y_center = 5.5
                difference_x = abs(self.pose.x - x_center)
                difference_y = abs(self.pose.y - y_center)

                if difference_x > 0.01:
                    self.move.linear.x = -self.lin_x
                else:
                    self.move.linear.x = 0.0

                if difference_y > 0.01:
                    self.move.linear.y = -self.lin_y
                else:
                    self.move.linear.y = 0.0

                if self.move.linear.x == 0.0 and self.move.linear.y == 0.0:
                    self.wait = 0

                self.get_logger().info(f'diff_x: {difference_x}')
                self.get_logger().info(f'diff_y: {difference_y}')
                self.get_logger().info(f'turtle_x: {self.move.linear.x}')
                self.get_logger().info(f'turtle_y: {self.move.linear.y}')

                odom__base_link.transform.translation.x = float(self.pose.x)
                odom__base_link.transform.translation.y = float(self.pose.y)

        # elif self.move_robot_now == 0 and self.wait == 1:
        #     self.move.linear.x = 0.0
        #     self.move.linear.y = 0.0
        #     odom__base_link.transform.translation.x = float(self.pose.x)
        #     odom__base_link.transform.translation.y = float(self.pose.y)

        else:
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0

            odom__base_link.transform.translation.x = 5.5
            odom__base_link.transform.translation.y = 5.5

            #self.get_logger().info("Not Moving!")


        self.get_logger().info(f'move_turtle: {self.move}')
        self.cmd_vel_pub.publish(self.move)
        self.hit_targ_pub.publish(self.targ)

        """NOTEE TO SELF
        
        maybe use a async function with waiting for future to wait for a turtlesim pose to then set odom frame
        """

        #if self.count == 30:
    

        self.broadcaster.sendTransform(odom__base_link)
        #else:
        #    self.count+=1

        #publish joint states to test
        # joint = ["base_to_stem"]
        # pose = float(0.5)
        # vel = float(0.5)
        # eff = float(0)
        # move = JointState(name = joint, position = pose, velocity = vel, effort = eff)
        # self.joint_state_publisher.publish(move)

        # joint = ["base_to_stem"]
        # pose = float(-0.5)
        # vel = float(-0.5)
        # eff = float(0)
        # move = JointState(name = joint, position = pose, velocity = vel, effort = eff)
        # self.joint_state_publisher.publish(move)
 
        #broadcast base_link constantly





def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    rclpy.shutdown()