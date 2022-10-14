from matplotlib.pyplot import angle_spectrum
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from turtlesim.msg import Pose
from sensor_msgs.msg import JointState
from .quaternion import angle_axis_to_quaternion
from std_msgs.msg import Bool


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

        #create a subscriber to turtle_pose
        self.turtle_pose = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)

        #create a publisher for cmd_vel   
        self.cmd_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        #create a subscriber to goal_pose
        self.goal_pose_sub = self.create_subscription(Bool,"goal_pose",self.goal_pose_callback,10)

        #create a listener for brick position  
        self.tf_buffer = Buffer()  
        self.tf_brick_listener = TransformListener(self.tf_buffer, self)

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

    def turtle_pose_callback(self,msg):
        self.pose = msg

    def goal_pose_callback(self,msg):
        self.goal = msg

    def timer(self):
        
        if self.i<8:
            self.lx += 1.0
            self.i+=1

        move = Twist(linear = Vector3(x = float(self.lx), y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 1.0))
        self.cmd_vel_pub.publish(move)

        odom__base_link = TransformStamped()

        """NOTEE TO SELF
        
        maybe use a async function with waiting for future to wait for a turtlesim pose to then set odom frame
        """

        if self.count == 30:
            time = self.get_clock().now().to_msg()
            odom__base_link.header.frame_id = "odom"
            odom__base_link.child_frame_id = "base_link"
            odom__base_link.header.stamp = time
            odom__base_link.transform.translation.x = float(self.pose.x)
            odom__base_link.transform.translation.y = float(self.pose.y)


            self.broadcaster.sendTransform(odom__base_link)
        else:
            self.count+=1

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