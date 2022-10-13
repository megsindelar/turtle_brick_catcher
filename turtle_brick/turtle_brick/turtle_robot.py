import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from math import sin, cos, sqrt, pi


def angle_axis_to_quaternion(theta, axis):
    """ Convert from angle-axis of rotation to a quaternion

        Args:
          theta:  rotation angle, in radians
          axis: the rotational axis. This will be normalized

        Returns:
          A Quaternion corresponding to the rotation
    """
    magnitude = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    normalized = [v/magnitude for v in axis]
    sinTheta2 = sin(theta/2.0)
    return Quaternion(x=normalized[0]*sinTheta2,
                      y=normalized[1]*sinTheta2,
                      z=normalized[2]*sinTheta2,
                      w=cos(theta/2.0))


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
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)   #LEFT OFF HERE
        

        world__odom_link = TransformStamped()
        world__odom_link.header.stamp = self.get_clock().now().to_msg()
        world__odom_link.header.frame_id = "world"
        world__odom_link.child_frame_id = "odom"
        world__odom_link.transform.translation.z = 1.0          #needs to be turtles initial conditions
        self.static_broadcaster.sendTransform(world__odom_link)

        #create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)
        #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.timer)

        self.dx = 4

    def timer(self):

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

        odom__base_link = TransformStamped()

        time = self.get_clock().now().to_msg()
        odom__base_link.header.frame_id = "odom"
        odom__base_link.child_frame_id = "base_link"
        odom__base_link.header.stamp = time
        odom__base_link.transform.translation.x = float(self.dx)


        self.broadcaster.sendTransform(odom__base_link)


def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    rclpy.shutdown()