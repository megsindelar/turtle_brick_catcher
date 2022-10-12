import rclpy
from rclpy import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from math import sin, cos, sqrt, pi

class Transform():
    """ A class used to convert from rotation axis to quaternion

    """
    def __init__(self, theta, axis):
        self.t = theta
        self.a = axis
    
    def quaternion(self,):
        mag = sqrt(self.a[0]**2 + self.a[1]**2 + self.a[2]**2)
        for i in self.a:
            norm = i/mag
        sintheta2 = sin(self.t/2.0)
        quaternion = Quaternion(x=norm[0]*sin,y=...,z=...,w=cos(self.t/2.0))


class Robot(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super.__init__('turtle_robot')
        #create a static broadcaster which will publish once to /tf_static
        self.static_broadcaster = StaticTransformBroadcaster(self)
        #create a broadcaster that will repeatedly publish to /tf
        self.broadcaster = TransformBroadcaster(self)
        #create a timer callback to broadcast transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.timer)
        #create a robot_state_publisher
        """self.robot_state_publisher = self.create_publisher()"""   #LEFT OFF HERE

    def timer(self):
        world__base_link = TransformStamped()
        world__base_link.header.stamp = self.get_clock().now().to_msg()
        world__base_link.header.frame_id = "world"
        world__base_link.child_frame_id = "base"
        world__base_link.transform.translation.z = 1.0



        # timestamp for transforms
        time = self.get_clock().now().to_msg()
        world__base_link.header.stamp = time


def main(args=None):
    rclpy.init(args=args)
    robot = Robot
    rclpy.spin(robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
