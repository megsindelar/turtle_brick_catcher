from sklearn.metrics import euclidean_distances
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from math import sqrt

class Catcher(Node):
    """ Move robot link/joint frames in space
    
        Static Broadcasters:

        Broadcasters:
    """
    def __init__(self):
        super().__init__('catcher')        

        # #create a listener for brick position  
        # self.tf_buffer = Buffer()  
        # self.tf_brick_listener = TransformListener(self.tf_buffer, self)

        # #create a listener for base_link position  
        # self.tf_buffer2 = Buffer()  
        # self.tf_base_listener = TransformListener(self.tf_buffer2, self)

        # #create a timer callback to broadcast transforms at 100 Hz
        # self.timer = self.create_timer(0.01, self.timer)

        self.max_vel = 0.22

    # def brick_base(self,x_brick,y_brick,z_brick,x_plat,y_plat,z_plat):
    #     euclid_dist = sqrt((x_brick - x_plat)**2 + (y_brick - y_plat)**2)


    # def timer(self):
    #     brick_t = self.tf_buffer.lookup_transform("brick","odom",Time)
    #     x_t_brick = brick_t.transform.translation.x
    #     y_t_brick = brick_t.transform.translation.y
    #     z_t_brick = brick_t.transform.translation.z

    #     base_t = self.tf_buffer2.lookup_transform("base_link","odom",Time)
    #     x_t_base = base_t.transform.translation.x
    #     y_t_base = base_t.transform.translation.y
    #     z_t_base = base_t.transform.translation.z + 0.85       #adding the height to the top of the platform



def catcher_entry(args=None):
    rclpy.init(args=args)
    catcher = Catcher()
    rclpy.spin(catcher)
    rclpy.shutdown()