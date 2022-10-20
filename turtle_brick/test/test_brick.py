import unittest
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import rclpy
from geometry_msgs.msg import Twist
import time

@pytest.mark.rostest
def generate_test_description():
    turtle_robot_action = Node(package="turtle_brick", executable="turtle_robot")
    return (
        LaunchDescription([
            turtle_robot_action,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'turtle_robot': turtle_robot_action
        }
    )

class TestTurtleRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_cmd_vel_pub_rate(self):
        msg_list = []
        self.cmd_vel_sub = self.node.create_subscription(Twist,"turtle1/cmd_vel",lambda msg: msg_list.append(msg), 10)

        try:
            end_time = time.time() + 5
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                #if len(msg_list) > 100:
                #break
            
            self.freq = 5/len(msg_list)

            self.assertAlmostEqual(self.freq, 0.01, 2)

        finally:
            self.tearDown()
