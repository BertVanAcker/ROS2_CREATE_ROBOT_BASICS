import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import sys

from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

from irobot_create_msgs.action import Undock, DockServo
from irobot_create_msgs.msg import Dock




class DockerNode(Node):
    def __init__(self):
        super().__init__('Docking_node')
        self.get_logger().info('Node setup start')
        # -- system attributes -- #

        # -- input -- #
        self.subscription = self.create_subscription(InterfaceButtons, 'interface_buttons', self.button_callback, 10)
        self.subscription  # prevent unused variable warning
        self.dock_sub = self.create_subscription(Dock,'/dock',self.dockCallback,qos_profile_sensor_data)

        # -- output -- #

        # -- actions -- #
        self.undock_action_client = ActionClient(self, Undock, '/undock')
        self.dock_action_client = ActionClient(self, DockServo, '/dock')

        self.get_logger().info('Node setup stop')




    # Dock subscription callback
    def dockCallback(self, msg: Dock):
        self.is_docked = msg.is_docked

    # Undock action
    def undock(self):
        if self.is_docked:
            self.get_logger().info('Undocking the robot')
            self.undock_action_client.wait_for_server()
            undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
            if undock_goal_result.result.is_docked:
                self.get_logger().info('Undocking of the robot FAILED')
        else:
            self.get_logger().info('Undocking of the robot FAILED - Robot is not docked')

    # Undock action
    def dock(self):
        if not self.is_docked:
            self.get_logger().info('Docking the robot')
            self.dock_action_client.wait_for_server()
            dock_goal_result = self.dock_action_client.send_goal(DockServo.Goal())
            if dock_goal_result.result.is_docked:
                self.get_logger().info('Docking of the robot FAILED')
        else:
            self.get_logger().info('Docking of the robot FAILED - Robot is already docked')

    def button_callback(self, msg):
        #identification of the pushed button
        if msg.button_1.is_pressed:
            self.undock()
        if msg.button_2.is_pressed:
            self.dock()


def main(args=None):
    rclpy.init(args=args)

    try:
        docker = DockerNode()
        rclpy.spin(docker)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if docker:
            docker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()