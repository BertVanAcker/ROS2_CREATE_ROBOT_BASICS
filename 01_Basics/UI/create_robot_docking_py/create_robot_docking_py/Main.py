import threading
from threading import Event

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import sys

from enum import Enum

from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

from irobot_create_msgs.action import Undock, DockServo
from irobot_create_msgs.msg import Dock


# DOCKING state
class State(Enum):
    DOCKED = 0
    DOCK_REQUESTED = 1
    UNDOCK_REQUESTED = 2
    UNDOCKED = 3
    UNDEFINED = 4

class DockerNode(Node):
    def __init__(self):
        super().__init__('Docking_node')
        
        # -- system attributes -- #
        self.docksub_ready = False
        self.state = State.UNDEFINED
        self.status = GoalStatus.STATUS_EXECUTING

        # -- multi-threading setup -- #
        self.action_done_event = Event()    #to communicate between threads
        self.callback_group = ReentrantCallbackGroup()


        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.override_system = False
        self.lightring = LightringLeds()

        # -- input -- #
        self.subscription = self.create_subscription(InterfaceButtons, 'interface_buttons', self.button_callback, 10)
        self.subscription  # prevent unused variable warning
        self.dock_sub = self.create_subscription(Dock,'/dock',self.dockCallback,qos_profile_sensor_data)

        # -- output -- #
        self.lights_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.lights_publisher  # prevent unused variable warning

        # -- actions -- #
        self.undock_action_client = ActionClient(self, Undock, 'undock',callback_group=self.callback_group)
        self.dock_action_client = ActionClient(self, DockServo, 'dock',callback_group=self.callback_group)

        self.get_logger().info('Docking node started.')

    # ____________________________STATEMACHINE____________________________#
    def stateMachine(self):

        if self.state == State.DOCKED:
            self.setLights([self.green, self.green, self.green, self.green, self.green, self.green])
        elif self.state == State.UNDOCKED:
            self.setLights([self.blue, self.blue, self.blue, self.blue, self.blue, self.blue])
        elif self.state == State.UNDOCK_REQUESTED:
            self.setLights([self.yellow, self.yellow, self.yellow, self.yellow, self.yellow, self.yellow])
        elif self.state == State.DOCK_REQUESTED:
            self.setLights([self.cyan, self.cyan, self.cyan, self.cyan, self.cyan, self.cyan])
        else:
            self.setLights([self.red, self.red, self.red, self.red, self.red, self.red])

    # ____________________________EXECUTION____________________________#
    def run(self):

        while True:
            self.stateMachine()
            time.sleep(0.1)

    # ____________________________BUTTONS____________________________#
    def button_callback(self, msg):
        #identification of the pushed button
        if msg.button_1.is_pressed:
            self.state = State.UNDOCK_REQUESTED
            self.undock()
        if msg.button_2.is_pressed:
            self.state = State.DOCK_REQUESTED
            self.dock()

    #____________________________LIGHTS____________________________#
    def setLights(self,color):
        # -- Format message -- #
        self.override_system = True
        self.lightring.override_system = self.override_system
        self.lightring.leds = color

        # -- Publish message -- #
        self.lights_publisher.publish(self.lightring)
        
    def stopSystemOverride(self):
        # -- Format message -- #
        self.override_system = False
        self.lightring.override_system = self.override_system

        # -- Publish message -- #
        self.lights_publisher.publish(self.lightring)

    # ____________________________DOCKING____________________________#
    # Dock subscription callback
    def dockCallback(self, msg: Dock):
        self.is_docked = msg.is_docked
        if msg.is_docked:
            self.state = State.DOCKED
        else:
            self.state = State.UNDOCKED

    # Undock action
    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')
        self.state = State.UNDOCKED


    

    def dock(self):
        """Perform Undock action."""
        self.get_logger().info('Docking the robot...')

        if not self.dock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            return -1

        goal_msg = DockServo.Goal()

        self.action_done_event.clear()

        self._send_goal_future = self.dock_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for action to be done
        self.action_done_event.wait()

        self.state = State.DOCKED


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        self.status = GoalStatus.STATUS_SUCCEEDED

        # Signal that action is done
        self.action_done_event.set()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))




def main(args=None):

    rclpy.init(args=args)

    node = DockerNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    rclpy.shutdown()


    # rclpy.init(args=args)
    #
    # node = DockerNode()
    #
    # # Spin rclpy on separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # thread.start()
    #
    # # Allow time for other nodes to start
    # time.sleep(5)
    #
    # print('Running Docking node...\n')
    #
    # try:
    #     node.run()
    # except KeyboardInterrupt:
    #     pass
    #
    # node.destroy_node()
    # rclpy.shutdown()
    #
    # thread.join()





    # rclpy.init(args=args)
    #
    # node = DockerNode()
    #
    # # Spin rclpy on separate thread
    # thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # thread.start()
    #
    # # Allow time for other nodes to start
    # time.sleep(5)
    #
    # print('Running Docking node...\n')
    #
    # try:
    #     node.run()
    # except KeyboardInterrupt:
    #     pass
    #
    # node.destroy_node()
    # rclpy.shutdown()
    #
    # thread.join()

    #try:
    #    docker = DockerNode()
    #    rclpy.spin(docker)
    #except KeyboardInterrupt:
    #    print('Caught keyboard interrupt')
    #finally:
    #    # Destroy the node explicitly
    #    # (optional - otherwise it will be done automatically
    #    # when the garbage collector destroys the node object)
    #    if docker:
    #        docker.destroy_node()
    #    rclpy.shutdown()


if __name__ == '__main__':
    main()
