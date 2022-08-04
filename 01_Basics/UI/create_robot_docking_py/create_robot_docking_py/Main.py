import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
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
        self.undock_action_client = ActionClient(self, Undock, '/undock')
        self.dock_action_client = ActionClient(self, DockServo, '/dock')

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
        """Perform Undock action."""
        self.get_logger().info('Undocking the robot...')
        self.undock_send_goal()

            
    def undock_send_goal(self):
        self.get_logger().info('Sending goal...')
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        self.goal_future = self.undock_action_client.send_goal_async(goal_msg)
        
        self.setLights([self.yellow, self.yellow, self.yellow, self.yellow, self.yellow, self.yellow])
        while rclpy.ok() and not self.goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.get_logger().info("Wating for action to finish")

        self.state = State.UNDOCKED
    

    def dock(self):
        """Perform Undock action."""
        self.get_logger().info('Docking the robot...')
        self.dock_send_goal()

        while not self.isDockComplete():
            time.sleep(0.1)

        self.state = State.DOCKED

    def dock_send_goal(self):
        goal_msg = DockServo.Goal()
        self.dock_action_client.wait_for_server()
        goal_future = self.dock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.dock_goal_handle = goal_future.result()

        if not self.dock_goal_handle.accepted:
            self.get_logger().info('Docking of the robot FAILED - Dock goal rejected')
            return

        self.dock_result_future = self.dock_goal_handle.get_result_async()

    def isDockComplete(self):
        """
        Get status of Dock action.
        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.dock_result_future is None or not self.dock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.dock_result_future, timeout_sec=0.1)

        if self.dock_result_future.result():
            self.dock_status = self.dock_result_future.result().status
            if self.dock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

       
        self.get_logger().info('Docking of the robot SUCCEEDED')
        return True



def main(args=None):
    rclpy.init(args=args)

    node = DockerNode()

    # Spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Allow time for other nodes to start
    time.sleep(5)

    print('Running FollowBot...\n')

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    thread.join()

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
