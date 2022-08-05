import math
import threading
from enum import Enum
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# -- robot specific -- #
from irobot_create_msgs.msg import KidnapStatus,HazardDetectionVector,IrIntensityVector
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import Dock
from geometry_msgs.msg import Twist


from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock


# FollowBot state
class State(Enum):
    DOCKED = 0
    DRIVING = 1
    BUMPED = 2
    SEARCHING_CLEARPATH = 3
    UNKNOWN = 4

class BaseMotion(Node):

    def __init__(self):
        super().__init__('Base_Motion')

        # -- system attributes -- #
        self.is_docked = False
        self.state = State.UNKNOWN
        self.safeDistance = 800

        # -- multi-threading setup -- #


        # -- input -- #
        dock_sub = self.create_subscription(Dock,'/dock', self.dockCallback,qos_profile_sensor_data)

        self.subscription = self.create_subscription(HazardDetectionVector, 'hazard_detection', self.hazard_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(IrIntensityVector, 'ir_intensity', self.proximity_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # -- output -- #
        self.lights_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)

        # -- actions -- #
        self.undock_action_client = ActionClient(self, Undock, '/undock')

        self.get_logger().info('Base motion node started.')

    # Dock subscription callback
    def dockCallback(self, msg: Dock):
        if msg.is_docked:
            self.state=State.DOCKED
        self.is_docked = msg.is_docked

    def hazard_callback(self, msg):
        #identification of the robot hazards
        for hazard in msg.detections:
            if hazard.type == 1:
                self.get_logger().info('Robot is bumped')
                self.state = State.BUMPED

    def proximity_callback(self, msg):
        #determine if clearpath or not
        clearPath = True
        for i in range(len(msg.readings)):
            if msg.readings[i].value> self.safeDistance:    #one of the sensors closer than safeDistance, search for free path
                self.get_logger().info("Sensor [" + i.__str__() + '] - proximity level: ' + msg.readings[i].value.__str__())
                clearPath = False
                self.state = State.SEARCHING_CLEARPATH
        if clearPath:
            self.state = State.DRIVING
        else:
            self.state = State.SEARCHING_CLEARPATH

    # Send cmd_vel to Create 3
    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x

        self.cmd_vel_pub.publish(msg)

    #command the robot to stop
    def stop(self):
        self.drive(0.0, 0.0)

    # Undock action
    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')

    # FollowBot state machine
    def stateMachine(self):
        # Searching
        if self.state == State.DOCKED:
            #do something when docked
            print('Robot is docked')
        elif self.state == State.DRIVING:
            #continously driving straight
            self.drive(0.3, 0.0)
        elif self.state == State.SEARCHING_CLEARPATH:
            #continously turning slowly
            self.drive(0.0, 0.5)
        elif self.state == State.BUMPED:
            #backup a bit and turn right
            self.drive(-0.3, 0.0)
            time.sleep(1)
            self.drive(0.0, 0.6)
            time.sleep(1)
            self.state = State.DRIVING

    def run(self):
        # Undock first
        if self.is_docked:
            print('Undocking')
            self.undock()
            self.state = State.DRIVING
        else:
            self.state = State.DRIVING

        while True:
            self.stateMachine()
            print('{0: <20}'.format(self.state), end='\r')
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)

    node = BaseMotion()

    # Spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Allow time for other nodes to start
    time.sleep(5)

    print('Running base motion node...\n')

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    thread.join()


if __name__ == '__main__':
    main()