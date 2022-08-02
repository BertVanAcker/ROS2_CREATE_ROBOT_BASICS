import rclpy
from rclpy.node import Node
import sys

from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

class ColorPalette():
    """ Helper Class to define frequently used colors"""
    def __init__(self):
        self.red = LedColor(red=255,green=0,blue=0)
        self.green = LedColor(red=0,green=255,blue=0)
        self.blue = LedColor(red=0,green=0,blue=255)
        self.yellow = LedColor(red=255,green=255,blue=0)
        self.pink = LedColor(red=255,green=0,blue=255)
        self.cyan = LedColor(red=0,green=255,blue=255)
        self.purple = LedColor(red=127,green=0,blue=255)
        self.white = LedColor(red=255,green=255,blue=255)
        self.grey = LedColor(red=189,green=189,blue=189)

class RobotLights():
    """ Helper Class to use the robot leds"""
    def __init__(self):
        # -- system attributes -- #
        self.override_system = False
        self.lightring = LightringLeds()

        # -- output -- #
        self.lights_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)

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


class LightingNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # -- inputs -- #
        self.subscription = self.create_subscription(InterfaceButtons,'interface_buttons',self.button_callback,10)
        self.subscription  # prevent unused variable warning

        # -- outputs -- #
        self.robotLights = RobotLights()

        # -- node-specific -- #
        self.colors = ColorPalette()

    def button_callback(self, msg):
        #identification of the pushed button
        if msg.button_1.is_pressed:
            self.robotLights.setLights(self.colors.green)
        if msg.button_2.is_pressed:
            self.robotLights.setLights(self.colors.red)
        if msg.button_power.is_pressed:
            #return led control to robot
            self.robotLights.stopSystemOverride()


def main(args=None):
    rclpy.init(args=args)

    Lights=None
    try:
        Lights = LightingNode()
        rclpy.spin(Lights)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception in dance:', file=sys.stderr)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if Lights:
            Lights.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()