import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import InterfaceButtons



class ButtonSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(InterfaceButtons,'interface_buttons',self.button_callback,10)
        self.subscription  # prevent unused variable warning

    def button_callback(self, msg):
        #identification of the pushed button
        if msg.button_1.is_pressed:
            self.get_logger().info(
                'Button 1 pressed for "%s" nanoseconds' % msg.button_1.last_pressed_duration.__str__())
        if msg.button_2.is_pressed:
            self.get_logger().info(
                'Button 2 pressed for "%s" nanoseconds' % msg.button_1.last_pressed_duration.__str__())
        if msg.button_power.is_pressed:
            self.get_logger().info(
                'Button power pressed for "%s" nanoseconds' % msg.button_1.last_pressed_duration.__str__())


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ButtonSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()