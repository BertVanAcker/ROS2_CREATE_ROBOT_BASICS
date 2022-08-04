import rclpy
from rclpy.node import Node
from enum import Enum

from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import KidnapStatus,HazardDetectionVector,IrIntensityVector
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default




class KidnapDetector(Node):

    def __init__(self):
        super().__init__('kidnap_detector')
        self.subscription = self.create_subscription(KidnapStatus,'kidnap_status',self.kidnap_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def kidnap_callback(self, msg):
        #identification of kidnap action (manual lift of robot)
        if msg.is_kidnapped:
            self.get_logger().info('Robot is kidnaped')

class HazardDetector(Node):

    def __init__(self):
        super().__init__('Hazard_detector')
        self.subscription = self.create_subscription(HazardDetectionVector,'hazard_detection',self.hazard_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def hazard_callback(self, msg):

        #identification of the robot hazards
        for hazard in msg.detections:

            if hazard.type == 1:
                self.get_logger().info('Robot is bumped')
            elif hazard.type == 2:
                self.get_logger().info('Robot detected a cliff')
            elif hazard.type == 3:
                self.get_logger().info('Robot motors are stalled')
            elif hazard.type == 4:
                self.get_logger().info('Robot wheels are not touching the ground')
            elif hazard.type == 5:
                self.get_logger().info('Robot detects an object very close')

class ProximityDetector(Node):

    def __init__(self):
        super().__init__('Proximity_detector')
        self.subscription = self.create_subscription(IrIntensityVector,'ir_intensity',self.proximity_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def proximity_callback(self, msg):
        #identification of the proximity per sensor
        for i in range(len(msg.readings)):
            self.get_logger().info("Sensor ["+i.__str__()+'] - proximity level: '+msg.readings[i].value.__str__())



def main(args=None):
    rclpy.init(args=args)

    hazarddetector = HazardDetector()
    #proximityDetector = ProximityDetector()
    kidnapDetector = KidnapDetector()

    rclpy.spin(hazarddetector)
    #rclpy.spin(proximityDetector)
    rclpy.spin(kidnapDetector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hazarddetector.destroy_node()
    #proximityDetector.destroy_node()
    kidnapDetector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()