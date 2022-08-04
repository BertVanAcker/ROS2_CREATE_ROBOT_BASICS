import rclpy
from rclpy.node import Node
from enum import Enum

from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import KidnapStatus,HazardDetectionVector,IrIntensityVector




class HAZARDS(Enum):
    # The Create3 webserver can be used to set a default value for the parameter.
    BACKUP_LIMIT = 0
    # The robot has bumped against an obstacle
    BUMP = 1
    # The robot detected a cliff
    CLIFF = 2
    # The wheels of the robot are stalled against an obstacle
    STALL = 3
    # The wheels of the robot are fully dropped
    WHEEL_DROP = 4
    # The robot detects an obstacle in close proximity
    OBJECT_PROXIMITY = 5


class KidnapDetector(Node):

    def __init__(self):
        super().__init__('kidnap_detector')
        self.subscription = self.create_subscription(KidnapStatus,'kidnap_status',self.kidnap_callback,10)
        self.subscription  # prevent unused variable warning

    def kidnap_callback(self, msg):
        #identification of kidnap action (manual lift of robot)
        if msg.is_kidnapped:
            self.get_logger().info('Robot is kidnaped')

class HazardDetector(Node):

    def __init__(self):
        super().__init__('Hazard_detector')
        self.subscription = self.create_subscription(HazardDetectionVector,'hazard_detection',self.hazard_callback,10)
        self.subscription  # prevent unused variable warning

    def hazard_callback(self, msg):
        #identification of the robot hazards
        for hazard in msg.HazardDetection:
            if hazard.type == HAZARDS.BACKUP_LIMIT:
                self.get_logger().info('Robot is in backup limit')
            elif hazard.type == HAZARDS.BUMP:
                self.get_logger().info('Robot is bumped')
            elif hazard.type == HAZARDS.CLIFF:
                self.get_logger().info('Robot detected a cliff')
            elif hazard.type == HAZARDS.STALL:
                self.get_logger().info('Robot motors are stalled')
            elif hazard.type == HAZARDS.WHEEL_DROP:
                self.get_logger().info('Robot wheels are not touching the ground')
            elif hazard.type == HAZARDS.OBJECT_PROXIMITY:
                self.get_logger().info('Robot detects an object very close')

class ProximityDetector(Node):

    def __init__(self):
        super().__init__('Proximity_detector')
        self.subscription = self.create_subscription(IrIntensityVector,'ir_intensity',self.proximity_callback,10)
        self.subscription  # prevent unused variable warning

    def proximity_callback(self, msg):
        #identification of the proximity per sensor
        for i in range(len(msg.IrIntensity)):
            self.get_logger().info("Sensor ["+i.__str__()+'] - proximity level: '+msg.IrIntensity.value.__str__())



def main(args=None):
    rclpy.init(args=args)

    #hazardDetector = HazardDetector()
    #proximityDetector = ProximityDetector()
    kidnapDetector = KidnapDetector

    #rclpy.spin(hazardDetector)
    #rclpy.spin(proximityDetector)
    rclpy.spin(kidnapDetector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #hazardDetector.destroy_node()
    #proximityDetector.destroy_node()
    kidnapDetector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()