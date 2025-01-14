import rclpy
from rclpy.node import Node
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import MotorCommand

class TestMotorNode(Node): #replaces state machine, which may be complicated to debug, for testing
    def __init__(self):
        super().__init__('test_motor_subscriber')
        self.cube_loc_sub = self.create_subscription(CubeTracking, "cube_location_info", self.motor_callback, 10)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10) 
        
    def motor_callback(self, msg: CubeTracking):
        #for testing:
        #if bb is on left of screen, turn left, if on right, turn right
        #if far, move faster
        speed = int(msg.distance*5/2-25) #0 -> 50 at range 10cm to 30cm
        if speed > 50:
            speed = 100
        motor_msg = MotorCommand()
        motor_msg.left_speed = speed
        motor_msg.right_speed = speed
        print(motor_msg.left_speed, motor_msg.right_speed)
        self.motor_pub.publish(motor_msg)
    
def main(args=None):
    rclpy.init()
    node = TestMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
