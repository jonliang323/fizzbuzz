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
        speed = int(msg.distance*10)#2.3 -> 23
        l_sign = -1 if msg.x_center < 680/2 else 1
        r_sign = 1 if msg.x_center < 680/2 else -1
        motor_msg = MotorCommand()
        motor_msg.left_speed = l_sign*speed
        motor_msg.right_speed = r_sign*speed
        self.motor_pub.publish(motor_msg)
    
def main(args=None):
    rclpy.init()
    node = TestMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()