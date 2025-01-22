import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import MotorCommand

class TestMotorNode(Node): #replaces state machine, which may be complicated to debug, for testing
    def __init__(self):
        super().__init__('test_motor_subscriber')
        self.clock = Clock()
        self.prev_time = self.clock.now()
        self.i = 0

        self.cube_loc_sub = self.create_subscription(CubeTracking, "cube_location_info", self.motor_callback, 10)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10) 
        
    def motor_callback(self, msg: CubeTracking):
        #for testing:
        #DC
        speed = 0 #int(msg.distance*5/2-25) #0 -> 50 at range 10cm to 30cm
        if speed > 50:
            speed = 100
        motor_msg = MotorCommand()

        motor_msg.drive_motors.left_speed = speed
        motor_msg.drive_motors.right_speed = speed
        #print(motor_msg.left_speed, motor_msg.right_speed)

        if (self.clock.now() - self.prev_time).nanoseconds/1e9 > 3:
            self.i = (self.i + 1)%2
            self.prev_time = self.clock.now()
        if self.i == 0:
            motor_msg.actuate_motors.angle1 = -90
            motor_msg.actuate_motors.angle2 = -40
        else:
            motor_msg.actuate_motors.angle1 = 90
            motor_msg.actuate_motors.angle2 = -25

        self.motor_pub.publish(motor_msg)
    
def main(args=None):
    rclpy.init()
    node = TestMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
