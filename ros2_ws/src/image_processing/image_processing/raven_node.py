import rclpy
from rclpy.node import Node
from raven import Raven
#from icm42688 import ICM42688
#import board
from image_processing_interfaces.msg import MotorCommand
from image_processing_interfaces.msg import EncoderCounts
#from image_processing_interfaces.msg import IMUInfo

class RavenNode(Node):
    def __init__(self):
        super().__init__('raven_subscriber')
        self.raven_board = Raven()
        #imu initialization
        #spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        #while not spi.try_lock():
            #pass

        #spi.configure(baudrate=5000000)

        #self.imu = ICM42688(spi)
        #self.imu.begin()

        # Set motors to DIRECT
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH2, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 50) # Let the motor use 50% max torque to get to speed factor
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH2, 50)

        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0) # Set encoder count for motor 1 to zero
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH2, 0) # Set encoder count for motor 1 to zero

        self.motor_sub = self.create_subscription(MotorCommand, "motor_command", self.raven_callback, 10)
        self.encoder_pub = self.create_publisher(EncoderCounts, "encoder_info", 10)

    def raven_callback(self, msg: MotorCommand):
        dc = msg.drive_motors
        servo = msg.actuate_motors
        rev = False
        if dc.left_speed < 0:
            left_speed = abs(dc.left_speed)
            left_rev = True
        else:
            left_speed = dc.left_speed
            left_rev = False
        
        if dc.right_speed < 0:
            right_speed = abs(dc.right_speed)
            right_rev=True
        else:
            right_speed = dc.right_speed
            right_rev=False

        # print(f"Left Motor - Speed: {left_speed}, Reverse: {left_rev}")
        # print(f"Right Motor - Speed: {right_speed}, Reverse: {right_rev}")

        # Speed controlled:
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, left_speed, reverse=left_rev)
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH2, right_speed, reverse=right_rev)

        # Torque controlled:
        # self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 100) # Make motor try to run at max speed forward
        # self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 10) # Let it use up to 10% available torque

        # Set the servo 1 to x degrees with custom pulse microseconds
        # self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle, servo.min_us, servo.max_us)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle1, 500, 2500)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH2, servo.angle2, 500, 2500)

        encoder_msg = EncoderCounts()
        encoder_msg.encoder1 = self.get_raven_board.motor_encoder(Raven.MotorChannel.CH1, 0)
        encoder_msg.encoder2 = self.get_raven_board.motor_encoder(Raven.MotorChannel.CH2, 0)
        self.encoder_pub.publish(encoder_msg)


def main(args=None):
    rclpy.init()
    node = RavenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def destroy_node(self):
    # Stop the motors before shutting down
    self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 0, reverse=False)
    self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH2, 0, reverse=False)
    super().destroy_node()  # Call the parent class destroy_node


if __name__ == "__main__":
    main()
